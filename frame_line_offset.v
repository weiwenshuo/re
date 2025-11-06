// =========================================================
// DDR3行数据搬移模块 - 使用同步FIFO
// 功能：从源地址读取一行数据，通过FIFO缓冲，写入目标地址
// =========================================================
module ddr3_line_mover #(
    parameter FRAME0_BASE_ADDR = 28'd0,           // 第一帧基地址
    parameter FRAME1_BASE_ADDR = 28'd921600,      // 第二帧基地址
    parameter FRAME2_BASE_ADDR = 28'd1843200,     // 第三帧基地址
    parameter DST_BASE_ADDR    = 28'd2764800 + 28'd307200,     // 压缩后目标基地址
    parameter IMG_WIDTH        = 1280,            // 图像宽度（像素）
    parameter IMG_HEIGHT       = 720              // 图像高度
)(
    // 系统时钟和复位
    input               clk,                  // 系统时钟
    input               rst_n,                // 系统复位
    
    // 控制接口
    input               start,                // 启动信号（单脉冲）
    output              idle,                 // 空闲状态
    output              line_done,            // 行处理完成
    output              frame_done_out,       // 帧处理完成
    
    // DDR3控制接口
    input               init_calib_complete,  //
    input               ddr3_cmd_ready,       // DDR3命令就绪
    output reg          ddr3_cmd_en,          // DDR3命令使能
    output reg [2:0]    ddr3_cmd,             // DDR3命令
    output reg [27:0]   ddr3_addr,            // DDR3地址
    input               ddr3_wr_rdy,          // DDR3写数据就绪
    output reg          ddr3_wr_en,           // DDR3写使能
    output reg          ddr3_wr_end,          // DDR3写结束
    output reg [127:0]  ddr3_wr_data,         // DDR3写数据
    output reg [15:0]   ddr3_wr_mask,         // DDR3写掩码
    input               ddr3_rd_data_valid,   // DDR3读数据有效
    input [127:0]       ddr3_rd_data,         // DDR3读数据
    input               ddr3_rd_data_end,     // DDR3读数据结束
    
    // FIFO状态监控（可选）
    output [8:0]        fifo_wr_count         // FIFO写入数据计数
);


// =========================================================
// 参数定义
// =========================================================
localparam PIXELS_PER_LINE  = 11'd480;      // 每行像素数
localparam BURSTS_PER_LINE  = PIXELS_PER_LINE / 8;  // 每行突发次数（128位=8像素）60
localparam TOTAL_LINES      = IMG_HEIGHT * IMG_WIDTH / PIXELS_PER_LINE;     // 总行数
localparam ADDR_INCREMENT   = 8;              // 每次地址增量（字节）
localparam WR_PER_LINE      = BURSTS_PER_LINE / 3; // 每行写入次数（每3个突发取1个突发）20
// =========================================================
// 内部信号定义
// =========================================================
// 状态机
localparam S_IDLE   = 3'd0;
localparam S_READ   = 3'd1;
localparam S_WAIT   = 3'd2;
localparam S_WRITE  = 3'd3;
localparam S_WAIT_WRITE = 3'd4;
localparam S_DONE   = 3'd5;
localparam S_COMPRESS = 3'd6;
reg [2:0] state, next_state;

// 地址和计数器
reg [27:0] current_src_addr;   // 当前源地址
reg [27:0] current_dst_addr;   // 当前目标地址
reg [15:0] line_count;         // 行计数
reg [27:0] src_line_base;      // 源行基地址
reg [27:0] dst_line_base;      // 目标行基地址
reg [7:0]  read_cmd_cnt;       // 读命令计数器
reg [15:0] delay_count;        // 延时计数器
reg [7:0]  rd_data_valid_cnt;  // 有效读数据计数器

// FIFO接口信号
wire [127:0] fifo_data_in;     // FIFO输入数据
wire         fifo_wr_en;       // FIFO写使能
wire [127:0] fifo_data_out;    // FIFO输出数据
wire         fifo_rd_en;       // FIFO读使能
wire         fifo_empty;       // FIFO空标志
wire         fifo_full;        // FIFO满标志
wire         fifo_almost_empty; // FIFO半空标志
wire         fifo_almost_full;  // FIFO半满标志
wire [8:0]   fifo_wr_num;      // FIFO写入数据计数

// 控制信号
reg processing_line;           // 行处理中标志
reg line_complete;             // 行完成标志
reg module_idle;               // 模块空闲标志
reg frame_done;                // 帧完成标志
assign frame_done_out = frame_done;

// 拼接数据定义
reg [127:0] compressed_data;      // 压缩后数据
reg [127:0] compressed_data_cached; // 压缩后数据缓存
reg compressed_data_valid;     // 压缩数据有效标志
wire [7:0] read_cnt_base;
wire [7:0] rd_data_valid_cnt_base;
wire [7:0] line_cnt_base;
wire line_jump; // 行跳转标志

assign line_cnt_base = (line_count % 8); // 当前行在8行组内的位置
assign read_cnt_base = (read_cmd_cnt % 3); // 当前读命令在行内的位置
assign rd_data_valid_cnt_base = (rd_data_valid_cnt % 3);
assign line_jump = (line_cnt_base == 8'd2 && read_cmd_cnt == 8'd39) || 
                   (line_cnt_base == 8'd5 && read_cmd_cnt == 8'd19) ||
                   (line_cnt_base == 8'd7 && read_cmd_cnt == 8'd59); // 每8行的最后一个读命令后跳转
// =========================================================
// 同步FIFO实例化
// =========================================================
fifo_sc_top fifo_inst(
    .Data(fifo_data_in),        // input [127:0] Data
    .Clk(clk),                  // input Clk
    .WrEn(fifo_wr_en),          // input WrEn
    .RdEn(fifo_rd_en),          // input RdEn
    .Reset(~rst_n),             // input Reset
    .Wnum(fifo_wr_num),         // output [8:0] Wnum
    .Almost_Empty(fifo_almost_empty), // output Almost_Empty
    .Almost_Full(fifo_almost_full),   // output Almost_Full
    .Q(fifo_data_out),          // output [127:0] Q
    .Empty(fifo_empty),         // output Empty
    .Full(fifo_full)            // output Full
);

// FIFO控制信号连接
assign fifo_data_in = compressed_data_cached;     // DDR3读取数据直接写入FIFO
assign fifo_wr_en = ddr3_rd_data_valid && compressed_data_valid && (fifo_wr_num < WR_PER_LINE); // DDR3读取有效时写入FIFO
assign fifo_rd_en = (state == S_WRITE) && ddr3_wr_rdy && ddr3_cmd_ready && !fifo_empty;
assign fifo_wr_count = fifo_wr_num;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_data_valid_cnt <= 8'd0;
        compressed_data_valid <= 1'b0;
        compressed_data_cached <= 128'd0;
        compressed_data <= 128'd0;
    end else begin
        
        if (ddr3_rd_data_valid && (state == S_READ)) begin
            compressed_data_valid <= 1'b0; // 默认无效
            case (rd_data_valid_cnt_base)
                8'd0: begin
                    compressed_data[15:0] <= ddr3_rd_data[15:0];
                    compressed_data[31:16] <= ddr3_rd_data[63:48];
                    compressed_data[47:32] <= ddr3_rd_data[111:96];
                    if (read_cmd_cnt > 0)begin
                        compressed_data_cached <= compressed_data;
                        compressed_data_valid <= 1'b1;     // 压缩数据有效标志
                    end
                end
                8'd1: begin
                    compressed_data[63:48] <= ddr3_rd_data[31:16];
                    compressed_data[79:64] <= ddr3_rd_data[79:64];
                    compressed_data[95:80] <= ddr3_rd_data[127:112];
                end
                8'd2: begin
                    compressed_data[111:96] <= ddr3_rd_data[47:32];
                    compressed_data[127:112] <= ddr3_rd_data[95:80];
                end
            endcase
            rd_data_valid_cnt <= rd_data_valid_cnt + 8'd1;
        end else if (state != S_READ) begin
            rd_data_valid_cnt <= 8'd0;
        end
    end
end

// =========================================================
// 状态机控制
// =========================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_IDLE;
        ddr3_cmd_en <= 1'b0;
        ddr3_cmd <= 3'd0;
        ddr3_addr <= 28'd0;
        ddr3_wr_en <= 1'b0;
        ddr3_wr_end <= 1'b0;
        ddr3_wr_data <= 128'd0;
        ddr3_wr_mask <= 16'd0;
        line_complete <= 1'b0;
        delay_count <= 16'd0;
        current_src_addr <= FRAME0_BASE_ADDR;
        current_dst_addr <= DST_BASE_ADDR;
        src_line_base <= FRAME0_BASE_ADDR;
        dst_line_base <= DST_BASE_ADDR;
        line_count <= 16'd0;
        read_cmd_cnt <= 8'd0;
        frame_done <= 1'b0;
        module_idle <= 1'b1;
    end else begin
        // 默认值
        ddr3_cmd_en <= 1'b0;
        ddr3_wr_en <= 1'b0;
        ddr3_wr_end <= 1'b0;
        
        case (state)
            S_IDLE: begin
                module_idle <= 1'b1;
                if (start && init_calib_complete && !frame_done) begin
                    current_src_addr <= src_line_base;
                    current_dst_addr <= dst_line_base;
                    read_cmd_cnt <= 8'd0;
                    state <= S_READ; // 启动读取状态
                end else if (frame_done) begin
                    frame_done <= 1'b0; // 清除帧完成标志
                end
            end
            
            S_READ: begin
                if (ddr3_cmd_ready && fifo_wr_num < WR_PER_LINE) begin
                    ddr3_cmd_en <= 1'b1;  // 启动DDR3命令
                    ddr3_cmd <= 3'b001;   // 读命令
                    ddr3_addr <= current_src_addr; // 设置读地址
                    read_cmd_cnt <= read_cmd_cnt + 8'd1; // 命令计数
                    if (line_jump) begin
                        // 行跳转，更新源地址
                        current_src_addr <= current_src_addr + ADDR_INCREMENT + 28'd921600 - 28'd1280; // 跳转到下一组行的起始地址
                    end else begin
                        // 行内，更新源地址
                        current_src_addr <= current_src_addr + ADDR_INCREMENT;
                    end
                end else if (read_cmd_cnt >= BURSTS_PER_LINE) begin
                    ddr3_cmd_en <= 1'b0;
                end
                if (fifo_wr_num == WR_PER_LINE) begin
                    // 行完成，等待状态
                    state <= S_WAIT;
                end
                
            end
            
            
            S_WAIT: begin
                // 如果cmd_ready为0，保持等待状态
                if (ddr3_cmd_ready) begin
                    state <= S_WRITE;
                end   
            end
            
            S_WRITE: begin
                if (ddr3_cmd_ready && ddr3_wr_rdy && fifo_rd_en) begin
                    // 发送写命令
                    ddr3_cmd_en <= 1'b1;
                    ddr3_cmd <= 3'b000;  // 写命令
                    ddr3_addr <= current_dst_addr;
                    ddr3_wr_en <= 1'b1;
                    ddr3_wr_end <= 1'b1;
                    ddr3_wr_data <= fifo_data_out;
                    ddr3_wr_mask <= 16'd0;  // 不屏蔽任何数据
                    current_dst_addr <= current_dst_addr + ADDR_INCREMENT;// 地址递增
                end
                // 如果cmd_ready为0，保持当前状态
                // 检查是否写完一行
                if (fifo_wr_num == 9'd1) begin // 最后一个数据
                    state <= S_WAIT_WRITE;
                    delay_count <= 16'd0;
                end
            end
            
            S_WAIT_WRITE: begin
                // 短暂等待确保写入完成
                if (delay_count < 16'd10) begin
                    delay_count <= delay_count + 16'd1;
                end else if (delay_count < 16'd12) begin
                    delay_count <= delay_count + 16'd1;
                    line_complete <= 1'b1;
                end else begin
                    delay_count <= 16'd0;
                    line_complete <= 1'b1;
                    state <= S_DONE;
                end                   
            end

            S_DONE: begin
                // 行完成时更新行基地址
                if (line_complete) begin
                    // 换帧源地址校准
                    if (line_cnt_base == 8'd2) begin
                        src_line_base <= src_line_base + PIXELS_PER_LINE + 28'd921600 - 28'd1280; // 每8行字节数
                    end else if (line_cnt_base == 8'd5) begin
                        src_line_base <= src_line_base + PIXELS_PER_LINE + 28'd921600 - 28'd1280; // 每8行字节数
                    end else if (line_cnt_base == 8'd7) begin
                        src_line_base <= src_line_base + PIXELS_PER_LINE - 28'd1843200 + 28'd2560; // 每8行字节数
                    end else begin
                        src_line_base <= src_line_base + PIXELS_PER_LINE; // 每行字节数
                    end
                    dst_line_base <= dst_line_base + (PIXELS_PER_LINE / 3);
                    line_count <= line_count + 16'd1;
                    line_complete <= 1'b0;
                    state <= S_IDLE;
                    // 帧完成时重置行基地址（可选）
                    if (line_count >= TOTAL_LINES - 1) begin
                        src_line_base <= FRAME0_BASE_ADDR;
                        dst_line_base <= DST_BASE_ADDR;
                        line_count <= 16'd0;
                        frame_done <= 1'b1;
                    end
                end
            end
            
            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

// =========================================================
// 输出信号
// =========================================================
assign idle = module_idle;
assign line_done = line_complete;

endmodule