// =========================================================
// 三帧图像压缩模块
// 功能：将三帧1280x720图像横向压缩为一帧，每隔3个像素取一个像素
// 压缩后图像存储在第四个地址空间
// =========================================================
module frame_compression #(
    parameter FRAME0_BASE_ADDR = 28'd0,           // 第一帧基地址
    parameter FRAME1_BASE_ADDR = 28'd921600,      // 第二帧基地址
    parameter FRAME2_BASE_ADDR = 28'd1843200,     // 第三帧基地址
    parameter DST_BASE_ADDR    = 28'd2764800,     // 压缩后目标基地址
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
localparam PIXELS_PER_BURST   = 8;              // 每次突发传输像素数（128位=8像素）
localparam BURSTS_PER_LINE    = IMG_WIDTH / PIXELS_PER_BURST;  // 每行突发次数
localparam COMPRESSED_WIDTH   = IMG_WIDTH / 3;  // 压缩后图像宽度（426像素）
localparam BURSTS_PER_COMPRESSED_LINE = (COMPRESSED_WIDTH + PIXELS_PER_BURST - 1) / PIXELS_PER_BURST; // 压缩后每行突发次数
localparam TOTAL_LINES        = IMG_HEIGHT;     // 总行数
localparam ADDR_INCREMENT     = 8;              // 每次地址增量（字节）

// 三帧地址偏移
localparam FRAME_SIZE         = IMG_WIDTH * IMG_HEIGHT;  // 每帧像素数 = 921600

// =========================================================
// 内部信号定义
// =========================================================
// 状态机
localparam S_IDLE           = 4'd0;
localparam S_READ_FRAME0    = 4'd1;
localparam S_READ_FRAME1    = 4'd2;
localparam S_READ_FRAME2    = 4'd3;
localparam S_PROCESS_DATA   = 4'd4;
localparam S_WRITE          = 4'd5;
localparam S_WAIT_WRITE     = 4'd6;
localparam S_DONE           = 4'd7;
reg [3:0] state, next_state;

// 地址和计数器
reg [27:0] current_frame0_addr;   // 当前帧0地址
reg [27:0] current_frame1_addr;   // 当前帧1地址
reg [27:0] current_frame2_addr;   // 当前帧2地址
reg [27:0] current_dst_addr;      // 当前目标地址
reg [15:0] line_count;            // 行计数
reg [10:0] pixel_count;           // 像素计数（当前行内）
reg [8:0]  burst_count;           // 突发计数
reg [15:0] delay_count;           // 延时计数器

// 数据缓存
reg [127:0] frame0_data;          // 帧0数据缓存
reg [127:0] frame1_data;          // 帧1数据缓存
reg [127:0] frame2_data;          // 帧2数据缓存
reg [127:0] compressed_data;      // 压缩后数据
reg [3:0]   data_valid;           // 数据有效标志

// FIFO接口信号
wire [127:0] fifo_data_in;        // FIFO输入数据
wire         fifo_wr_en;          // FIFO写使能
wire [127:0] fifo_data_out;       // FIFO输出数据
wire         fifo_rd_en;          // FIFO读使能
wire         fifo_empty;          // FIFO空标志
wire         fifo_full;           // FIFO满标志
wire         fifo_almost_empty;   // FIFO半空标志
wire         fifo_almost_full;    // FIFO半满标志
wire [8:0]   fifo_wr_num;         // FIFO写入数据计数

// 控制信号
reg processing_line;              // 行处理中标志
reg line_complete;                // 行完成标志
reg module_idle;                  // 模块空闲标志
reg frame_done;                   // 帧完成标志
assign frame_done_out = frame_done;

// =========================================================
// 同步FIFO实例化
// =========================================================
fifo_sc_top fifo_inst(
    .Data(fifo_data_in),          // input [127:0] Data
    .Clk(clk),                    // input Clk
    .WrEn(fifo_wr_en),            // input WrEn
    .RdEn(fifo_rd_en),            // input RdEn
    .Reset(~rst_n),               // input Reset
    .Wnum(fifo_wr_num),           // output [8:0] Wnum
    .Almost_Empty(fifo_almost_empty), // output Almost_Empty
    .Almost_Full(fifo_almost_full),   // output Almost_Full
    .Q(fifo_data_out),            // output [127:0] Q
    .Empty(fifo_empty),           // output Empty
    .Full(fifo_full)              // output Full
);

// FIFO控制信号连接
assign fifo_data_in = compressed_data;     // 压缩后数据写入FIFO
assign fifo_wr_en = (state == S_PROCESS_DATA) && (data_valid == 4'b1111); // 所有数据有效时写入FIFO
assign fifo_rd_en = (state == S_WRITE) && ddr3_wr_rdy && !fifo_empty;
assign fifo_wr_count = fifo_wr_num;

// =========================================================
// 像素压缩逻辑
// =========================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        frame0_data <= 128'd0;
        frame1_data <= 128'd0;
        frame2_data <= 128'd0;
        compressed_data <= 128'd0;
        data_valid <= 4'b0000;
    end else begin
        case (state)
            S_READ_FRAME0: begin
                if (ddr3_rd_data_valid) begin
                    frame0_data <= ddr3_rd_data;
                    data_valid[0] <= 1'b1;
                end
            end
            
            S_READ_FRAME1: begin
                if (ddr3_rd_data_valid) begin
                    frame1_data <= ddr3_rd_data;
                    data_valid[1] <= 1'b1;
                end
            end
            
            S_READ_FRAME2: begin
                if (ddr3_rd_data_valid) begin
                    frame2_data <= ddr3_rd_data;
                    data_valid[2] <= 1'b1;
                end
            end
            
            S_PROCESS_DATA: begin
                if (data_valid == 4'b1111) begin
                    // 每隔3个像素取一个像素进行压缩
                    // 从三帧数据中分别取像素0,3,6...进行组合
                    compressed_data[15:0]   <= frame0_data[15:0];    // 帧0像素0
                    compressed_data[31:16]  <= frame1_data[15:0];    // 帧1像素0
                    compressed_data[47:32]  <= frame2_data[15:0];    // 帧2像素0
                    compressed_data[63:48]  <= frame0_data[47:32];   // 帧0像素3
                    compressed_data[79:64]  <= frame1_data[47:32];   // 帧1像素3
                    compressed_data[95:80]  <= frame2_data[47:32];   // 帧2像素3
                    compressed_data[111:96] <= frame0_data[79:64];   // 帧0像素6
                    compressed_data[127:112]<= frame1_data[79:64];   // 帧1像素6
                    data_valid[3] <= 1'b1;
                end else begin
                    data_valid <= 4'b0000;
                end
            end
            
            default: begin
                data_valid <= 4'b0000;
            end
        endcase
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
        current_frame0_addr <= FRAME0_BASE_ADDR;
        current_frame1_addr <= FRAME1_BASE_ADDR;
        current_frame2_addr <= FRAME2_BASE_ADDR;
        current_dst_addr <= DST_BASE_ADDR;
        line_count <= 16'd0;
        burst_count <= 9'd0;
        pixel_count <= 11'd0;
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
                    current_frame0_addr <= FRAME0_BASE_ADDR + (line_count * IMG_WIDTH);
                    current_frame1_addr <= FRAME1_BASE_ADDR + (line_count * IMG_WIDTH);
                    current_frame2_addr <= FRAME2_BASE_ADDR + (line_count * IMG_WIDTH);
                    current_dst_addr <= DST_BASE_ADDR + (line_count * COMPRESSED_WIDTH);
                    burst_count <= 9'd0;
                    pixel_count <= 11'd0;
                    state <= S_READ_FRAME0;
                    module_idle <= 1'b0;
                end else if (frame_done) begin
                    frame_done <= 1'b0; // 清除帧完成标志
                end
            end
            
            S_READ_FRAME0: begin
                if (ddr3_cmd_ready) begin
                    ddr3_cmd_en <= 1'b1;
                    ddr3_cmd <= 3'b001;   // 读命令
                    ddr3_addr <= current_frame0_addr + (burst_count * ADDR_INCREMENT);
                    state <= S_READ_FRAME1;
                end
            end
            
            S_READ_FRAME1: begin
                if (ddr3_cmd_ready) begin
                    ddr3_cmd_en <= 1'b1;
                    ddr3_cmd <= 3'b001;   // 读命令
                    ddr3_addr <= current_frame1_addr + (burst_count * ADDR_INCREMENT);
                    state <= S_READ_FRAME2;
                end
            end
            
            S_READ_FRAME2: begin
                if (ddr3_cmd_ready) begin
                    ddr3_cmd_en <= 1'b1;
                    ddr3_cmd <= 3'b001;   // 读命令
                    ddr3_addr <= current_frame2_addr + (burst_count * ADDR_INCREMENT);
                    state <= S_PROCESS_DATA;
                end
            end
            
            S_PROCESS_DATA: begin
                // 等待数据压缩完成
                if (data_valid == 4'b1111) begin
                    state <= S_WRITE;
                end
            end
            
            S_WRITE: begin
                if (ddr3_cmd_ready && ddr3_wr_rdy && fifo_rd_en) begin
                    // 发送写命令
                    ddr3_cmd_en <= 1'b1;
                    ddr3_cmd <= 3'b000;  // 写命令
                    ddr3_addr <= current_dst_addr + (burst_count * ADDR_INCREMENT);
                    ddr3_wr_en <= 1'b1;
                    ddr3_wr_end <= 1'b1;
                    ddr3_wr_data <= fifo_data_out;
                    ddr3_wr_mask <= 16'd0;  // 不屏蔽任何数据
                    
                    // 更新计数器和状态
                    burst_count <= burst_count + 9'd1;
                    pixel_count <= pixel_count + PIXELS_PER_BURST;
                    
                    if (burst_count < BURSTS_PER_COMPRESSED_LINE - 1) begin
                        state <= S_READ_FRAME0; // 继续处理下一组数据
                    end else begin
                        state <= S_WAIT_WRITE;
                        delay_count <= 16'd0;
                    end
                end
            end
            
            S_WAIT_WRITE: begin
                // 短暂等待确保写入完成
                if (delay_count < 16'd10) begin
                    delay_count <= delay_count + 16'd1;
                end else begin
                    delay_count <= 16'd0;
                    line_complete <= 1'b1;
                    state <= S_DONE;
                end
            end
            
            S_DONE: begin
                if (line_complete) begin
                    line_count <= line_count + 16'd1;
                    line_complete <= 1'b0;
                    state <= S_IDLE;
                    
                    // 检查是否完成所有行
                    if (line_count >= TOTAL_LINES - 1) begin
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