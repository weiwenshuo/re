module ddr3_arbit(
input dma_clk,
input rst_n,

input cmd_ready,
output [2:0] cmd,
output cmd_en,
output [28:0] addr,
input wr_data_rdy,
output wr_data_en,
output wr_data_end,
output [127:0] wr_data,
input rd_data_valid,
input rd_data_end,
input [127:0] rd_data,

output cmd_ready_0,
input [2:0] cmd_0,
input cmd_en_0,
input [28:0] addr_0,
output wr_data_rdy_0,
input wr_data_en_0,
input wr_data_end_0,
input [127:0] wr_data_0,
output rd_data_valid_0,
output rd_data_end_0,
output [127:0] rd_data_0,

output cmd_ready_1,
input [2:0] cmd_1,
input cmd_en_1,
input [28:0] addr_1,
output wr_data_rdy_1,
input wr_data_en_1,
input wr_data_end_1,
input [127:0] wr_data_1,
output rd_data_valid_1,
output rd_data_end_1,
output [127:0] rd_data_1,

output cmd_ready_2,
input [2:0] cmd_2,
input cmd_en_2,
input [28:0] addr_2,
output wr_data_rdy_2,
input wr_data_en_2,
input wr_data_end_2,
input [127:0] wr_data_2,
output rd_data_valid_2,
output rd_data_end_2,
output [127:0] rd_data_2,

// 新增第三个buffer接口
output cmd_ready_3,
input [2:0] cmd_3,
input cmd_en_3,
input [27:0] addr_3,  // 注意：这里是27:0，与其他通道的28:0不同
output wr_data_rdy_3,
input wr_data_en_3,
input wr_data_end_3,
input [127:0] wr_data_3,
output rd_data_valid_3,
output rd_data_end_3,
output [127:0] rd_data_3,

input idle_4,
output cmd_ready_4,
output start_4,
input [2:0] cmd_4,
input cmd_en_4,
input [27:0] addr_4,  // 注意：这里是27:0，与其他通道的28:0不同
output wr_data_rdy_4,
input wr_data_en_4,
input wr_data_end_4,
input [127:0] wr_data_4,
output rd_data_valid_4,
output rd_data_end_4,
output [127:0] rd_data_4

);

// 将2位扩展为3位，支持5个通道(0-4)
reg [2:0] wr_channel,rd_channel;
reg [9:0] rd_credit_cnt;
reg [10:0] wr_cnt;
reg clear;
reg switched;
reg rd_data_valid_dl,wr_data_end_dl_0,wr_data_end_dl_1,wr_data_end_dl_2;
wire idle_state_cb;

// 声明idle_4延迟寄存器（添加到现有寄存器声明区域）
reg idle_4_dly;
wire idle_4_rising_edge;

// 在现有延迟寄存器always块中添加（或新建always块）
always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)begin
        idle_4_dly <= 1'b0;
    end
    else begin
        idle_4_dly <= idle_4;  // 采样前一周期的idle_4状态
    end
end

// 移除按键相关寄存器

always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)begin
        rd_data_valid_dl <= 1'b0;
        wr_data_end_dl_0 <= 1'b0;
        wr_data_end_dl_1 <= 1'b0;
        wr_data_end_dl_2 <= 1'b0;
    end
    else begin
        rd_data_valid_dl <= rd_data_valid;
        wr_data_end_dl_0 <= wr_data_end_0;
        wr_data_end_dl_1 <= wr_data_end_1;
        wr_data_end_dl_2 <= wr_data_end_2;
    end
end

// 修改通道切换逻辑，将buffer4纳入正常轮转仲裁
always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)begin
        wr_channel <= 3'b000;
        rd_channel <= 3'b000;
        clear <= 1'b0;
        switched <= 1'b0;
    end
    else begin
        // 正常仲裁模式，五个buffer轮转
        if((!rd_data_valid & rd_data_valid_dl & rd_credit_cnt == 10'b0 ) | (idle_state_cb) |(idle_4_rising_edge))begin
            if(rd_channel == 3'b100) begin  // 如果是buffer4，回到buffer0
                rd_channel <= 3'b000;
                wr_channel <= 3'b000;
            end
            else begin
                rd_channel <= rd_channel + 1;
                wr_channel <= wr_channel + 1;
            end
            clear <= 1'b1;
            switched <= 1'b1;
        end
        else begin 
            clear <= 1'b0;
        end
    end
end



// 上升沿检测逻辑
assign idle_4_rising_edge = (idle_4 == 1'b1) && (idle_4_dly == 1'b0);
reg start_4_reg;
always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)begin
        start_4_reg <= 1'b0;
    end else begin
        if(rd_channel == 3'd4 && !idle_4) begin
            start_4_reg <= 1'b1;
        end else if (idle_4) begin
            start_4_reg <= 1'b0;
        end
    end
end
assign start_4 = start_4_reg;


//add wr_cnt++ logic
reg wr_64ok;
always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)begin
        wr_cnt <= 11'b0;
    end
    else begin
        if(clear)
            wr_cnt <= 0;
        else if(wr_data_end & wr_cnt < 11'd63)
            wr_cnt <= wr_cnt + 1;
    end
end

always@(posedge dma_clk or negedge rst_n)begin
    if(~rst_n)begin
        wr_64ok <= 1'b0;
    end
    else begin
        if(clear)
            wr_64ok <= 1'b0;
        else if(wr_cnt == 11'd63)
            wr_64ok <= 1'b1;
    end
end
//

always@(posedge dma_clk or negedge rst_n)begin
    if(!rst_n)
        rd_credit_cnt <= 10'b0;
    else if(idle_4_rising_edge == 1'd1) begin
        rd_credit_cnt <= 10'b0;
    end
    else if(rd_channel < 3'd4) begin
        if(cmd == 3'b001 & cmd_en == 1'b1 & rd_data_valid == 1'b1)
            rd_credit_cnt <= rd_credit_cnt;
        else if(cmd == 3'b001 & cmd_en == 1'b1)
            rd_credit_cnt <= rd_credit_cnt + 1;
        else if(rd_data_valid == 1'b1)
            rd_credit_cnt <= rd_credit_cnt - 1;
    end else if (rd_channel == 3'd4) begin
        rd_credit_cnt <= 10'b1; // buffer4不参与读信用计数
    end
end

//某一通道长时间不发命令，代表空闲，空闲指定个数的时钟后自动跳转
reg start;
always@(posedge dma_clk or negedge rst_n)begin
    if(~rst_n)
        start <= 1'b0;
    else 
        if(cmd_en)
            start <= 1'b1;
end

reg idle_state;
reg [10:0] idle_cnt;
always@(posedge dma_clk or negedge rst_n)begin
    if(~rst_n)
        idle_cnt <= 11'b0;
    else begin
        if(clear || cmd_en)begin
            idle_cnt <= 11'b0;
        end
        else if(cmd_ready & !cmd_en & idle_cnt <11'd1024)begin
            idle_cnt <= idle_cnt + 1;
        end
    end
end

always@(posedge dma_clk or negedge rst_n)begin
    if(~rst_n)
        idle_state <= 1'b0;
    else begin
        if(clear || cmd_en)
            idle_state <= 1'b0;
        else if(idle_cnt == 11'd1024)
            idle_state <= 1'b1;
    end
end


assign idle_state_cb = (clear) ? 1'b0 : idle_state;

// 修改数据分配逻辑，加入buffer4
assign rd_data_0 = (rd_channel == 3'b000) ? rd_data : 128'b0;
assign rd_data_1 = (rd_channel == 3'b001) ? rd_data : 128'b0;
assign rd_data_2 = (rd_channel == 3'b010) ? rd_data : 128'b0;
assign rd_data_3 = (rd_channel == 3'b011) ? rd_data : 128'b0;  // 新增buffer3
assign rd_data_4 = (rd_channel == 3'b100) ? rd_data : 128'b0;  // 新增buffer4

assign rd_data_valid_0 = (rd_channel == 3'b000) ? rd_data_valid : 1'b0;
assign rd_data_valid_1 = (rd_channel == 3'b001) ? rd_data_valid : 1'b0;
assign rd_data_valid_2 = (rd_channel == 3'b010) ? rd_data_valid : 1'b0;  
assign rd_data_valid_3 = (rd_channel == 3'b011) ? rd_data_valid : 1'b0;  // 新增buffer3
assign rd_data_valid_4 = (rd_channel == 3'b100) ? rd_data_valid : 1'b0;  // 新增buffer4

assign rd_data_end_0 = (rd_channel == 3'b000) ? rd_data_end : 1'b0;
assign rd_data_end_1 = (rd_channel == 3'b001) ? rd_data_end : 1'b0;
assign rd_data_end_2 = (rd_channel == 3'b010) ? rd_data_end : 1'b0; 
assign rd_data_end_3 = (rd_channel == 3'b011) ? rd_data_end : 1'b0;  // 新增buffer3
assign rd_data_end_4 = (rd_channel == 3'b100) ? rd_data_end : 1'b0;  // 新增buffer4

// 修改写数据准备好信号分配
assign wr_data_rdy_0 = (rd_channel == 3'b000) ? wr_data_rdy : 1'b0;
assign wr_data_rdy_1 = (rd_channel == 3'b001) ? wr_data_rdy : 1'b0;
assign wr_data_rdy_2 = (rd_channel == 3'b010) ? wr_data_rdy : 1'b0; 
assign wr_data_rdy_3 = (rd_channel == 3'b011) ? wr_data_rdy : 1'b0;  // 新增buffer3
assign wr_data_rdy_4 = (rd_channel == 3'b100) ? wr_data_rdy : 1'b0;  // 新增buffer4

assign cmd_ready_0 = (rd_channel == 3'b000) ? cmd_ready : 1'b0;
assign cmd_ready_1 = (rd_channel == 3'b001) ? cmd_ready : 1'b0;
assign cmd_ready_2 = (rd_channel == 3'b010) ? cmd_ready : 1'b0; 
assign cmd_ready_3 = (rd_channel == 3'b011) ? cmd_ready : 1'b0;  // 新增buffer3
assign cmd_ready_4 = (rd_channel == 3'b100) ? cmd_ready : 1'b0;  // 新增buffer4

// 修改DDR3输出信号选择，加入buffer4
assign wr_data = (rd_channel == 3'b000) ? wr_data_0 :
                 (rd_channel == 3'b001) ? wr_data_1 : 
                 (rd_channel == 3'b010) ? wr_data_2 :
                 (rd_channel == 3'b011) ? wr_data_3 :
                 wr_data_4;  // 新增buffer4

assign wr_data_en = (rd_channel == 3'b000) ? wr_data_en_0 :
                    (rd_channel == 3'b001) ? wr_data_en_1 : 
                    (rd_channel == 3'b010) ? wr_data_en_2 :
                    (rd_channel == 3'b011) ? wr_data_en_3 :
                    wr_data_en_4;  // 新增buffer4

assign wr_data_end = (rd_channel == 3'b000) ? wr_data_end_0 :
                     (rd_channel == 3'b001) ? wr_data_end_1 : 
                     (rd_channel == 3'b010) ? wr_data_end_2 :
                     (rd_channel == 3'b011) ? wr_data_end_3 :
                     wr_data_end_4;  // 新增buffer4

// 注意：addr_3和addr_4都是27位，需要扩展为28位
assign addr = (rd_channel == 3'b000) ? addr_0 :
              (rd_channel == 3'b001) ? addr_1 : 
              (rd_channel == 3'b010) ? addr_2 :
              (rd_channel == 3'b011) ? {1'b0, addr_3} :  // 将27位addr_3扩展为28位
              {1'b0, addr_4};  // 将27位addr_4扩展为28位

assign cmd = (rd_channel == 3'b000) ? cmd_0 :
             (rd_channel == 3'b001) ? cmd_1 : 
             (rd_channel == 3'b010) ? cmd_2 :
             (rd_channel == 3'b011) ? cmd_3 :
             cmd_4;  // 新增buffer4

assign cmd_en = (rd_channel == 3'b000) ? cmd_en_0 :
                (rd_channel == 3'b001) ? cmd_en_1 : 
                (rd_channel == 3'b010) ? cmd_en_2 :
                (rd_channel == 3'b011) ? cmd_en_3 :
                cmd_en_4;  // 新增buffer4

endmodule