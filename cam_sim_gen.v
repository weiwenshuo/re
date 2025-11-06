module camera_timing_sim #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter H_BLANK    = 370,      // 行消隐
    parameter V_BLANK    = 30        // 场消隐
)(
    input           clk,              // 外部像素时钟输入
    input           rst_n,
    
    // FIFO接口
    input [15:0]    fifo_data,
    input           fifo_empty,
    input           fifo_almost_empty,
    output reg      fifo_rd_en,
    
    // 摄像头时序输出
    output          pixel_clk,
    output reg      vsync,
    output reg      href,
    output reg [15:0] data_out,
    output          data_valid
);

// =========================================================
// 时序参数计算
// =========================================================
localparam H_TOTAL = IMG_WIDTH + H_BLANK;
localparam V_TOTAL = IMG_HEIGHT + V_BLANK;

// =========================================================
// 计数器定义
// =========================================================
reg [15:0] h_count;      // 水平计数器
reg [15:0] v_count;      // 垂直计数器
reg        frame_active; // 帧有效标志

// =========================================================
// 像素时钟生成 (可选分频)
// =========================================================
reg pixel_clk_reg;
reg [1:0] clk_div;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        clk_div <= 2'd0;
        pixel_clk_reg <= 1'b0;
    end else begin
        clk_div <= clk_div + 2'd1;
        if (clk_div == 2'd1) begin
            pixel_clk_reg <= ~pixel_clk_reg;
        end
    end
end

assign pixel_clk = pixel_clk_reg;

// =========================================================
// 时序生成逻辑
// =========================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        h_count <= 16'd0;
        v_count <= 16'd0;
        vsync <= 1'b0;
        href <= 1'b0;
        data_out <= 16'd0;
        fifo_rd_en <= 1'b0;
        frame_active <= 1'b0;
    end else if (pixel_clk_reg) begin
        // 水平计数器
        if (h_count < H_TOTAL - 1) begin
            h_count <= h_count + 16'd1;
        end else begin
            h_count <= 16'd0;
            // 垂直计数器
            if (v_count < V_TOTAL - 1) begin
                v_count <= v_count + 16'd1;
            end else begin
                v_count <= 16'd0;
            end
        end
        
        // VSYNC生成 (低电平有效)
        vsync <= (v_count >= V_BLANK) ? 1'b1 : 1'b0;
        
        // HREF生成 (行有效)
        href <= (v_count >= V_BLANK) && (v_count < V_BLANK + IMG_HEIGHT) && 
                (h_count >= H_BLANK) && (h_count < H_BLANK + IMG_WIDTH);
        
        // 数据输出和FIFO读取
        if (href && !fifo_empty) begin
            data_out <= fifo_data;
            fifo_rd_en <= 1'b1;
        end else begin
            data_out <= 16'd0;
            fifo_rd_en <= 1'b0;
        end
        
        // 帧有效标志
        frame_active <= (v_count >= V_BLANK) && (v_count < V_BLANK + IMG_HEIGHT);
    end
end

assign data_valid = href && !fifo_empty;

endmodule