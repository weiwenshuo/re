module UDP#(
	parameter BOARD_MAC 	= 48'h00_0a_35_01_fe_c0 		,//开发板MAC地址
	parameter BOARD_IP 		= {8'd192,8'd168,8'd0,8'd2}	, 	//开发板IP地址
	parameter BOARD_PORT	= 16'd8080, 					 //开发板IP地址-端口 
	parameter DES_MAC 		= 48'hff_ff_ff_ff_ff_ff 		,//目的MAC地址
	parameter DES_IP 		= {8'd192,8'd168,8'd0,8'd3} 	,//目的IP地址
	parameter DES_PORT		= 16'd8080, 					 //目的IP地址-端口 
	parameter DATA_SIZE		= 16'd2562			 			 //数据包长度 46~1500 B
	)(
	input 			clk,
	input 			rst_n, 			
    //数据输入接口
    input [15:0] 	cmos_db,
    input 			cmos_pclk,
    input 			cmos_vsync,
    input 			cmos_href,
    //GMII/RGMII接口
	output 			PHY_CLK,
	output 			RGMII_GTXCLK,
	output 			RGMII_RST_N, 	
	output [3:0] 	RGMII_TXD, 		
	output reg		RGMII_TXEN
	);
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////// 			 Camera ETH Formator 模块	        /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
wire [15:0]  camera_wrdata;
wire        camera_pixel_clk;
wire        camera_fifo_aclr;
wire        camera_data_valid;

// 行计数器相关信号
reg [15:0] current_line_num;     // 当前行号计数器
reg [15:0] fifo_line_num;        // FIFO中存储的行号
reg href_delay;                  // HREF延迟寄存器，用于边沿检测

Camera_ETH_Formator Camera_ETH_Formator_m0(
    .PCLK(cmos_pclk),           // 像素时钟
    .Rst_n(rst_n),              // 模块复位
    .Init_Done(1'b1),           // 摄像头初始化完成信号
    .HREF(cmos_href),           // 行同步信号
    .VSYNC(cmos_vsync),         // 场同步信号
    .DATA(cmos_db),             // 输入数据
    .wrdata(camera_wrdata),     // 输出数据
    .pixel_clk(camera_pixel_clk), // 像素时钟输出
    .fifo_aclr(camera_fifo_aclr), // FIFO清零信号
    .data_valid(camera_data_valid) // 数据有效信号
);

// 行计数器逻辑
always @(posedge cmos_pclk or negedge rst_n) begin
    if (!rst_n) begin
        current_line_num <= 16'd0;
        href_delay <= 1'b0;
    end else begin
        href_delay <= cmos_href;
        
        // 场同步信号到来时重置行计数器
        if (cmos_vsync) begin
            current_line_num <= 16'd0;
        end
        // HREF上升沿检测，新行开始
        else if ({href_delay, cmos_href} == 2'b01) begin
            current_line_num <= current_line_num + 16'd1;
        end
    end
end

// FIFO写入时更新行号
always @(posedge camera_pixel_clk or negedge rst_n) begin
    if (!rst_n) begin
        fifo_line_num <= 16'd0;
    end else if (control_state == 2'b00) begin
        fifo_line_num <= current_line_num;
    end
end
wire [15:0] fifo_data;
wire hblank;
wire camera_data_clk;
assign fifo_data = camera_wrdata;
assign camera_data_clk = camera_pixel_clk;
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////// 			    数据控制模块 	        /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
reg control_flag;//数据控制标志位 1是读数据 0是写数据
reg mid_flag;
reg [1:0] control_state;//状态机状态定义
reg camera_data_valid_r; // camera_data_valid延迟寄存器
reg send_enable; // 发送使能信号
// camera_data_valid延迟寄存器，用于检测上升沿
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
        camera_data_valid_r <= 1'b0;
    else
        camera_data_valid_r <= camera_data_valid;
end
//状态机设计
always@(posedge clk or negedge rst_n) begin
    if(!rst_n)
    begin
        control_flag <= 1'b0;
		control_state <= 2'b10;
		mid_flag <= 1'b1;
	end
    else begin
        case (control_state)
            2'b00: begin//写状态;
			    mid_flag <= 1'b1;
				control_flag <= 1'b0;
				send_enable <= 1'b0;
                if(Almost_Full==1'b1)
                begin
                    // FIFO快满时切换到读状态
                    control_flag <= 1'b1;
                    control_state <= 2'b01;
                end
            end
			2'b01: begin//读状态
			    mid_flag <= 1'b1;
				control_flag <= 1'b1;
				send_enable <= 1'b1;
			    if(Empty==1'b1)
				begin
					// FIFO空时切换到写过度状态
					control_flag <= 1'b0;
					control_state <= 2'b10;
				end
			end
            2'b10: begin//中间状态
                mid_flag <= 1'b0;
				send_enable <= 1'b0;
                if({camera_data_valid_r, camera_data_valid} == 2'b01)//切换回写状态
                begin
                    control_state <= 2'b00;
					mid_flag <= 1'b1;
					control_flag <= 1'b0;
                end
            end
        endcase
    end
end
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////// 			 UDP的FIFO储存模块	        /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
	wire [7:0] Data;
	wire WrClk;
	wire RdClk;
	wire WrEn;
	wire RdEn;
	wire Almost_Empty;
	wire Almost_Full;
	wire [15:0] Q;
	wire Empty;
	wire Full;
	wire FIFO_enable;
	wire FIFO_CLK;
	assign Data = camera_wrdata;
	assign WrClk = camera_data_clk;
	assign WrEn = camera_data_valid&&!control_flag&&mid_flag;
    assign RdEn = control_flag&&mid_flag&&FIFO_enable;
	assign RdClk = FIFO_CLK;
	UDP_FIFO your_instance_name(
		.Data(fifo_data), //input [15:0] Data
		.WrClk(WrClk), //input WrClk
		.RdClk(RdClk), //input RdClk
		.WrEn(WrEn), //input WrEn
		.RdEn(RdEn), //input RdEn
		.Almost_Empty(Almost_Empty), //output Almost_Empty
		.Almost_Full(Almost_Full), //output Almost_Full
		.Q(Q), //output [15:0] Q
		.Empty(Empty), //output Empty
		.Full(Full) //output Full
	);

/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////// 			    GMII发送子模块 	        /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
GMII_pll GMII_pll_m0(
	.clkin 		(clk 	 	),
    .mdclk   (clk),
	.clkout0 	(RGMII_GTXCLK 	),
	.clkout1 	(PHY_CLK 		)
	);	

wire GMII_TXEN;
wire [7:0] GMII_TXD;

GMII_send #(
	.BOARD_MAC 	(BOARD_MAC  ),//开发板MAC地址
	.BOARD_IP 	(BOARD_IP 	),//开发板IP地址
	.BOARD_PORT (BOARD_PORT ),
	.DES_MAC 	(DES_MAC 	),//目的MAC地址
	.DES_IP 	(DES_IP 	),//目的IP地址
	.DES_PORT 	(DES_PORT 	),
	.DATA_SIZE	(DATA_SIZE	)
	)GMII_send_m0(
	.rst_n 			(rst_n 				),
	.enable 		(send_enable 		), // 发送使能信号

	.GMII_GTXCLK 	(RGMII_GTXCLK 		),
	.GMII_TXD 		(GMII_TXD 			),
	.GMII_TXEN 		(GMII_TXEN 			),
	.GMII_TXER 		(	 				),
		//定义读写时钟
	.FIFO_CLK		(FIFO_CLK),
	.fifo_data		(Q),
	.FIFO_enable	(FIFO_enable),
	
	// 行号输入端口
	.fifo_line_num	(fifo_line_num)		// FIFO中存储的行号
	);
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
//////////////////// 			    GMII 2 RGMII	        /////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
reg [7:0] GMII_TXD_R;
always@(posedge RGMII_GTXCLK) GMII_TXD_R <= GMII_TXD;

GMII2RGMII GMII2RGMII_m0(
	.clk 		(RGMII_GTXCLK 		),
	.din 		(GMII_TXD_R 		),
	.q 			(RGMII_TXD 			)
	);

reg [2:0] GMII_TXEN_R;
always@(posedge RGMII_GTXCLK) GMII_TXEN_R <= {GMII_TXEN_R[1:0],GMII_TXEN};
always@(posedge RGMII_GTXCLK) RGMII_TXEN <= GMII_TXEN_R[2];
assign RGMII_RST_N = rst_n;


endmodule