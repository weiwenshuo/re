module top #(
    parameter USE_TPG = "false",
    parameter DDR_BYPASS = "false"
)(
	input                  clk,
	input                  rst_n,
    input                  key_AB13,
    input                  Key_AA13,
	inout                  cmos_scl,       //cmos i2c clock
	inout                  cmos_sda,       //cmos i2c data
	input                  cmos_vsync,     //cmos vsync
	input                  cmos_href,      //cmos hsync refrence,data valid
	input                  cmos_pclk,      //cmos pxiel clock
    output                 cmos_xclk,      //cmos externl clock 
	input  [7:0]           cmos_db,        //cmos data
	output                 cmos_rst_n,     //cmos reset 
	output                 cmos_pwdn,      //cmos power down

	input                  cmos_vsync_1,     //cmos vsync
	input                  cmos_href_1,      //cmos hsync refrence,data valid
	input                  cmos_pclk_1,      //cmos pxiel clock 
	input  [7:0]           cmos_db_1,        //cmos data
	
	input                  cmos_vsync_2,     //cmos vsync
	input                  cmos_href_2,      //cmos hsync refrence,data valid
	input                  cmos_pclk_2,      //cmos pxiel clock 
	input  [7:0]           cmos_db_2,        //cmos data

	output [6:0]           state_led,

	output [15:0]          ddr_addr,       //ROW_WIDTH=16
	output [2:0]           ddr_bank,       //BANK_WIDTH=3
	output                 ddr_cs,
	output                 ddr_ras,
	output                 ddr_cas,
	output                 ddr_we,
	output                 ddr_ck,
	output                 ddr_ck_n,
	output                 ddr_cke,
	output                 ddr_odt,
	output                 ddr_reset_n,
	output [1:0]           ddr_dm,         //DM_WIDTH=4
	inout  [15:0]          ddr_dq,         //DQ_WIDTH=32
	inout  [1:0]           ddr_dqs,        //DQS_WIDTH=4
	inout  [1:0]           ddr_dqs_n,      //DQS_WIDTH=4
  
    output                 tmds_clk_n_0,
    output                 tmds_clk_p_0,
    output [2:0]           tmds_d_n_0, //{r,g,b}
    output [2:0]           tmds_d_p_0,
    output                 hpd_en,
    //hdmi
    output 			PHY_CLK,
	output 			RGMII_GTXCLK,
	output 			RGMII_RST_N, 	
	output [3:0] 	RGMII_TXD, 		
	output		RGMII_TXEN
);

//memory interface
    wire                   memory_clk         ;
    wire                   dma_clk         	  ;
    wire                   DDR_pll_lock       ;
    wire                   cmd_ready          ;
    wire[2:0]              cmd                ;
    wire                   cmd_en             ;
    //wire[5:0]              app_burst_number   ;
    wire[ADDR_WIDTH-1:0]   addr               ;
    wire                   wr_data_rdy        ;
    wire                   wr_data_en         ;
    wire                   wr_data_end        ;
    wire[DATA_WIDTH-1:0]   wr_data            ;   
    wire[DATA_WIDTH/8-1:0] wr_data_mask       ;   
    wire                   rd_data_valid      ;  
    wire                   rd_data_end        ;//unused 
    wire[DATA_WIDTH-1:0]   rd_data            ;   
    wire                   init_calib_complete;
    wire                   err;
    wire                   TMDS_DDR_pll_lock  ;

    //According to IP parameters to choose
    `define	    WR_VIDEO_WIDTH_32
    `define	DEF_WR_VIDEO_WIDTH 32

    `define	    RD_VIDEO_WIDTH_32
    `define	DEF_RD_VIDEO_WIDTH 32

    `define	USE_THREE_FRAME_BUFFER

    `define	DEF_ADDR_WIDTH 29 
    `define	DEF_SRAM_DATA_WIDTH 128
    
    //=========================================================
    //SRAM parameters
    parameter ADDR_WIDTH          = `DEF_ADDR_WIDTH;        //存储单元是byte，总容量=2^29*16bit = 8Gbit,增加1位rank地址，{rank[0],bank[2:0],row[15:0],cloumn[9:0]}
    parameter DATA_WIDTH          = `DEF_SRAM_DATA_WIDTH;   //与生成DDR3IP有关，此ddr3 4Gbit, x32， 时钟比例1:4 ，则固定256bit
    parameter WR_VIDEO_WIDTH      = `DEF_WR_VIDEO_WIDTH;  
    parameter RD_VIDEO_WIDTH      = `DEF_RD_VIDEO_WIDTH;  

    wire                            video_clk;  //video pixel clock
    //-------------------
    //syn_code
    wire                      syn_off0_re;      // ofifo read enable signal
    wire                      syn_off0_vs;
    wire                      syn_off0_hs;

    wire                      off0_syn_de  ;
    wire [RD_VIDEO_WIDTH-1:0] off0_syn_data;

    wire[15:0]                      cmos_16bit_data,cmos_16bit_data_1,cmos_16bit_data_2;
    wire                            cmos_16bit_clk,cmos_16bit_clk_1,cmos_16bit_clk_2;
    wire[15:0] 						write_data;

    wire[9:0]                       lut_index;
    wire[31:0]                      lut_data;
    wire i2c_done;
    wire i2c_err;

    assign cmos_xclk = cmos_clk;
    assign cmos_pwdn = 1'b0;
//    assign cmos_rst_n = 1'b1;
    assign cmos_rst_n = cmos_reset;
    assign write_data = cmos_16bit_data;
    assign hpd_en = 1;
    //assign write_data = {cmos_16bit_data[4:0],cmos_16bit_data[10:5],cmos_16bit_data[15:11]};

    reg [4:0] cmos_vs_cnt;
    always@(posedge cmos_vsync) 
        cmos_vs_cnt <= cmos_vs_cnt + 1;


    //状态指示灯
    assign state_led[4] = ~i2c_done;
    assign state_led[3] = ~cmos_vs_cnt[4];
    assign state_led[2] = ~TMDS_DDR_pll_lock;
    assign state_led[1] = ~DDR_pll_lock; 
    assign state_led[0] = ~init_calib_complete; //DDR3初始化指示灯
    assign state_led[5] = ~hdmi_channel[0];
    assign state_led[6] = ~hdmi_channel[1];
    //generate the CMOS sensor clock and the SDRAM controller, I2C controller clock
    wire [1:0]  DDRPLL_MDR_OPC;
    wire        DDRPLL_MDR_AINC;
    wire [7:0]  DDRPLL_MDR_DO;
    wire [7:0]  DDRPLL_MDR_DI;
    DDR_PLL Gowin_PLL_m0(
    	.clkin                     (clk                         ),
    	.clkout0                   (cmos_clk 	              	),
        .clkout1                   (aux_clk 	              	),
        .clkout2                   (memory_clk 	              	),
    	.lock 					   (DDR_pll_lock 				),
        .reset                     (~rst_n ),
            //25/60K Extra Control
        .mdclk                     (clk),
        .mdopc                     (DDRPLL_MDR_OPC), //input [1:0] mdopc
        .mdainc                    (DDRPLL_MDR_AINC), //input mdainc
        .mdrdo                     (DDRPLL_MDR_DO), //output [7:0] mdrdo
        .mdwdi                     (DDRPLL_MDR_DI) //input [7:0] mdwdi
	);

    // Reset for camera sensor复位等待和初始化等待
    reg [31:0] cmos_reset_delay_cnt;
    reg cmos_reset;
    reg cmos_start_config;
    always@(posedge clk or negedge rst_n)
    begin
        if(!rst_n)
        begin
            cmos_reset_delay_cnt <= 0;
            cmos_reset <= 0;
            cmos_start_config <= 0;
        end else begin
            if(cmos_reset_delay_cnt == 32'd3_000_000)
            begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt;
                cmos_reset <= 1'b1;
                cmos_start_config <= 1'b1;
            end else if(cmos_reset_delay_cnt == 32'd100_000)
            begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                cmos_reset <= 1'b1;
                cmos_start_config <= 1'b0;
            end else begin
                cmos_reset_delay_cnt <= cmos_reset_delay_cnt + 1;
                cmos_reset <= cmos_reset;
                cmos_start_config <= cmos_start_config;
            end
            
        end
    end

    //configure look-up table
    lut_ov5640_rgb565 #(
    	.HActive(12'd1280),
    	.VActive(12'd720),
    	.HTotal(13'd1892),
    	.VTotal(13'd740),
        .USE_4vs3_frame("false")
    )lut_ov5640_rgb565_m0(
    	.lut_index(lut_index),
    	.lut_data(lut_data)
    );


    //I2C master controller
    i2c_config i2c_config_m0(
    	.rst                        (~cmos_start_config       ),
    	.clk                        (clk                      ),
    	.clk_div_cnt                (16'd1000                  ),
    	.i2c_addr_2byte             (1'b1                     ),
    	.lut_index                  (lut_index                ),
    	.lut_dev_addr               (lut_data[31:24]          ),
    	.lut_reg_addr               (lut_data[23:8]           ),
    	.lut_reg_data               (lut_data[7:0]            ),
    	.error                      (i2c_err                  ),
    	.done                       (i2c_done                 ),
    	.i2c_scl                    (cmos_scl                 ),
    	.i2c_sda                    (cmos_sda                 )
    );
    

    //CMOS sensor 8bit data is converted to 16bit data
    cmos_8_16bit cmos_8_16bit_m0(
    	.rst                        (~rst_n                   ),
    	.pclk                       (cmos_pclk                ),
    	.pdata_i                    (cmos_db                  ),
    	.de_i                       (cmos_href                ),
    	.pdata_o                    (cmos_16bit_data          ),
    	.hblank                     (cmos_16bit_wr            ),
    	.de_o                       (cmos_16bit_clk           )
    );
    cmos_8_16bit cmos_8_16bit_m1(
    	.rst                        (~rst_n                   ),
    	.pclk                       (cmos_pclk_1                ),
    	.pdata_i                    (cmos_db_1                  ),
    	.de_i                       (cmos_href_1                ),
    	.pdata_o                    (cmos_16bit_data_1          ),
    	.hblank                     (cmos_16bit_wr_1            ),
    	.de_o                       (cmos_16bit_clk_1           )
    );
    cmos_8_16bit cmos_8_16bit_m2(
    	.rst                        (~rst_n                   ),
    	.pclk                       (cmos_pclk_2                ),
    	.pdata_i                    (cmos_db_2                  ),
    	.de_i                       (cmos_href_2                ),
    	.pdata_o                    (cmos_16bit_data_2          ),
    	.hblank                     (cmos_16bit_wr_2            ),
    	.de_o                       (cmos_16bit_clk_2           )
    );
    //The video output timing generator and generate a frame read data request
    //输出
    wire out_de;
    wire [9:0] lcd_x,lcd_y;

    vga_timing #(
        .H_ACTIVE(16'd1280), 
        .H_FP(16'd110),
        .H_SYNC(16'd40),
        .H_BP(16'd220),
        .V_ACTIVE(16'd720),
        .V_FP(16'd5),
        .V_SYNC(16'd5),
        .V_BP(16'd20), 	
        .HS_POL(1'b1),   	
        .VS_POL(1'b1)
    ) vga_timing_m0(
        .clk (video_clk),
        .rst (~rst_n),
        .active_x(lcd_x),
        .active_y(lcd_y),
        .hs(syn_off0_hs),
        .vs(syn_off0_vs),
        .de(out_de)
    );
    
//    reg wait_done;
//    reg [25:0]wait_cnt;
//    always@(posedge clk or negedge rst_n)begin
//        if(!rst_n)begin
//            wait_done <= 1'b0;
//            wait_cnt <= 24'b0;
//        end
//        else begin
//            if(wait_cnt <= 26'd50000000 & wait_done == 1'b0)
//                wait_cnt <= wait_cnt + 1;
//            else begin
//                wait_done <= 1'b1;
//            end
//        end
//    end
                

    //Test pattern generate
    ///--------------------------
    wire        tp0_vs_in  ;
    wire        tp0_hs_in  ;
    wire        tp0_de_in ;
    wire [ 7:0] tp0_data_r;
    wire [ 7:0] tp0_data_g;
    wire [ 7:0] tp0_data_b;

    generate if(USE_TPG == "true")         
    begin
        testpattern testpattern_inst_1280
        (
            .I_pxl_clk   (video_clk    ),//pixel clock
            .I_rst_n     (rst_n        ),//low active 
            .I_mode      (3'b000       ),//data select
            .I_single_r  (8'd255       ),
            .I_single_g  (8'd255       ),
            .I_single_b  (8'd255       ),                  //800x600    //1024x768   //1280x720   //1920x1080 
            .I_h_total   (12'd1650     ),//hor total time  // 12'd1056  // 12'd1344  // 12'd1650  // 12'd2200
            .I_h_sync    (12'd40       ),//hor sync time   // 12'd128   // 12'd136   // 12'd40    // 12'd44  
            .I_h_bporch  (12'd220      ),//hor back porch  // 12'd88    // 12'd160   // 12'd220   // 12'd148 
            .I_h_res     (12'd1280     ),//hor resolution  // 12'd800   // 12'd1024  // 12'd1280  // 12'd1920
            .I_v_total   (12'd750      ),//ver total time  // 12'd628   // 12'd806   // 12'd750   // 12'd1125 
            .I_v_sync    (12'd5        ),//ver sync time   // 12'd4     // 12'd6     // 12'd5     // 12'd5   
            .I_v_bporch  (12'd20       ),//ver back porch  // 12'd23    // 12'd29    // 12'd20    // 12'd36  
            .I_v_res     (12'd720      ),//ver resolution  // 12'd600   // 12'd768   // 12'd720   // 12'd1080 
            .I_hs_pol    (1'b1         ),//0,负极性;1,正极性
            .I_vs_pol    (1'b1         ),//0,负极性;1,正极性
            .O_de        (tp0_de_in    ),   
            .O_hs        (tp0_hs_in    ),
            .O_vs        (tp0_vs_in    ),
            .O_data_r    (tp0_data_r   ),   
            .O_data_g    (tp0_data_g   ),
            .O_data_b    (tp0_data_b   )
        );
    end
    endgenerate
    
    // Input data for DDR Buffer
    wire fb_vin_clk;
    wire fb_vin_vsync;
    wire [15:0] fb_vin_data;
    wire fb_vin_de;

    generate if(USE_TPG == "true")
    begin
        assign fb_vin_clk      = video_clk;
        assign fb_vin_vsync    = tp0_vs_in;
        assign fb_vin_data     = {tp0_data_r[7:3],tp0_data_g[7:2],tp0_data_b[7:3]};
        assign fb_vin_de       = tp0_de_in;
    end else begin //CMOS DATA
        assign fb_vin_clk      = cmos_16bit_clk;
        assign fb_vin_vsync    = cmos_vsync;
        assign fb_vin_data     = write_data;
        assign fb_vin_de       = cmos_16bit_wr;
    end
    endgenerate
    
    wire cmd_ready_0;
    wire [2:0] cmd_0;
    wire cmd_en_0;
    wire [27:0] addr_0;
    wire wr_data_rdy_0;
    wire wr_data_en_0;
    wire wr_data_end_0;
    wire [127:0] wr_data_0;
    wire rd_data_valid_0;
    wire rd_data_end_0;
    wire [127:0] rd_data_0;

    wire cmd_ready_1;
    wire [2:0] cmd_1;
    wire cmd_en_1;
    wire [27:0] addr_1;
    wire wr_data_rdy_1;
    wire wr_data_en_1;
    wire wr_data_end_1;
    wire [127:0] wr_data_1;
    wire rd_data_valid_1;
    wire rd_data_end_1;
    wire [127:0] rd_data_1;

    wire cmd_ready_2;
    wire [2:0] cmd_2;
    wire cmd_en_2;
    wire [27:0] addr_2;
    wire wr_data_rdy_2;
    wire wr_data_en_2;
    wire wr_data_end_2;
    wire [127:0] wr_data_2;
    wire rd_data_valid_2;
    wire rd_data_end_2;
    wire [127:0] rd_data_2;

    wire cmd_ready_3;
    wire [2:0] cmd_3;
    wire cmd_en_3;
    wire [27:0] addr_3;
    wire wr_data_rdy_3;
    wire wr_data_en_3;
    wire wr_data_end_3;
    wire [127:0] wr_data_3;
    wire rd_data_valid_3;
    wire rd_data_end_3;
    wire [127:0] rd_data_3;   

    wire cmd_ready_4;
    wire [2:0] cmd_4;
    wire cmd_en_4;
    wire [27:0] addr_4;
    wire wr_data_rdy_4;
    wire wr_data_en_4;
    wire wr_data_end_4;
    wire [127:0] wr_data_4;
    wire rd_data_valid_4;
    wire rd_data_end_4;
    wire [127:0] rd_data_4;   
    wire idle_4;

    // 视频输出数据信号
    wire [15:0] O_vout_data_0, O_vout_data_1, O_vout_data_2;
    wire [15:0] O_vout_data_3;  // 确保这个信号被定义

    wire [3:0] o_vout_fifo_empty, o_vin_fifo_full;

    // 第三个摄像头的信号（如果没有实际连接，给默认值）
    wire cmos_vsync_3 = 1'b0;
    wire cmos_href_3 = 1'b0; 
    wire cmos_pclk_3 = 1'b0;
    wire [7:0] cmos_db_3 = 8'b0;
    wire [15:0] cmos_16bit_data_3 = 16'b0;
    wire cmos_16bit_wr_3 = 1'b0;
    wire cmos_16bit_clk_3 = 1'b0;

    wire [27:0] addr_0_00,addr_1_01,addr_2_10,addr_3_11;
//    assign addr_0_00 = {4'b0000,addr_0[23:0]};
//    assign addr_1_01 = {4'b0001,addr_1[23:0]};
//    assign addr_2_10 = {4'b0010,addr_2[23:0]};
//    assign addr_3_11 = {4'b0010,addr_3[23:0]};
assign addr_0_00 = addr_0 + 27'd0;
assign addr_1_01 = addr_1 + 27'd921600;   
assign addr_2_10 = addr_2 + 27'd1843200; 
//assign addr_3_11 = addr_3 + 27'd1382400;  



wire [10:0] addr_3_1280;

// 对addr_3进行1280取余得到addr_3_1280

assign addr_3_1280 = addr_3 % 11'd1280;
reg [12:0] K ;
wire [27:0] K_frame , K_frame2;
wire [12:0] K_base;
assign K_base = K % 11'd1280;
assign K_frame = (K < 1280) ? 27'b0 : 27'd921600;
assign K_frame2 = (K < 2560) ? 27'b0 : 27'd921600;
//assign addr_3_11 = (addr_3_1280 < 1280-K_base) ? (addr_3 + K_base + K_frame + K_frame2) : ((addr_3 + 27'd920320 + K_base + K_frame + K_frame2)%27'd2764800);

 assign addr_3_11 = 28'd2764800 + 28'd24 + addr_3;
// 最终地址计算（语法修正但不改变逻辑）
// assign addr_3_11 = (addr_3_1280 < 11'd1040) ? (addr_3 + 27'd922248) : (addr_3 + 27'd1843200);
//                  assign addr_3_11 = (addr_3_1280 < 11'd1040) ? (addr_3 + 27'd922240) : (addr_3 + 27'd1843200); 
// 按键检测相关寄存器
reg key_aa13_dly;
reg key_aa13_pressed;

// 按键检测逻辑
always@(posedge dma_clk or negedge rst_n) begin
    if(!rst_n) begin
        key_aa13_dly <= 1'b0;
        key_aa13_pressed <= 1'b0;
    end else begin
        key_aa13_dly <= Key_AA13;
        // 检测按键上升沿
        key_aa13_pressed <= (key_aa13_dly && !Key_AA13);
    end
end

// K值增加逻辑（带回退机制）
always@(posedge dma_clk or negedge rst_n) begin
    if(!rst_n) begin
        K <= 13'd0;  // 初始值
    end else begin
        if(key_aa13_pressed) begin
            // 每次按键K值加128，当K>=2560时回退到K-2560
            if(K >= 13'd3840 - 13'd128) begin
                K <= K + 13'd128 - 13'd3840;
            end else begin
                K <= K + 13'd128;
            end
        end
    end
end           

    reg [1:0] hdmi_channel;
    reg key_state,key_state_dl;

    always@(posedge clk or negedge rst_n)begin
        if(!rst_n)begin
            key_state <= 1'b1;
            key_state <= 1'b1;
            hdmi_channel <= 2'b00;
        end
        else begin
            key_state <= Key_AA13;
            key_state_dl <= key_state;
            if(!key_state & key_state_dl)begin
                if(hdmi_channel == 2'b11)
                    hdmi_channel <= 2'b00;
                else
                    hdmi_channel <= hdmi_channel + 1;
            end
        end
    end        

assign off0_syn_data = O_vout_data_3;
//    assign off0_syn_data = (hdmi_channel == 2'b00) ? O_vout_data_0 :
//                         (hdmi_channel == 2'b01) ? O_vout_data_1 :
//                         (hdmi_channel == 2'b10) ? O_vout_data_2 :  O_vout_data_3;
//    assign off0_syn_de = (hdmi_channel == 2'b00) ? off0_syn_de_0 :
//                         (hdmi_channel == 2'b01) ? off0_syn_de_1 : off0_syn_de_2;
    Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst0
    ( 
        .I_rst_n              (init_calib_complete ),
        .I_dma_clk            (dma_clk          ),

        // video data input       
        .I_vin0_clk           (fb_vin_clk   ),
        .I_vin0_vs_n          (~fb_vin_vsync),//只接收负极性
        .I_vin0_de            (fb_vin_de    ),
        .I_vin0_data          (fb_vin_data  ),
        .O_vin0_fifo_full     (o_vin_fifo_full[0]),

        // video data output            
        .I_vout0_clk          (video_clk        ),
        .I_vout0_vs_n         (~syn_off0_vs     ),//只接收负极性
        .I_vout0_de           (out_de           ),
        .O_vout0_den          (off0_syn_de      ),
        .O_vout0_data         (O_vout_data_0    ),
        .O_vout0_fifo_empty   (o_vout_fifo_empty[0]),
        // ddr write request
        .I_cmd_ready          (cmd_ready_0          ),
        .O_cmd                (cmd_0                ),//0:write;  1:read
        .O_cmd_en             (cmd_en_0             ),
    //    .O_app_burst_number   (app_burst_number   ),
        .O_addr               (addr_0               ),//[ADDR_WIDTH-1:0]
        .I_wr_data_rdy        (wr_data_rdy_0        ),
        .O_wr_data_en         (wr_data_en_0         ),//
        .O_wr_data_end        (wr_data_end_0        ),//
        .O_wr_data            (wr_data_0            ),//[DATA_WIDTH-1:0]
        .O_wr_data_mask       (wr_data_mask_0       ),
        .I_rd_data_valid      (rd_data_valid_0      ),
        .I_rd_data_end        (rd_data_end_0        ),//unused 
        .I_rd_data            (rd_data_0            ),//[DATA_WIDTH-1:0]
        .I_init_calib_complete(init_calib_complete)
    ); 
    Video_Frame_Buffer_Top_1 Video_Frame_Buffer_Top_inst1(
		.I_rst_n(init_calib_complete), //input I_rst_n
		.I_dma_clk(dma_clk), //input I_dma_clk
		.I_vin0_clk(cmos_16bit_clk_1), //input I_vin0_clk
		.I_vin0_vs_n(~cmos_vsync_1), //input I_vin0_vs_n
		.I_vin0_de(cmos_16bit_wr_1), //input I_vin0_de
		.I_vin0_data(cmos_16bit_data_1), //input [15:0] I_vin0_data
		.O_vin0_fifo_full(o_vin_fifo_full[1]), //output O_vin0_fifo_full

		.I_vout0_clk(video_clk), //input I_vout0_clk
		.I_vout0_vs_n(~syn_off0_vs), //input I_vout0_vs_n
		.I_vout0_de(out_de), //input I_vout0_de
		.O_vout0_den(), //output O_vout0_den
		.O_vout0_data(O_vout_data_1), //output [15:0] O_vout0_data
		.O_vout0_fifo_empty(o_vout_fifo_empty[1]), //output O_vout0_fifo_empty

		.I_cmd_ready(cmd_ready_1), //input I_cmd_ready
		.O_cmd(cmd_1), //output [2:0] O_cmd
		.O_cmd_en(cmd_en_1), //output O_cmd_en
		.O_addr(addr_1), //output [27:0] O_addr
		.I_wr_data_rdy(wr_data_rdy_1), //input I_wr_data_rdy
		.O_wr_data_en(wr_data_en_1), //output O_wr_data_en
		.O_wr_data_end(wr_data_end_1), //output O_wr_data_end
		.O_wr_data(wr_data_1), //output [127:0] O_wr_data
		.O_wr_data_mask(), //output [15:0] O_wr_data_mask
		.I_rd_data_valid(rd_data_valid_1), //input I_rd_data_valid
		.I_rd_data_end(rd_data_end_1), //input I_rd_data_end
		.I_rd_data(rd_data_1), //input [127:0] I_rd_data
		.I_init_calib_complete(init_calib_complete) //input I_init_calib_complete
	);
    Video_Frame_Buffer_Top_2 Video_Frame_Buffer_Top_inst2(
		.I_rst_n(init_calib_complete), //input I_rst_n
		.I_dma_clk(dma_clk), //input I_dma_clk
		.I_vin0_clk(cmos_16bit_clk_2), //input I_vin0_clk
		.I_vin0_vs_n(~cmos_vsync_2), //input I_vin0_vs_n
		.I_vin0_de(cmos_16bit_wr_2), //input I_vin0_de
		.I_vin0_data(cmos_16bit_data_2), //input [15:0] I_vin0_data
		.O_vin0_fifo_full(o_vin_fifo_full[2]), //output O_vin0_fifo_full

		.I_vout0_clk(video_clk), //input I_vout0_clk
		.I_vout0_vs_n(~syn_off0_vs), //input I_vout0_vs_n
		.I_vout0_de(out_de), //input I_vout0_de
		.O_vout0_den(), //output O_vout0_den
		.O_vout0_data(O_vout_data_2), //output [15:0] O_vout0_data
		.O_vout0_fifo_empty(o_vout_fifo_empty[2]), //output O_vout0_fifo_empty

		.I_cmd_ready(cmd_ready_2), //input I_cmd_ready
		.O_cmd(cmd_2), //output [2:0] O_cmd
		.O_cmd_en(cmd_en_2), //output O_cmd_en
		.O_addr(addr_2), //output [27:0] O_addr
		.I_wr_data_rdy(wr_data_rdy_2), //input I_wr_data_rdy
		.O_wr_data_en(wr_data_en_2), //output O_wr_data_en
		.O_wr_data_end(wr_data_end_2), //output O_wr_data_end
		.O_wr_data(wr_data_2), //output [127:0] O_wr_data
		.O_wr_data_mask(), //output [15:0] O_wr_data_mask
		.I_rd_data_valid(rd_data_valid_2), //input I_rd_data_valid
		.I_rd_data_end(rd_data_end_2), //input I_rd_data_end
		.I_rd_data(rd_data_2), //input [127:0] I_rd_data
		.I_init_calib_complete(init_calib_complete) //input I_init_calib_complete
	);
Video_Frame_Buffer_Top Video_Frame_Buffer_Top_inst3(
    .I_rst_n(init_calib_complete), //input I_rst_n
    .I_dma_clk(dma_clk), //input I_dma_clk
    .I_vin0_clk(cmos_16bit_clk_3), //input I_vin0_clk - 修正连接
    .I_vin0_vs_n(~cmos_vsync_3), //input I_vin0_vs_n
    .I_vin0_de(cmos_16bit_wr_3), //input I_vin0_de - 修正连接
    .I_vin0_data(cmos_16bit_data_3), //input [15:0] I_vin0_data
    .O_vin0_fifo_full(o_vin_fifo_full[3]), //output O_vin0_fifo_full

    .I_vout0_clk(video_clk), //input I_vout0_clk
    .I_vout0_vs_n(~syn_off0_vs), //input I_vout0_vs_n
    .I_vout0_de(out_de), //input I_vout0_de
    .O_vout0_den(), //output O_vout0_den
    .O_vout0_data(O_vout_data_3), //output [15:0] O_vout0_data - 确保这个信号被正确定义
    .O_vout0_fifo_empty(o_vout_fifo_empty[3]), //output O_vout0_fifo_empty

    .I_cmd_ready(cmd_ready_3), //input I_cmd_ready
    .O_cmd(cmd_3), //output [2:0] O_cmd
    .O_cmd_en(cmd_en_3), //output O_cmd_en
    .O_addr(addr_3), //output [27:0] O_addr
    .I_wr_data_rdy(wr_data_rdy_3), //input I_wr_data_rdy
    .O_wr_data_en(wr_data_en_3), //output O_wr_data_en
    .O_wr_data_end(wr_data_end_3), //output O_wr_data_end
    .O_wr_data(wr_data_3), //output [127:0] O_wr_data
    .O_wr_data_mask(), //output [15:0] O_wr_data_mask
    .I_rd_data_valid(rd_data_valid_3), //input I_rd_data_valid
    .I_rd_data_end(rd_data_end_3), //input I_rd_data_end
    .I_rd_data(rd_data_3), //input [127:0] I_rd_data
    .I_init_calib_complete(init_calib_complete) //input I_init_calib_complete
);

wire [15:0] work_number;
wire ddr3_line_mover_start;
ddr3_line_mover ddr3_line_mover_inst
(
    // 系统时钟和复位
    .clk(dma_clk),           // 系统时钟 (DDR3操作时钟)
    .rst_n(rst_n),             // 系统复位（低有效）
    
    // 控制接口
    .start(ddr3_line_mover_start),          // 启动信号


    // DDR3控制接口
    .init_calib_complete(init_calib_complete), // DDR3初始化完成
    .ddr3_cmd_ready(cmd_ready_4),    // DDR3命令就绪
    .ddr3_cmd_en(cmd_en_4),       // DDR3命令使能
    .ddr3_cmd(cmd_4),          // DDR3命令
    .ddr3_addr(addr_4),         // DDR3地址
    .ddr3_wr_rdy(wr_data_rdy_4),       // DDR3写数据就绪
    .ddr3_wr_en(wr_data_en_4),        // DDR3写使能
    .ddr3_wr_end(wr_data_end_4),       // DDR3写结束
    .ddr3_wr_data(wr_data_4),      // DDR3写数据
    // .ddr3_wr_mask(wr_data_mask_4),      // DDR3写掩码
    .ddr3_rd_data_valid(rd_data_valid_4),// DDR3读数据有效
    .ddr3_rd_data(rd_data_4),      // DDR3读数据
    .ddr3_rd_data_end(rd_data_end_4),  // DDR3读数据结束
    // 状态指示
    .line_done(idle_4)
);

    
ddr3_arbit ddr3_inst(
    .dma_clk(dma_clk),
    .rst_n(rst_n),
    
    .cmd_ready(cmd_ready),
    .cmd(cmd),
    .cmd_en(cmd_en),
    .addr(addr),
    .wr_data_rdy(wr_data_rdy),
    .wr_data_en(wr_data_en),
    .wr_data_end(wr_data_end),
    .wr_data(wr_data),
    .rd_data_valid(rd_data_valid),
    .rd_data_end(rd_data_end),
    .rd_data(rd_data),
    
    .cmd_ready_0(cmd_ready_0),
    .cmd_0(cmd_0),
    .cmd_en_0(cmd_en_0),
    .addr_0(addr_0_00),
    .wr_data_rdy_0(wr_data_rdy_0),
    .wr_data_en_0(wr_data_en_0),
    .wr_data_end_0(wr_data_end_0),
    .wr_data_0(wr_data_0),
    .rd_data_valid_0(rd_data_valid_0),
    .rd_data_end_0(rd_data_end_0),
    .rd_data_0(rd_data_0),

    .cmd_ready_1(cmd_ready_1),
    .cmd_1(cmd_1),
    .cmd_en_1(cmd_en_1),
    .addr_1(addr_1_01),
    .wr_data_rdy_1(wr_data_rdy_1),
    .wr_data_en_1(wr_data_en_1),
    .wr_data_end_1(wr_data_end_1),
    .wr_data_1(wr_data_1),
    .rd_data_valid_1(rd_data_valid_1),
    .rd_data_end_1(rd_data_end_1),
    .rd_data_1(rd_data_1),

    .cmd_ready_2(cmd_ready_2),
    .cmd_2(cmd_2),
    .cmd_en_2(cmd_en_2),
    .addr_2(addr_2_10),
    .wr_data_rdy_2(wr_data_rdy_2),
    .wr_data_en_2(wr_data_en_2),
    .wr_data_end_2(wr_data_end_2),
    .wr_data_2(wr_data_2),
    .rd_data_valid_2(rd_data_valid_2),
    .rd_data_end_2(rd_data_end_2),
    .rd_data_2(rd_data_2),

    // 新增第三个buffer接口
    .cmd_ready_3(cmd_ready_3),
    .cmd_3(cmd_3),
    .cmd_en_3(cmd_en_3),
    .addr_3(addr_3_11),  // 注意：这里是27位地址，与其他通道不同
    .wr_data_rdy_3(wr_data_rdy_3),
    .wr_data_en_3(wr_data_en_3),
    .wr_data_end_3(wr_data_end_3),
    .wr_data_3(wr_data_3),
    .rd_data_valid_3(rd_data_valid_3),
    .rd_data_end_3(rd_data_end_3),
    .rd_data_3(rd_data_3),

    .idle_4(idle_4),
    .start_4(ddr3_line_mover_start),
    .cmd_4(cmd_4),
    .cmd_en_4(cmd_en_4),
    .cmd_ready_4(cmd_ready_4),
    .addr_4(addr_4),  // 注意：这里是27:0，与其他通道的28:0不同
    .wr_data_rdy_4(wr_data_rdy_4),
    .wr_data_en_4(wr_data_en_4),
    .wr_data_end_4(wr_data_end_4),
    .wr_data_4(wr_data_4),
    .rd_data_valid_4(rd_data_valid_4),
    .rd_data_end_4(rd_data_end_4),
    .rd_data_4(rd_data_4)
);
    reg [1:0] DDRPLL_LOCK_R;
    wire DDRPLL_STOP;
    reg DDRPLL_STOP_R;
    reg DDRPLL_WR;

    always@(posedge clk)
    begin
        DDRPLL_LOCK_R[0] <= DDR_pll_lock;
        DDRPLL_LOCK_R[1] <= DDRPLL_LOCK_R[0];

        DDRPLL_STOP_R <= DDRPLL_STOP;

        if(!DDRPLL_LOCK_R[1])
            DDRPLL_WR <= 1'b0;
        else if(~DDRPLL_STOP & DDRPLL_STOP_R) begin
            DDRPLL_WR <= 1'b1;
        end else if(DDRPLL_STOP & (~DDRPLL_STOP_R))begin
            DDRPLL_WR <= 1'b1;
        end else begin
            DDRPLL_WR <= 1'b0;
        end
    end
    // for 25K & 60K, not for 138K
    pll_mDRP_intf DDRPLL_STOP_inst(
        .clk(clk),
        .rst_n(rst_n),
        .pll_lock(DDRPLL_LOCK_R[1]),
        .wr(DDRPLL_WR),
        .mdrp_inc(DDRPLL_MDR_AINC),
        .mdrp_op(DDRPLL_MDR_OPC),
        .mdrp_wdata(DDRPLL_MDR_DI),
        .mdrp_rdata(DDRPLL_MDR_DO)
    );    

    DDR3MI u_ddr3 
    (
        .clk                (clk                ),
        .memory_clk         (memory_clk         ),
        .pll_stop           (DDRPLL_STOP        ),
        .pll_lock           (DDR_pll_lock       ),
        .rst_n              (rst_n              ),
    //    .app_burst_number   (app_burst_number   ),
        .cmd_ready          (cmd_ready          ),
        .cmd                (cmd                ),
        .cmd_en             (cmd_en             ),
        .addr               (addr               ),
        .wr_data_rdy        (wr_data_rdy        ),
        .wr_data            (wr_data            ),
        .wr_data_en         (wr_data_en         ),
        .wr_data_end        (wr_data_end        ),
        .wr_data_mask       (wr_data_mask       ),
        .rd_data            (rd_data            ),
        .rd_data_valid      (rd_data_valid      ),
        .rd_data_end        (rd_data_end        ),
        .sr_req             (1'b0               ),
        .ref_req            (1'b0               ),
        .sr_ack             (                   ),
        .ref_ack            (                   ),
        .init_calib_complete(init_calib_complete),
        .clk_out            (dma_clk            ),
        .burst              (1'b1               ),
        // mem interface
        .ddr_rst            (                 ),
        .O_ddr_addr         (ddr_addr         ),
        .O_ddr_ba           (ddr_bank         ),
        .O_ddr_cs_n         (ddr_cs           ),
        .O_ddr_ras_n        (ddr_ras          ),
        .O_ddr_cas_n        (ddr_cas          ),
        .O_ddr_we_n         (ddr_we           ),
        .O_ddr_clk          (ddr_ck           ),
        .O_ddr_clk_n        (ddr_ck_n         ),
        .O_ddr_cke          (ddr_cke          ),
        .O_ddr_odt          (ddr_odt          ),
        .O_ddr_reset_n      (ddr_reset_n      ),
        .O_ddr_dqm          (ddr_dm           ),
        .IO_ddr_dq          (ddr_dq           ),
        .IO_ddr_dqs         (ddr_dqs          ),
        .IO_ddr_dqs_n       (ddr_dqs_n        )
    );


    // DDR Output video Timing Align
    //---------------------------------------------
    wire [4:0] lcd_r,lcd_b;
    wire [5:0] lcd_g;
    wire lcd_vs,lcd_de,lcd_hs,lcd_dclk;
    reg  [1:0]  Pout_hs_dn;
    reg  [1:0]  Pout_vs_dn;
    reg  [1:0]  Pout_de_dn;

    
    assign lcd_vs      			  = Pout_vs_dn[1];//syn_off0_vs;
    assign lcd_hs      			  = Pout_hs_dn[1];//syn_off0_hs;
    assign lcd_de      			  = Pout_de_dn[1];//off0_syn_de 是延迟两时钟周期后的输入de信号，在此也可以用Pout_de_dn[1]代替
    assign lcd_dclk    			  = video_clk;//video_clk_phs;

    reg [15:0] off0_syn_data_user;

    always@(posedge video_clk or negedge rst_n)
    begin
        if(!rst_n)
            begin                          
                Pout_hs_dn  <= {2'b11};
                Pout_vs_dn  <= {2'b11}; 
                Pout_de_dn  <= {2'b00}; 
            end
        else 
            begin           
                if (lcd_y < 240) begin
                    off0_syn_data_user <= 16'hFFFF; // White
                end else if (lcd_y >= 480)begin
                    off0_syn_data_user <= 16'hFFFF; // White
                end else begin
                    off0_syn_data_user <= off0_syn_data[15:0];
                end
                Pout_hs_dn  <= {Pout_hs_dn[0],syn_off0_hs};
                Pout_vs_dn  <= {Pout_vs_dn[0],syn_off0_vs}; 
                Pout_de_dn  <= {Pout_de_dn[0],out_de}; 
            end
    end
    // RGB Data
    
    assign {lcd_r,lcd_g,lcd_b}    = off0_syn_de ? off0_syn_data_user : 16'h0000;//{r,g,b}
    //==============================================================================
    //TMDS TX(HDMI4)
    wire serial_clk;
    wire hdmi4_rst_n;

    TMDS_PLL u_tmds_pll(
        .clkin     (clk              ),     //input clk 
        .clkout0   (serial_clk       ),     //output clk x5ni
        .clkout1   (video_clk        ),     //output clk x1
        .lock      (TMDS_DDR_pll_lock)      //output lock
        );

    assign hdmi4_rst_n = rst_n & TMDS_DDR_pll_lock;

    wire dvi0_rgb_clk;
    wire dvi0_rgb_vs ;
    wire dvi0_rgb_hs ;
    wire dvi0_rgb_de ;
    wire [7:0] dvi0_rgb_r  ;
    wire [7:0] dvi0_rgb_g  ;
    wire [7:0] dvi0_rgb_b  ;

    wire dvi1_rgb_clk;
    wire dvi1_rgb_vs ;
    wire dvi1_rgb_hs ;
    wire dvi1_rgb_de ;
    wire [7:0] dvi1_rgb_r  ;
    wire [7:0] dvi1_rgb_g  ;
    wire [7:0] dvi1_rgb_b  ;

generate if(DDR_BYPASS == "true")
begin
    //DVI directly use TPG video
    assign dvi0_rgb_clk = video_clk ;
    assign dvi0_rgb_vs  = tp0_vs_in ;
    assign dvi0_rgb_hs  = tp0_hs_in ;
    assign dvi0_rgb_de  = tp0_de_in ;
    assign dvi0_rgb_r   = tp0_data_r;
    assign dvi0_rgb_g   = tp0_data_g;
    assign dvi0_rgb_b   = tp0_data_b;
end else begin
    assign dvi0_rgb_clk = lcd_dclk;
    assign dvi0_rgb_vs  = lcd_vs;
    assign dvi0_rgb_hs  = lcd_hs;
    assign dvi0_rgb_de  = lcd_de;
    assign dvi0_rgb_r   = {lcd_r,lcd_r[4:2]};
    assign dvi0_rgb_g   = {lcd_g,lcd_g[5:4]};
    assign dvi0_rgb_b   = {lcd_b,lcd_b[4:2]};
end
endgenerate

    DVI_TX_Top DVI_TX_Top_inst0
    (
        .I_rst_n       (hdmi4_rst_n   ),  //asynchronous reset, low active
        .I_serial_clk  (serial_clk    ),

        //CMOS
        .I_rgb_clk     (dvi0_rgb_clk),  //pixel clock
        .I_rgb_vs      (dvi0_rgb_vs ), 
        .I_rgb_hs      (dvi0_rgb_hs ),    
        .I_rgb_de      (dvi0_rgb_de ), 
        .I_rgb_r       (dvi0_rgb_r  ), 
        .I_rgb_g       (dvi0_rgb_g  ),  
        .I_rgb_b       (dvi0_rgb_b  ),  

        .O_tmds_clk_p  (tmds_clk_p_0  ),
        .O_tmds_clk_n  (tmds_clk_n_0  ),
        .O_tmds_data_p (tmds_d_p_0    ),  //{r,g,b}
        .O_tmds_data_n (tmds_d_n_0    )
    );
    UDP UDP_test(
	.clk(clk),
	.rst_n(rst_n), 			
    //数据输入接口
    .cmos_db(off0_syn_data_user),
    .cmos_pclk(video_clk),
    .cmos_vsync(syn_off0_vs),
    .cmos_href(~syn_off0_hs),
    //GMII/RGMII接口
	.PHY_CLK(PHY_CLK),
	.RGMII_GTXCLK(RGMII_GTXCLK),
	.RGMII_RST_N(RGMII_RST_N), 	
	.RGMII_TXD(RGMII_TXD), 		
	.RGMII_TXEN(RGMII_TXEN)
	);
    // UDP UDP_test(
	// .clk(clk),
	// .rst_n(rst_n), 			
    // //数据输入接口
    // .cmos_db(cmos_16bit_data),
    // .cmos_pclk(cmos_16bit_clk),
    // .cmos_vsync(cmos_vsync),
    // .cmos_href(cmos_href),
    // //GMII/RGMII接口
	// .PHY_CLK(PHY_CLK),
	// .RGMII_GTXCLK(RGMII_GTXCLK),
	// .RGMII_RST_N(RGMII_RST_N), 	
	// .RGMII_TXD(RGMII_TXD), 		
	// .RGMII_TXEN(RGMII_TXEN)
	// );
endmodule