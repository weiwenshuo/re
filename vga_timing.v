module vga_timing#(
	parameter H_ACTIVE = 16'd1280,  	//水平有效显示时间（像素数）
	parameter H_FP 	   = 16'd110,		//水平前肩（像素数）
	parameter H_SYNC   = 16'd40,   		//水平同步时间（像素数）
	parameter H_BP	   = 16'd220,  		//水平后肩（像素数）
	parameter V_ACTIVE = 16'd720,		//垂直有效显示时间（行数）
	parameter V_FP     = 16'd5,  		//垂直前肩（行数）
	parameter V_SYNC   = 16'd5,  		//垂直同步时间（行数）
	parameter V_BP     = 16'd20, 		//垂直后肩（行数）
	parameter HS_POL   = 1'b1,   		//水平同步极性，1：正极性，0：负极性
	parameter VS_POL   = 1'b1    		//垂直同步极性，1：正极性，0：负极性
)(
	input                 clk,           //像素时钟
	input                 rst,           //复位信号，高电平有效
	output                hs,            //水平同步信号
	output                vs,            //垂直同步信号
	output                de,            //视频数据有效信号

	output reg [9:0] active_x,           //视频X坐标位置 
	output reg [9:0] active_y            //视频Y坐标位置 
	
	);

	//计算水平总周期（像素数）
	parameter H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;
	//计算垂直总周期（行数）
	parameter V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;


/***** 参考配置信息 *******/

//1280x720视频格式配置
///-----------------------------------------------------------------------
/*
parameter H_ACTIVE = 16'd1280;           //水平有效显示时间（像素数）
parameter H_FP = 16'd110;                //水平前肩（像素数）
parameter H_SYNC = 16'd40;               //水平同步时间（像素数）
parameter H_BP = 16'd220;                //水平后肩（像素数）
parameter V_ACTIVE = 16'd720;            //垂直有效显示时间（行数）
parameter V_FP  = 16'd5;                 //垂直前肩（行数）
parameter V_SYNC  = 16'd5;               //垂直同步时间（行数）
parameter V_BP  = 16'd20;                //垂直后肩（行数）
parameter HS_POL = 1'b1;                 //水平同步极性，1：正极性，0：负极性
parameter VS_POL = 1'b1;                 //垂直同步极性，1：正极性，0：负极性
parameter H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP; //水平总周期（像素数）
parameter V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP; //垂直总周期（行数）
*/

//1280x720_30_DMT视频格式配置
/*//-----------------------------------------------------------------------
parameter H_ACTIVE = 16'd1280;           //水平有效显示时间（像素数）
parameter H_FP = 16'd1760;                //水平前肩（像素数）
parameter H_SYNC = 16'd40;               //水平同步时间（像素数）
parameter H_BP = 16'd220;                //水平后肩（像素数）
parameter V_ACTIVE = 16'd720;            //垂直有效显示时间（行数）
parameter V_FP  = 16'd5;                 //垂直前肩（行数）
parameter V_SYNC  = 16'd5;               //垂直同步时间（行数）
parameter V_BP  = 16'd20;                //垂直后肩（行数）
parameter HS_POL = 1'b1;                 //水平同步极性，1：正极性，0：负极性
parameter VS_POL = 1'b1;                 //垂直同步极性，1：正极性，0：负极性

parameter H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP; //水平总周期（像素数）
parameter V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP; //垂直总周期（行数）
*/

/*
//800x600视频格式配置
///-----------------------------------------------------------------------
parameter H_ACTIVE = 16'd800;           //水平有效显示时间（像素数）
parameter H_FP = 16'd40;                //水平前肩（像素数）
parameter H_SYNC = 16'd128;             //水平同步时间（像素数）
parameter H_BP = 16'd88;                //水平后肩（像素数）
parameter V_ACTIVE = 16'd600;           //垂直有效显示时间（行数）
parameter V_FP  = 16'd1;                //垂直前肩（行数）
parameter V_SYNC  = 16'd4;              //垂直同步时间（行数）
parameter V_BP  = 16'd23;               //垂直后肩（行数）
parameter HS_POL = 1'b1;                //水平同步极性，1：正极性，0：负极性
parameter VS_POL = 1'b1;                //垂直同步极性，1：正极性，0：负极性
parameter H_TOTAL = H_ACTIVE + H_FP + H_SYNC + H_BP; //水平总周期（像素数）
parameter V_TOTAL = V_ACTIVE + V_FP + V_SYNC + V_BP; //垂直总周期（行数）
*/

//水平同步寄存器
reg hs_reg;                      
//垂直同步寄存器
reg vs_reg;                      
//水平计数器（12位）
reg[11:0] h_cnt;                 
//垂直计数器（12位）
reg[11:0] v_cnt;                 

//水平视频有效标志
reg h_active;                    
//垂直视频有效标志
reg v_active;                    

//输出水平同步信号
assign hs = hs_reg;
//输出垂直同步信号
assign vs = vs_reg;
//输出视频数据有效信号（水平和垂直都有效时才有效）
assign de = h_active & v_active;


//水平计数器（列计数）
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时清零
		h_cnt <= 12'd0;
	else if(h_cnt == H_TOTAL - 1)  //达到水平总周期时归零
		h_cnt <= 12'd0;
	else  //否则递增
		h_cnt <= h_cnt + 12'd1;
end

//X坐标计数（显示区域内的水平位置）
always@(posedge clk)
begin
	if(h_cnt >= H_FP + H_SYNC + H_BP)  //在有效显示区域内
		active_x <= h_cnt - (H_FP[11:0] + H_SYNC[11:0] + H_BP[11:0]);
	else
		active_x <= active_x;  //保持原值
end

//Y坐标计数（显示区域内的垂直位置）
always@(posedge clk)
begin	
	if(v_cnt >= V_FP + V_SYNC + V_BP)  //在有效显示区域内
		active_y <= v_cnt - (V_FP[11:0] + V_SYNC[11:0] + V_BP[11:0]);
	else
		active_y <= active_y;  //保持原值
end

//垂直计数器（行计数）
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时清零
		v_cnt <= 12'd0;
	else if(h_cnt == H_FP  - 1)  //每完成一行计数
		if(v_cnt == V_TOTAL - 1)  //达到垂直总周期时归零
			v_cnt <= 12'd0;
		else  //否则递增
			v_cnt <= v_cnt + 12'd1;
	else
		v_cnt <= v_cnt;  //保持原值
end

//生成水平同步信号
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时同步信号为0
		hs_reg <= 1'b0;
	else if(h_cnt == H_FP - 1)  //到达前肩结束位置时开始同步
		hs_reg <= HS_POL;
	else if(h_cnt == H_FP + H_SYNC - 1)  //到达同步结束位置时结束同步
		hs_reg <= ~hs_reg;
	else
		hs_reg <= hs_reg;  //保持原值
end

//生成水平有效信号
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时无效
		h_active <= 1'b0;
	else if(h_cnt == H_FP + H_SYNC + H_BP - 1)  //到达有效显示开始位置
		h_active <= 1'b1;
	else if(h_cnt == H_TOTAL - 1)  //到达行结束位置
		h_active <= 1'b0;
	else
		h_active <= h_active;  //保持原值
end

//生成垂直同步信号
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时同步信号为0
		vs_reg <= 1'd0;
	else if((v_cnt == V_FP - 1) && (h_cnt == H_FP - 1))  //到达垂直前肩结束位置时开始同步
		vs_reg <= HS_POL;
	else if((v_cnt == V_FP + V_SYNC - 1) && (h_cnt == H_FP - 1))  //到达垂直同步结束位置
		vs_reg <= ~vs_reg;  
	else
		vs_reg <= vs_reg;  //保持原值
end

//生成垂直有效信号
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)  //复位时无效
		v_active <= 1'd0;
	else if((v_cnt == V_FP + V_SYNC + V_BP - 1) && (h_cnt == H_FP - 1))  //到达有效显示开始位置
		v_active <= 1'b1;
	else if((v_cnt == V_TOTAL - 1) && (h_cnt == H_FP - 1))  //到达帧结束位置
		v_active <= 1'b0;   
	else
		v_active <= v_active;  //保持原值
end


endmodule 