`include "VGA_Param.h" 

module DE2_115_CAMERA(

	//////////// CLOCK //////////
	CLOCK2_50,

	//////////// LED //////////
	LEDG,
	LEDR,

	//////////// KEY //////////
	KEY,


	//////////// SW //////////
	SW,

	//////////// SEG7 //////////
	HEX0,
	HEX1,
	HEX2,
	HEX3,
	HEX4,
	HEX5,
	HEX6,
	HEX7,

	//////////// VGA //////////
	VGA_B,
	VGA_BLANK_N,
	VGA_CLK,
	VGA_G,
	VGA_HS,
	VGA_R,
	VGA_SYNC_N,
	VGA_VS,



	//////////// I2C for Audio Tv-Decoder  //////////
	I2C_SCLK,
	I2C_SDAT,

	//////////// SDRAM //////////
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_DQM,
	DRAM_RAS_N,
	DRAM_WE_N,

	//////////// SRAM //////////
	SRAM_ADDR,
	SRAM_CE_N,
	SRAM_DQ,
	SRAM_LB_N,
	SRAM_OE_N,
	SRAM_UB_N,
	SRAM_WE_N,


	//////////// GPIO, GPIO connect to D5M - 5M Pixel Camera //////////
	D5M_D,
	D5M_FVAL,
	D5M_LVAL,
	D5M_PIXLCLK,
	D5M_RESET_N,
	D5M_SCLK,
	D5M_SDATA,
	D5M_STROBE,
	D5M_TRIGGER,
	D5M_XCLKIN 
);

parameter LINE_WIDTH = 800;
parameter N_LINES_IN_BUF = 30;
parameter LINE_BUF_SIZE = LINE_WIDTH * (N_LINES_IN_BUF - 1) + N_LINES_IN_BUF * 1;
parameter PIX_SIZE = 12;
parameter KERNEL_SIZE = 3;
parameter HARRIS_THRESHOLD = 2000;

input		          		CLOCK2_50;


//////////// LED //////////
output		     [8:0]		LEDG;
output		    [17:0]		LEDR;

//////////// KEY //////////
input		     [3:0]		KEY;


//////////// SW //////////
input		    [17:0]		SW;

//////////// SEG7 //////////
output		     [6:0]		HEX0;
output		     [6:0]		HEX1;
output		     [6:0]		HEX2;
output		     [6:0]		HEX3;
output		     [6:0]		HEX4;
output		     [6:0]		HEX5;
output		     [6:0]		HEX6;
output		     [6:0]		HEX7;

//////////// VGA //////////
output		     [7:0]		VGA_B;
output		          		VGA_BLANK_N;
output		          		VGA_CLK;
output		     [7:0]		VGA_G;
output		          		VGA_HS;
output		     [7:0]		VGA_R;
output		          		VGA_SYNC_N;
output		          		VGA_VS;



//////////// I2C for Audio Tv-Decoder  //////////
output		          		I2C_SCLK;
inout		          		I2C_SDAT;

//////////// SDRAM //////////
output		    [12:0]		DRAM_ADDR;
output		     [1:0]		DRAM_BA;
output		          		DRAM_CAS_N;
output		          		DRAM_CKE;
output		          		DRAM_CLK;
output		          		DRAM_CS_N;
inout		    [31:0]		DRAM_DQ;
output		     [3:0]		DRAM_DQM;
output		          		DRAM_RAS_N;
output		          		DRAM_WE_N;

//////////// SRAM //////////
output		    [19:0]		SRAM_ADDR;
output		          		SRAM_CE_N;
inout		    [15:0]		SRAM_DQ;
output		          		SRAM_LB_N;
output		          		SRAM_OE_N;
output		          		SRAM_UB_N;
output		          		SRAM_WE_N;

//////////// GPIO, GPIO connect to D5M - 5M Pixel Camera //////////
input		    [11:0]		D5M_D;
input		          		D5M_FVAL;
input		          		D5M_LVAL;
input		          		D5M_PIXLCLK;
output		          		D5M_RESET_N;
output		          		D5M_SCLK;
inout		          		D5M_SDATA;
input		          		D5M_STROBE;
output		          		D5M_TRIGGER;
output		          		D5M_XCLKIN;

//=======================================================
//  REG/WIRE declarations
//=======================================================
wire	[15:0]	Read_DATA1;
wire	[15:0]	Read_DATA2;

wire	[11:0]	mCCD_DATA;
wire			mCCD_DVAL;
wire			mCCD_DVAL_d;
wire	[15:0]	X_Cont;
wire	[15:0]	Y_Cont;
wire	[9:0]	X_ADDR;
wire	[31:0]	Frame_Cont;
wire			DLY_RST_0;
wire			DLY_RST_1;
wire			DLY_RST_2;
wire			DLY_RST_3;
wire			DLY_RST_4;
wire			Read;
reg		[11:0]	rCCD_DATA;
reg				rCCD_LVAL;
reg				rCCD_FVAL;
wire	[11:0]	sCCD_R;
wire	[11:0]	sCCD_G;
wire	[11:0]	sCCD_B;
wire	[11:0]	sCCD_R_raw;
wire	[11:0]	sCCD_G_raw;
wire	[11:0]	sCCD_B_raw;
wire	[11:0]	memCCD_R;
wire[11:0]	memCCD_G;
wire	[11:0]	memCCD_B;

wire	[11:0]	sCCD_Gray;

wire			sCCD_DVAL;

wire			sdram_ctrl_clk;
wire	[9:0]	oVGA_R;   				//	VGA Red[9:0]
wire	[9:0]	oVGA_G;	 				//	VGA Green[9:0]
wire	[9:0]	oVGA_B;   				//	VGA Blue[9:0]

//power on start
wire             auto_start;

//Line buffers
reg  [PIX_SIZE : 0] line_buf [0 : LINE_BUF_SIZE - 1];
reg [PIX_SIZE - 1 : 0] kernel_out;

reg [PIX_SIZE + 3 : 0] y_grad_window[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];
reg [PIX_SIZE + 3 : 0] x_grad_window[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];
reg [PIX_SIZE + 3 : 0] x_grad_window_reg[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];

reg [PIX_SIZE + 3 : 0] top_window[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];
reg [PIX_SIZE + 3 : 0] bottom_window[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];
reg [PIX_SIZE + 3 : 0] left_window[0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];
reg [PIX_SIZE + 3 : 0] right_window [0 : KERNEL_SIZE - 1][0 : KERNEL_SIZE - 1];

reg [13 : 0] y_2_sum;
reg [13 : 0] x_2_sum;
reg [13 : 0] x_y_sum;

wire [12:0]		H_Cont;
wire [12:0]		V_Cont;
wire [9:0] 	VGA_red;
wire [9:0] 	VGA_green;
wire [9:0] 	VGA_blue;


reg [31 : 0] harris_measure;
reg [31 : 0] x_2_times_y_2;
reg [31 : 0] x_y_squared;
reg [31 : 0] x2_add_y2,trace,trace_k;
reg [31 : 0] det,det_buff;
reg [18 : 0] pixel_storage_count; 



//Lindsey More
reg corner;  
reg [31:0] Image_Address_In; 
wire [31:0] Image_Address_Out; 

wire [12:0]		H_Cont_true;
wire [12:0]		V_Cont_true;
wire	[15:0]	X_Cont_true;
wire	[15:0]	Y_Cont_true;

//reg [799:0] test1 [599:0]; 

reg corner_frame_input, color_frame_input;
wire corner_frame_output, color_frame_output;

reg test_clk; 

//Loop integers
integer i,j,k;

//Ref integers
integer start_pix;
integer start_pix_lin, start_pix_lin_1;
integer sum, avg, w_count, b_count; 

//=======================================================
//  Structural coding
//=======================================================
// D5M
assign	D5M_TRIGGER	=	1'b1;  // tRIGGER
assign	D5M_RESET_N	=	DLY_RST_1;
assign  VGA_CTRL_CLK = ~VGA_CLK;

assign	LEDR		=	X_Cont;
assign	LEDG		=	(Y_Cont > 16'h00FF )?8'hFF:8'h00;

//fetch the high 8 bits
assign  VGA_R = oVGA_R[9:2];
assign  VGA_G = oVGA_G[9:2];
assign  VGA_B = oVGA_B[9:2];

//D5M read 
always@(posedge D5M_PIXLCLK)
begin
	rCCD_DATA	<=	D5M_D;
	rCCD_LVAL	<=	D5M_LVAL;
	rCCD_FVAL	<=	D5M_FVAL;
end

//auto start when power on
assign auto_start = ((KEY[0])&&(DLY_RST_3)&&(!DLY_RST_4))? 1'b1:1'b0;
//Reset module
Reset_Delay_Test	u2	(	.iCLK(CLOCK2_50),
							.iRST(KEY[0]),
							.oRST_0(DLY_RST_0),
							.oRST_1(DLY_RST_1),
							.oRST_2(DLY_RST_2),
							.oRST_3(DLY_RST_3),
							.oRST_4(DLY_RST_4)
						);
//D5M image capture
CCD_Capture			u3	(	.oDATA(mCCD_DATA),
							.oDVAL(mCCD_DVAL),
							.oX_Cont(X_Cont),
							.oY_Cont(Y_Cont),
							.oX_Cont_true(X_Cont_true), 
							.oY_Cont_true(Y_Cont_true), 
							.oFrame_Cont(Frame_Cont),
							.iDATA(rCCD_DATA),
							.iFVAL(rCCD_FVAL),
							.iLVAL(rCCD_LVAL),
							.iSTART(!KEY[3]|auto_start),
							.iEND(!KEY[2]),
							.iCLK(~D5M_PIXLCLK),
							.iRST(DLY_RST_2)
						);
//D5M raw date convert to RGB data
`ifdef VGA_640x480p60
RAW2RGB				u4	(	.iCLK(D5M_PIXLCLK),
							.iRST(DLY_RST_1),
							.iDATA(mCCD_DATA),
							.iDVAL(mCCD_DVAL),
							.oRed(sCCD_R),
							.oGreen(sCCD_G),
							.oBlue(sCCD_B),
							.oDVAL(sCCD_DVAL),
							.iX_Cont(X_Cont),
							.iY_Cont(Y_Cont)
						);
`else
RAW2RGB				u4	(	.iCLK(D5M_PIXLCLK),
							.iRST_n(DLY_RST_1),
							.iData(mCCD_DATA),
							.iDval(mCCD_DVAL),
							.oRed(sCCD_R),
							.oGreen(sCCD_G),
							.oBlue(sCCD_B),
							.oDval(sCCD_DVAL),
							.iZoom(0),
							.iX_Cont(X_Cont),
							.iY_Cont(Y_Cont)
						);
`endif
//Frame count display
SEG7_LUT_8 			u5	(	.oSEG0(HEX0),.oSEG1(HEX1),
							.oSEG2(HEX2),.oSEG3(HEX3),
							.oSEG4(HEX4),.oSEG5(HEX5),
							.oSEG6(HEX6),.oSEG7(HEX7),
							.iDIG(Frame_Cont[31:0])
						);

sdram_pll 			u6	(
							.inclk0(CLOCK2_50),
							.c0(sdram_ctrl_clk),
							.c1(DRAM_CLK),
							.c2(D5M_XCLKIN), //25M
`ifdef VGA_640x480p60
							.c3(VGA_CLK)     //25M 
`else
						    .c4(VGA_CLK)     //40M 	
`endif
						);

//SDRam Read and Write as Frame Buffer
Sdram_Control	u7	(	//	HOST Side						
						    .RESET_N(KEY[0]),
							.CLK(sdram_ctrl_clk),

							//	FIFO Write Side 1
							.WR1_DATA({1'b0,sCCD_G[11:7],sCCD_B[11:2]}),
							.WR1(sCCD_DVAL),
							.WR1_ADDR(0),
`ifdef VGA_640x480p60
						    .WR1_MAX_ADDR(640*480/2),
						    .WR1_LENGTH(8'h50),
`else
							.WR1_MAX_ADDR(800*600/2),
							.WR1_LENGTH(8'h80),
`endif							
							.WR1_LOAD(!DLY_RST_0),
							.WR1_CLK(D5M_PIXLCLK),

							//	FIFO Write Side 2
							.WR2_DATA({1'b0,sCCD_G[6:2],sCCD_R[11:2]}),
							.WR2(sCCD_DVAL),
							.WR2_ADDR(23'h100000),
`ifdef VGA_640x480p60
						    .WR2_MAX_ADDR(23'h100000+640*480/2),
							.WR2_LENGTH(8'h50),
`else							
							.WR2_MAX_ADDR(23'h100000+800*600/2),
							.WR2_LENGTH(8'h80),
`endif	
							.WR2_LOAD(!DLY_RST_0),
							.WR2_CLK(D5M_PIXLCLK),

							//	FIFO Read Side 1
						    .RD1_DATA(Read_DATA1),
				        	.RD1(Read),
				        	.RD1_ADDR(0),
`ifdef VGA_640x480p60
						    .RD1_MAX_ADDR(640*480/2),
							.RD1_LENGTH(8'h50),
`else
							.RD1_MAX_ADDR(800*600/2),
							.RD1_LENGTH(8'h80),
`endif
							.RD1_LOAD(!DLY_RST_0),
							.RD1_CLK(~VGA_CTRL_CLK),
							
							//	FIFO Read Side 2
						    .RD2_DATA(Read_DATA2),
							.RD2(Read),
							.RD2_ADDR(23'h100000),
`ifdef VGA_640x480p60
						    .RD2_MAX_ADDR(23'h100000+640*480/2),
							.RD2_LENGTH(8'h50),
`else
							.RD2_MAX_ADDR(23'h100000+800*600/2),
							.RD2_LENGTH(8'h80),
`endif
				        	.RD2_LOAD(!DLY_RST_0),
							.RD2_CLK(~VGA_CTRL_CLK),
							
							//	SDRAM Side
						    .SA(DRAM_ADDR),
							.BA(DRAM_BA),
							.CS_N(DRAM_CS_N),
							.CKE(DRAM_CKE),
							.RAS_N(DRAM_RAS_N),
							.CAS_N(DRAM_CAS_N),
							.WE_N(DRAM_WE_N),
							.DQ(DRAM_DQ),
							.DQM(DRAM_DQM)
						);
//D5M I2C control
I2C_CCD_Config 		u8	(	//	Host Side
							.iCLK(CLOCK2_50),
							.iRST_N(DLY_RST_2),
							.iEXPOSURE_ADJ(KEY[1]),
							.iEXPOSURE_DEC_p(SW[0]),
							.iZOOM_MODE_SW(0),
							//	I2C Side
							.I2C_SCLK(D5M_SCLK),
							.I2C_SDAT(D5M_SDATA)
						);
//VGA DISPLAY
VGA_Controller		u1	(	//	Host Side
							.oRequest(Read),
//							.iRed(Read_DATA2[9:0]),
//							.iGreen({Read_DATA1[14:10],Read_DATA2[14:10]}),
//							.iBlue(Read_DATA1[9:0]),
							.iRed(VGA_red),
							.iGreen(VGA_green),
							.iBlue(VGA_blue),
							//	VGA Side
							.oVGA_R(oVGA_R),
							.oVGA_G(oVGA_G),
							.oVGA_B(oVGA_B),
							.oVGA_H_SYNC(VGA_HS),
							.oVGA_V_SYNC(VGA_VS),
							.oVGA_SYNC(VGA_SYNC_N),
							.oVGA_BLANK(VGA_BLANK_N),
							//	Control Signal
							.iCLK(VGA_CTRL_CLK),
							.iRST_N(DLY_RST_2),
							.iZOOM_MODE_SW(0),
							.H_Cont(H_Cont),
							.V_Cont(V_Cont), 
							.H_Cont_true(H_Cont_true),
							.V_Cont_true(V_Cont_true)
						);
						
//Convert to gray scale
assign sCCD_Gray = sCCD_R/3 + sCCD_G/3 + sCCD_B/3;
						
//Line buffer
always @(posedge D5M_PIXLCLK) begin
	if(sCCD_DVAL) begin
		line_buf[23229] <= {1'b0, sCCD_Gray[11:0]};
		for(i=23229; i >= 1;i=i-1) begin
			if( i == 21626 && corner == 1) begin 
				line_buf[i-1] <= {1'b1, line_buf[i][11:0]};
			end 
			else begin 
				line_buf[i-1] <= line_buf[i];
			end
		end
	end
end

//Calculate X and Y gardients for a givin window within the kernal
always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
			for(i = 0; i < KERNEL_SIZE; i=i+1) begin
				for(j = 0; j < KERNEL_SIZE; j=j+1) begin
					//start of 5x5
					start_pix = 20025 + j*LINE_WIDTH + i;
					top_window[i][j] = 0;
					bottom_window[i][j] = 0;
					left_window[i][j] = 0;
					right_window[i][j] = 0;
					for(k = 0; k < KERNEL_SIZE; k=k+1) begin
						top_window[i][j] = top_window[i][j] + line_buf[k + start_pix][11:0];
						bottom_window[i][j] = bottom_window[i][j] + line_buf[((KERNEL_SIZE - 1)*LINE_WIDTH) + k + start_pix][11:0];
						left_window[i][j] = left_window[i][j] + line_buf[k*LINE_WIDTH + start_pix][11:0];
						right_window[i][j] = right_window[i][j] + line_buf[k*LINE_WIDTH + (KERNEL_SIZE - 1) + start_pix][11:0];
					end
					y_grad_window[i][j] <= ((top_window[i][j] > bottom_window[i][j])?top_window[i][j] - bottom_window[i][j]:bottom_window[i][j] - top_window[i][j])/2;
					x_grad_window[i][j] <= ((left_window[i][j] > right_window[i][j])?left_window[i][j] - right_window[i][j]:right_window[i][j] - left_window[i][j])/2;
				end
				
			
			/*if(line_buffer[11215][12] == 1'b1) begin 
				sum = 0; 
				for(i = 0; i < 29; i=i+1) begin
					for(j = 0; j < 29; j=j+1) begin
						start_pix = i*800 + j; 
						sum = sum + line_buffer[start_pix][11:0];	
					end
				end 
				Sum <= sum; 
			end */
		end
		//kernel_out <= (SW[17])?y_grad_window[SW[1:0]][SW[3:2]]:x_grad_window[SW[5:4]][SW[7:6]];
	end
end

always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		y_2_sum = 0;
		x_2_sum = 0;
		x_y_sum = 0;
		for(i = 0; i < KERNEL_SIZE; i=i+1) begin
			for(j = 0; j < KERNEL_SIZE; j=j+1) begin
				y_2_sum = y_2_sum + (y_grad_window[i][j]>>7)**2;
				x_2_sum = x_2_sum + (x_grad_window[i][j]>>7)**2;
				x_y_sum = x_y_sum + ((x_grad_window[i][j]>>7) * (y_grad_window[i][j]>>7));
			end
		end
		
	end
end


//Clk 1
always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		x_2_times_y_2 <= x_2_sum * y_2_sum;
		x_y_squared <= x_y_sum * x_y_sum;
		x2_add_y2 <= x_2_sum + y_2_sum;
	end
end

//Clk 2
always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		det <= x_2_times_y_2 - x_y_squared;
		trace <= (x2_add_y2)**2;
	end
end

//Clk 3
always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		det_buff <= det;
		trace_k <= trace >> 4;
	end
end

//Clk 4
always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		harris_measure <= (det_buff > trace_k)? det_buff - trace_k : 0;
	end
end

always @(negedge D5M_PIXLCLK)begin 
	if(sCCD_DVAL) begin
		if((harris_measure > (SW<<8)) && (harris_measure < (25'b1000000000000000000000000))) begin
			//center of 5x5 
			//line_buf[21627][12] <= 1'b1; 
			corner <= 1'b1; 
		end 
		else begin 
			//line_buf[21627][12] <= 1'b0; 
			corner <= 1'b0; 
		end
	end
end

always @(posedge D5M_PIXLCLK)begin
	if(sCCD_DVAL) begin
		if((Y_Cont_true >= (N_LINES_IN_BUF + 5)) && (Y_Cont_true < (600 - (N_LINES_IN_BUF + 5))))begin 
			if((X_Cont_true >= (N_LINES_IN_BUF + 5)) && (X_Cont_true < (800 - (N_LINES_IN_BUF + 5))))begin 
				if(line_buf[11214][12] == 1'b1) begin
			
			
					sum = 0; 
					for(i = 0; i < 29; i=i+1) begin
						for(j = 0; j < 29; j=j+1) begin
							start_pix = i*800 + j; 
							sum = sum + line_buf[start_pix][11:0];	
						end
					end 
					
					avg = sum / 900; 
					w_count = 0; 
					b_count = 0; 
					for(i = 0; i < 29; i=i+1) begin
						for(j = 0; j < 29; j=j+1) begin
							start_pix = i*800 + j; 
							if( line_buf[start_pix][11:0] >= avg) begin	
								w_count = w_count + 1; 
							end
							else begin 
								b_count = b_count + 1; 
							end 
						end 
					end 
					
					
					//corner_frame_input <= ((harris_measure > (SW<<8)) && (harris_measure < (25'b1000000000000000000000000)))? 1'b1 : 1'b0;
					//12015
					Image_Address_In <= (Y_Cont_true)*(LINE_WIDTH) + (X_Cont_true) - 12020;
					corner_frame_input <= 1'b1; 
					if(b_count > w_count) begin 
						color_frame_input <= 1'b0; 
					end 
					else begin 
						color_frame_input <= 1'b1; 
					end 
					
				end
				else begin 
					//12015
					Image_Address_In <= (Y_Cont_true)*(LINE_WIDTH) + (X_Cont_true) - 12020;
					corner_frame_input <= 1'b0; 
					color_frame_input <= 1'b0;
				end
			end
			else begin 
				corner_frame_input <= 1'b0; 
				color_frame_input <= 1'b0;
				Image_Address_In <= (Y_Cont_true)*(LINE_WIDTH) + (X_Cont_true);
			end
		end 
		else begin 
			corner_frame_input <= 1'b0; 
			color_frame_input <= 1'b0;
			Image_Address_In <= (Y_Cont_true)*(LINE_WIDTH) + (X_Cont_true);
		end 
	end
end


//red = more white green = more black 
assign VGA_red = (corner_frame_output == 1 && Frame_Cont > 1)? ((color_frame_output == 1) ? 10'h3FF : 10'h000) : Read_DATA2[9:0];
assign VGA_green =(corner_frame_output == 1 && Frame_Cont > 1)? ((color_frame_output == 1) ? 10'h000 : 10'h3FF): {Read_DATA1[14:10],Read_DATA2[14:10]};
assign VGA_blue = (corner_frame_output == 1 && Frame_Cont > 1)? 10'h000: Read_DATA1[9:0];


assign Image_Address_Out = (V_Cont_true)*(LINE_WIDTH)+(H_Cont_true); 

//assign sCCD_R = (corner_frame_output) ? 12'b111111111111 : sCCD_R_raw; 
//assign sCCD_G = (corner_frame_output) ? 12'b000000000000 : sCCD_G_raw; 
//assign sCCD_B = (corner_frame_output) ? 12'b000000000000 : sCCD_B_raw; 


bob lin1(	.data(corner_frame_input),
			.rdaddress(Image_Address_Out),
			.rdclock(VGA_CTRL_CLK), 
			.wraddress(Image_Address_In),
			.wrclock(~D5M_PIXLCLK),
			.wren(1'b1),
			.q(corner_frame_output));
			
bob lin2(	.data(color_frame_input),
			.rdaddress(Image_Address_Out),
			.rdclock(VGA_CTRL_CLK), 
			.wraddress(Image_Address_In),
			.wrclock(~D5M_PIXLCLK),
			.wren(1'b1),
			.q(color_frame_output));
			
endmodule
