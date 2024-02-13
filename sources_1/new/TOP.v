`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/09/24 17:24:41
// Design Name: 
// Module Name: TOP
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module TOP(
//ARM	
	inout [14:0]DDR_addr,
	inout [2:0]DDR_ba,
	inout DDR_cas_n,
	inout DDR_ck_n,
	inout DDR_ck_p,
	inout DDR_cke,
	inout DDR_cs_n,
	inout [3:0]DDR_dm,
	inout [31:0]DDR_dq,
	inout [3:0]DDR_dqs_n,
	inout [3:0]DDR_dqs_p,
	inout DDR_odt,
	inout DDR_ras_n,
	inout DDR_reset_n,
	inout DDR_we_n,
	inout FIXED_IO_ddr_vrn,
	inout FIXED_IO_ddr_vrp,
	inout [53:0]FIXED_IO_mio,
	inout FIXED_IO_ps_clk,
	inout FIXED_IO_ps_porb,
	inout FIXED_IO_ps_srstb,
	
	
//master clock (NOTICE:phase invert for the differential clock pin i_clk_9552)
	input i_clk_20M,
	input i_clk_9552_p,	
	input i_clk_9552_n,
	
//Sync clock from each DDS
	input i_DDS1_sync_clk,
	input i_DDS2_sync_clk,
	input i_DDS3_sync_clk,
	input i_DDS4_sync_clk,
	
//Clock chip
	inout AD9552_IOS_VE2,
	inout AD9552_CS_n,
	inout AD9508_IOS_VE2,
	inout AD9508_CS_n,
	inout AD9508_SYNC,
	inout AD9508_RST_n,
	inout AD9552_Locked,
	inout AD9552_RST,
	
//DDS2	
	inout DDS2_PWR_DWN,
	inout DDS2_RESET,
	inout DDS2_IO_SYNC,
	inout DDS2_CS_n,
	inout DDS2_SYNC_IN,
	inout DDS2_IO_UPDATE,
	inout DDS2_IOS_VE2,	

//DDS3		
	inout DDS3_PWR_DWN,
	inout DDS3_RESET,
	inout DDS3_IO_SYNC,
	inout DDS3_CS_n,
	inout DDS3_SYNC_IN,
	inout DDS3_IO_UPDATE,
	inout DDS3_IOS_VE2,	

//DDS1		
	inout DDS1_PWR_DWN,
	inout DDS1_RESET,
	inout DDS1_IO_SYNC,
	inout DDS1_CS_n,
	inout DDS1_SYNC_IN,
	inout DDS1_IO_UPDATE,
	inout DDS1_IOS_VE2,	

//SPARE IO

	inout SPARE_1,
	inout SPARE_2,
	inout SPARE_3,
	inout SPARE_4,
	inout SPARE_5,
    inout SPARE_6,
    inout SPARE_7,
    inout SPARE_8,
 	inout SPARE_9,
    inout SPARE_10,
    inout SPARE_11,
    inout SPARE_12,
    inout SPARE_13,

	
//SPI0
	input MISO_0,
	output MOSI_0,
	// output SS,
    output SCLK_0,
    
 //SPI1
    input MISO_1,
    output MOSI_1,
    // output SS,
    output SCLK_1,
	
//Test IO
 	input i_rst_n,	//map to the BTN0 on the schematic 
	output LED,
	
//HC595_AMP
    output         o_SRCLR_n_AMP             	,
    output         o_RCLK_AMP                	,
    output         o_SER_AMP                 	,
    output         o_SRCLK_AMP               	,
	
//HC595_ATT   Vx_ATT
    output         o_SRCLR_n_ATT_A            	,
    output         o_RCLK_ATT_A               	,
    output         o_SER_ATT_A                	,
    output         o_SRCLK_ATT_A 				,   
//HC595_ATT   Vmod_ATT	
    output         o_SRCLR_n_ATT_B            	,
    output         o_RCLK_ATT_B              	,
    output         o_SER_ATT_B                	,
    output         o_SRCLK_ATT_B 				,

//HC595_ANALOG	
    output         o_OE_n_ANALOG		,
    output         o_SRCLR_n_ANALOG,	
    output         o_RCLK_ANALOG			,
    output         o_SER_ANALOG			,
    output         o_SRCLK_ANALOG		,

	
//LTC2378
    input      i_LTC2378_SDO_MUL         	,
    input      i_LTC2378_BUSY_MUL        	,
    output     o_LTC2378_CNV_MUL         	,
    output     o_LTC2378_SCK_MUL            , 		
    input      i_LTC2378_SDO_LP         	,
    input      i_LTC2378_BUSY_LP        	,
    output     o_LTC2378_CNV_LP         	,
    output     o_LTC2378_SCK_LP             ,
    
    
    //power board
    inout   POWER_CUREN,
    inout   POWER_Ctrl1,
    inout   POWER_CS_n,
    inout   POWER_IOS_VE2,
    inout   POWER_SDA,
    inout   POWER_SCL

 );
 
wire  [47:0]	GPIO_Out;
wire  [47:0]	GPIO_In;

wire clk_400M_dif;
wire clk_50M_single;
wire DDS1_sync_clk;
wire DDS2_sync_clk;
wire DDS3_sync_clk;

/*****	output assignment	*****/

//Clock Path related
	assign AD9552_IOS_VE2 = GPIO_Out[7];
	assign AD9552_CS_n	  = GPIO_Out[6];
	assign AD9508_IOS_VE2 = GPIO_Out[5];
	assign AD9508_CS_n	  = GPIO_Out[4];
	assign AD9508_SYNC	  = GPIO_Out[3];
	assign AD9508_RST_n	  = GPIO_Out[2];
	assign AD9552_RST	  = GPIO_Out[0];

//DDS2 related
//	assign DDS2_PWR_DWN   = GPIO_Out[14];
	assign DDS2_RESET 	  = GPIO_Out[13];
	assign DDS2_IO_SYNC   = GPIO_Out[12];
	assign DDS2_CS_n 	  = GPIO_Out[11];
	assign DDS2_SYNC_IN   = GPIO_Out[10];
//	assign DDS2_IO_UPDATE = GPIO_Out[9];
	assign DDS2_IOS_VE2   = GPIO_Out[8];	

//DDS3 related
//	assign DDS3_PWR_DWN   = GPIO_Out[21];
	assign DDS3_RESET 	  = GPIO_Out[20];
	assign DDS3_IO_SYNC   = GPIO_Out[19];
	assign DDS3_CS_n 	  = GPIO_Out[18];
	assign DDS3_SYNC_IN   = GPIO_Out[17];
//	assign DDS3_IO_UPDATE = GPIO_Out[16];
	assign DDS3_IOS_VE2   = GPIO_Out[15];		

//DDS1 related
//	assign DDS1_PWR_DWN   = GPIO_Out[28];
	assign DDS1_RESET 	  = GPIO_Out[27];
	assign DDS1_IO_SYNC   = GPIO_Out[26];
	assign DDS1_CS_n 	  = GPIO_Out[25];
	assign DDS1_SYNC_IN   = GPIO_Out[24];
//	assign DDS1_IO_UPDATE = GPIO_Out[23];
	assign DDS1_IOS_VE2   = GPIO_Out[22];	
	
//SPARE IO related    
    assign SPARE_13 = GPIO_Out[47];
	assign SPARE_12 = GPIO_Out[46];
    assign SPARE_11 = GPIO_Out[45];
    assign SPARE_10 = GPIO_Out[44];
    assign SPARE_9 = GPIO_Out[43];
	assign SPARE_8 = GPIO_Out[42];
    assign SPARE_7 = GPIO_Out[41];
    assign SPARE_6 = GPIO_Out[40];
    assign SPARE_5 = GPIO_Out[39];
	assign SPARE_4 = GPIO_Out[38];
    assign SPARE_3 = GPIO_Out[37];
    assign SPARE_2 = GPIO_Out[36]; 
    assign SPARE_1 = GPIO_Out[35];             	

//POWER BOARD
    assign POWER_CUREN = GPIO_Out[34];
	assign POWER_Ctrl1 = GPIO_Out[33];
    assign POWER_CS_n = GPIO_Out[32];   
    assign POWER_IOS_VE2 = GPIO_Out[31];  
    assign POWER_SDA = GPIO_Out[30];  
    assign POWER_SCL = GPIO_Out[29];  

/*****	input assignment	*****/

//clock path related
	assign GPIO_In[1] = AD9552_Locked;	
	
/*****	clock management	*****/
wire clk_20M;
wire clk_250M;
wire clk_50M;
wire clk_100M;
wire clk_80M;
wire clk_90M;
wire locked_pll;
(*mark_debug = "true" *)(*keep = "true"*) wire rst_n;
wire i_clk_20M_ibuf;

(*mark_debug = "true" *)(*keep = "true"*) wire rst_add_PS_n;
(*mark_debug = "true" *)(*keep = "true"*) reg PS_reset_PL_n;	//PS to reset the PL
assign rst_n = i_rst_n & locked_pll;
assign rst_add_PS_n = i_rst_n & locked_pll & PS_reset_PL_n;

/*****	For ps_o_PL_reset_nt signal synchronous	*****/ 	
wire [31:0] ps_o_PL_reset_n;
reg PS_reset_PL_n_0;
reg temp_dds_pwr_down;
	





always@(posedge clk_250M or negedge rst_n)
	if(!rst_n)	begin
			PS_reset_PL_n_0 <= 1'd0;
			PS_reset_PL_n   <= 1'd0;
			end
	else	begin
			PS_reset_PL_n_0 <= ps_o_PL_reset_n[0];
			PS_reset_PL_n   <= PS_reset_PL_n_0;
			end		









IBUF IBUF_clk_20M(.O(i_clk_20M_ibuf),.I(i_clk_20M));
BUFG BUFG_clk_20M ( .O(clk_20M), .I(i_clk_20M_ibuf)  );

  PLL1 PLL1
   (
      // Clock in ports
    .clk_in1(clk_20M),   // input clk_in1
    // Clock out ports
    .clk_out1_50M(clk_50M),     // output clk_out1_50M
    .clk_out2_250M(clk_250M),     // output clk_out2_250M
    .clk_out3_90M(clk_90M),     // output clk_out2_250M
    // Status and control signals
    .resetn(i_rst_n), // input resetn
    .locked(locked_pll)       // output locked
    );   

wire clk_1M;
wire clk_1M_buf;//division clk
wire clk_1M_sync;
wire clk_1M_sync_buf;

Clk_Divide_50 clk_divide_1M(
    .i_clk              	(clk_50M),
    .i_rst_n          	(rst_n),
    .o_clk              (clk_1M), 
    .o_sync			(clk_1M_sync)
);

BUFG U3(.O(clk_1M_buf),.I(clk_1M));	
BUFG U4(.O(clk_1M_sync_buf),.I(clk_1M_sync));	


wire o_clk_9552;
wire gclk_9552;

IBUFDS 	IBUFDS_clk_9552	(.O(o_clk_9552), .I(i_clk_9552_p), .IB(i_clk_9552_n));
BUFG	BUFG_clk_9552	(.O(gclk_9552), .I(o_clk_9552));

 
BUFG BUFG_DDS1_sync ( .O(DDS1_sync_clk), .I(i_DDS1_sync_clk)  );
BUFG BUFG_DDS2_sync ( .O(DDS2_sync_clk), .I(i_DDS2_sync_clk)  );
BUFG BUFG_DDS3_sync ( .O(DDS3_sync_clk), .I(i_DDS3_sync_clk)  );

/******	sync all IO Update of 3 DDS to the DDS1 sync clock	*****/
IO_update IO_update_DDS1(
	.clk(DDS1_sync_clk),
	//.clk(gclk_9552),
	.rstn(i_rst_n),	
    // .ps_dds_update(GPIO_Out[23]),
	.ps_dds_update(GPIO_Out[9]),    	//all the DDS use the same IO UPDATE [from DDS2]
	.o_dds_update(DDS1_IO_UPDATE)
);

IO_update IO_update_DDS2(
	 .clk(DDS2_sync_clk),
	//.clk(gclk_9552),
	.rstn(i_rst_n),	
    .ps_dds_update(GPIO_Out[9]),    	//all the DDS use the same IO UPDATE [from DDS2]
	.o_dds_update(DDS2_IO_UPDATE)
);

IO_update IO_update_DDS3(
	.clk(DDS3_sync_clk),
	//.clk(gclk_9552),
	.rstn(i_rst_n),	
	// .ps_dds_update(GPIO_Out[16]),
    .ps_dds_update(GPIO_Out[9]),    	//all the DDS use the same IO UPDATE [from DDS2]
	.o_dds_update(DDS3_IO_UPDATE)
);




			
			
			
DDS_PWR_all DDS_PWR_DDS_all_inst(	
	.clk(gclk_9552),
	.rstn(i_rst_n),
    .ps_dds_pwr_down(ps_o_PL_reset_n[1]),
	.o_dds1_pwr_down(DDS1_PWR_DWN),
	.o_dds2_pwr_down(DDS2_PWR_DWN),
	.o_dds3_pwr_down(DDS3_PWR_DWN)
);


//Program check
assign LED = 1'b1;

//////////////////////////////////////// 				HC595						////////////////////

wire [3:0] ATT_A;	
wire [3:0] ATT_B;
wire [3:0] AMP_A;	
wire [3:0] AMP_B;	
wire [31:0] CONTROL;//include AMP/ATTEN/LPF COE/LPF SLOPE
	
	assign AMP_A = CONTROL[3:0];
	assign AMP_B = CONTROL[7:4];
	assign ATT_A = CONTROL[11:8];
	assign ATT_B = CONTROL[15:12];
	
HC595_CTRL_ATT_A HC595_CTRL_ATT_A(
	.i_clk       			(clk_1M_buf),
	.i_rst_n     			(rst_n),
	.i_ATT_A      		(ATT_A),
//	.o_OE_n     		(o_OE_n_ATT_A),//The OE pin connected to GND in the version 3.0
	.o_SRCLR_n   		(o_SRCLR_n_ATT_A),
	.o_RCLK      		(o_RCLK_ATT_A),
	.o_SER       			(o_SER_ATT_A),
	.o_SRCLK     		(o_SRCLK_ATT_A)
);

HC595_CTRL_ATT_B HC595_CTRL_ATT_B(
	.i_clk       			(clk_1M_buf),
	.i_rst_n     			(rst_n),
	.i_ATT_B 			(ATT_B), 
//	.o_OE_n     		(o_OE_n_ATT_B),//The OE pin connected to GND in the version 3.0
	.o_SRCLR_n   		(o_SRCLR_n_ATT_B),
	.o_RCLK      		(o_RCLK_ATT_B),
	.o_SER       			(o_SER_ATT_B),
	.o_SRCLK     		(o_SRCLK_ATT_B)
);	

	
HC595_CTRL_AMP HC595_CTRL_AMP(
	.i_clk       		(clk_1M_buf),
	.i_rst_n     		(rst_n),
	.i_AMP_A      		(AMP_A),
	.i_AMP_B 			(AMP_B), 
   //.o_OE_n     		(o_OE_n_AMP),//The OE pin connected to GND in the version 3.0
	.o_SRCLR_n   		(o_SRCLR_n_AMP),
	.o_RCLK      		(o_RCLK_AMP),
	.o_SER       		(o_SER_AMP),
	.o_SRCLK     		(o_SRCLK_AMP)
);	

wire  [1:0]    CTRL_Vr_Sel;	
wire  [2:0] 	CTRL_Rr_Sel;
wire  [3:0]	    CTRL_Measure_Sel;
wire  [1:0] 	CTRL_freq_switch;
wire			CTRL_Bias_ON;	
wire           CTRL_Vx_Vr_AMP;
(*mark_debug = "true" *)(*keep = "true"*)wire [12:0] CTRL_ANALOG;
assign CTRL_Vr_Sel				= CTRL_ANALOG[1:0]	;		
assign CTRL_Rr_Sel				= CTRL_ANALOG[4:2]	;
assign CTRL_Measure_Sel	        = CTRL_ANALOG[8:5]	;
assign CTRL_freq_switch		    = CTRL_ANALOG[10:9]	;
assign CTRL_Bias_ON			    = CTRL_ANALOG[11]	;
assign CTRL_Vx_Vr_AMP           = CTRL_ANALOG[12]	;

HC595_CTRL_ANALOG HC595_CTRL_ANALOG(
	.i_clk       					(clk_1M_buf			),
	.i_rst_n     				    (rst_n			    ),
	.i_CTRL_Vr_Sel				    (CTRL_Vr_Sel		),	
	.i_CTRL_Rr_Sel				    (CTRL_Rr_Sel		),
	.i_CTRL_Measure_Sel     	    (CTRL_Measure_Sel	),
	.i_CTRL_Bias_ON			        (CTRL_Bias_ON		),	
	.i_CTRL_Vx_Vr_AMP	            (CTRL_Vx_Vr_AMP     ),
	.o_OE_n      					(o_OE_n_ANALOG		),
	.o_SRCLR_n   					(o_SRCLR_n_ANALOG	),
	.o_RCLK      					(o_RCLK_ANALOG	    ),
	.o_SER       					(o_SER_ANALOG		),
	.o_SRCLK  						(o_SRCLK_ANALOG		)
);	 

 /********************************			LTC2378		***********************************************/	
wire LTC2378_SYN_MUL;
(*mark_debug = "true" *)(*keep = "true"*)  wire [19:0] LTC2378_DATA_MUL;
wire LTC2378_OVD_MUL;
(*mark_debug = "true" *)(*keep = "true"*)  wire AD_flag;

LTC2378 LTC2378_MUL(
    .i_rst_n 				(rst_n           ), 
    .i_SDO 					(i_LTC2378_SDO_MUL ), 
    .i_clk   				(clk_90M         ), 
    .i_BUSY 				(i_LTC2378_BUSY_MUL),
    .o_CNV 					(o_LTC2378_CNV_MUL ),
    .o_SCK 					(o_LTC2378_SCK_MUL ),
    .o_AD_data_syn		    (LTC2378_SYN_MUL   ),
    .o_AD_data       		(LTC2378_DATA_MUL  ),
    .o_AD_Overload		    (LTC2378_OVD_MUL ),
	.o_flag					(AD_flag)
);

wire LTC2378_SYN_LP;
(*mark_debug = "true" *)(*keep = "true"*) wire [19:0] LTC2378_DATA_LP;
wire LTC2378_OVD_LP;

LTC2378 LTC2378_LP(
    .i_rst_n 				(rst_n           ), 
    .i_SDO 				    (i_LTC2378_SDO_LP ), 
    .i_clk   				(clk_90M         ), 
    .i_BUSY 				(i_LTC2378_BUSY_LP),
    .o_CNV 				    (o_LTC2378_CNV_LP ),
    .o_SCK 				    (o_LTC2378_SCK_LP ),
    .o_AD_data_syn	        (LTC2378_SYN_LP   ),
    .o_AD_data       	    (LTC2378_DATA_LP  ),
    .o_AD_Overload	        (LTC2378_OVD_LP ),
	.o_flag					()
);		

//////////////////////////////////////// 		NCO			//////////////
wire [31:0] Vx_Vr_phase_inc;		
wire [31:0] Vx_phase_mod;				// 	DATA = phase/(2^32)*360	;
wire [31:0] Vr_phase_mod;				// 	DATA = phase/(2^32)*360		;	
wire [31:0] Vmod_phase_inc_1k8	;
wire [31:0] Vmod_phase_mod_1k8;
(*mark_debug = "true" *)(*keep = "true"*)wire [31:0] Vref_phase_inc;
(*mark_debug = "true" *)(*keep = "true"*)wire [31:0] Vref_phase_mod;
(*mark_debug = "true" *)(*keep = "true"*)wire [15:0] Vx_data;
(*mark_debug = "true" *)(*keep = "true"*)wire [15:0] Vr_data;
(*mark_debug = "true" *)(*keep = "true"*)wire [15:0] Vmod_data_1k8;
(*mark_debug = "true" *)(*keep = "true"*)wire [15:0] Vref_sin;
(*mark_debug = "true" *)(*keep = "true"*)wire [15:0] Vref_cos;
wire nco_psd_flag;


 NCO nco(
	.i_clk_250M						(clk_250M),
	.i_rst_n                 		(rst_add_PS_n								),		//Tony alter : rst_n
	.i_Vx_Vr_phase_inc				(Vx_Vr_phase_inc	),		//DDS Frequency Control, Set by USER
	.i_Vx_phase_mod  				(Vx_phase_mod 	),		//DDS Phase Control, Set by USER
	.i_Vr_phase_mod					(Vr_phase_mod	), 		//DDS Phase Control, Set by USER
	.i_Vmod_phase_inc_1k8		    (Vmod_phase_inc_1k8	),		//DDS Frequency Control, Set by Algorithm	
	.i_Vmod_phase_mod_1k8	        (Vmod_phase_mod_1k8	), 
	.i_Vref_phase_inc				(Vref_phase_inc	),		//DDS Frequency Control, Set by Algorithm	
	.i_Vref_phase_mod				(Vref_phase_mod	), 		//Tony alter : Vref_phase_mod
	.o_nco_psd_flag					(nco_psd_flag				),
	.o_Vx_sin						(Vx_data							),
	.o_Vmod_sin					    (Vmod_data_1k8					),
	.o_Vr_sin						(Vr_data							)		,
	.o_Vref_sin					    (Vref_sin),
	.o_Vref_cos					    (Vref_cos)		
    );


	
//////////////////////////////////////// 		PSD			//////////////
(*mark_debug = "true" *)(*keep = "true"*)wire [35:0]	Measure_A_X;
(*mark_debug = "true" *)(*keep = "true"*)wire [35:0]	Measure_A_Y;
(*mark_debug = "true" *)(*keep = "true"*)wire [35:0]	Measure_B_X;
(*mark_debug = "true" *)(*keep = "true"*)wire [35:0]	Measure_B_Y;
(*mark_debug = "true" *)(*keep = "true"*)wire psd_aver_flag;

(*mark_debug = "true" *)(*keep = "true"*)wire [3:0]	coefficient;
wire [1:0]	mod;

assign coefficient = CONTROL[19:16];
assign mod = CONTROL[21:20];

PSD psd(
	//.i_clk_250M					(clk_250M		 ),
	//.i_clk_50M					(clk_50M		 ),
	.i_clk_1M						(clk_1M_buf	     ),
	.i_clk_1M_sync					(	 ),  
	.i_rst_n                 		(rst_n			 ),	
	.i_coe							(coefficient     ),
	.i_mod							(2'd3            ),
	.i_data_ad_A             		(LTC2378_DATA_MUL),		//Tony alter : LTC2378_DATA_A
	.i_data_ad_B					(LTC2378_DATA_LP ),		//Tony alter : LTC2378_DATA_B
	.i_ad_data_done_A			    (AD_flag         ),
	.i_ad_data_done_B			    (AD_flag         ),	
	.i_Vref_sin						(Vref_sin		 ),
	.i_Vref_cos						(Vref_cos		 ),
	.i_nco_psd_flag 				(nco_psd_flag    ),
	.o_A_X            				(Measure_A_X	 ),
	.o_A_Y            				(Measure_A_Y	 ),
	.o_B_X            				(Measure_B_X	 ),
	.o_B_Y							(Measure_B_Y	 ),
	.o_psd_aver_flag				(psd_aver_flag	 ),
	.o_psd_valid					(			     )
);
	
//////////////////////////////////////// 		DFT			//////////////
wire [27:0] point_of_period;
(*mark_debug = "true" *)wire [63:0]	DFT_A_X;
(*mark_debug = "true" *)wire [63:0]	DFT_A_Y;
(*mark_debug = "true" *)wire [63:0]	DFT_B_X;
(*mark_debug = "true" *)wire [63:0]	DFT_B_Y;
 
/***********	Tony add for DFT module test	*****/
reg [15:0] Vx_data_temp;
reg [15:0] Vr_data_temp;

always @ (posedge clk_1M_buf or negedge rst_n)
	if(!rst_n)
	begin
		Vx_data_temp <= 16'd0;
		Vr_data_temp <= 16'd0;
	end
	else
	begin
		Vx_data_temp <= Vx_data;
		Vr_data_temp <= Vr_data;
	end
/***********	Tony add for DFT module test	*****/

DFT dft_A(
	.i_clk_1M					(clk_1M_buf		),
	.i_rst_n					(rst_n		),
	.i_data_ad					(LTC2378_DATA_MUL),		//Tony alter : LTC2378_DATA_A	{4'b0000,Vx_data_temp}	
	.i_Vref_sin					(Vref_sin		),
	.i_Vref_cos					(Vref_cos		),
	.i_point_of_period	        (point_of_period		),
	.o_X						(DFT_A_X		),
	.o_Y						(DFT_A_Y		)
);

DFT dft_B(
	.i_clk_1M					(clk_1M_buf		),
	.i_rst_n					(rst_n		),
	.i_data_ad					(LTC2378_DATA_LP),		//Tony alter : LTC2378_DATA_B	{4'b0000,Vr_data_temp}	
	.i_Vref_sin					(Vref_sin		),
	.i_Vref_cos					(Vref_cos		),
	.i_point_of_period	        (point_of_period		),
	.o_X						(DFT_B_X		),
	.o_Y						(DFT_B_Y		)
);

////////////////////////////////////////////		ARM Interface								//////////////
wire [31:0] ps_i_Measure_A_X_h;
wire [31:0] ps_i_Measure_A_Y_h;
wire [31:0] ps_i_Measure_B_X_h;
wire [31:0] ps_i_Measure_B_Y_h;
wire [31:0] ps_i_Measure_A_B_X_Y_l;
wire [31:0] ps_i_psd_aver_flag;
wire [31:0] ps_i_DFT_A_X_h;
wire [31:0] ps_i_DFT_B_X_h;
wire [31:0] ps_i_DFT_A_Y_h;
wire [31:0] ps_i_DFT_B_Y_h;
wire [31:0] ps_i_DFT_A_X_l;
wire [31:0] ps_i_DFT_B_X_l;
wire [31:0] ps_i_DFT_A_Y_l;
wire [31:0] ps_i_DFT_B_Y_l;

wire clk_ARM;
CDC_PL2PS CDC_PL2PS_ARM(
	.i_PS_clk	(clk_ARM	),
	.i_rst_n		(rst_n	),
    .i_data_0	(Measure_A_X[35:4]	),				
    .i_data_1	(Measure_A_Y[35:4]	),				
    .i_data_2	(Measure_B_X[35:4]	),				
    .i_data_3	(Measure_B_Y[35:4]	),				
    .i_data_4	({Measure_B_Y[3:0], Measure_B_X[3:0], Measure_A_Y[3:0], Measure_A_X[3:0]}	),		
	.i_data_5	(psd_aver_flag	),
    .i_data_6	(DFT_A_X[63:32]	 		),				
    .i_data_7	(DFT_A_Y[63:32]			),				
    .i_data_8	(DFT_B_X[63:32]			),				
    .i_data_9	(DFT_B_Y[63:32	]			),
    .i_data_10  (DFT_A_X[31:0]			),				
    .i_data_11  (DFT_A_Y[31:0]			),				
    .i_data_12  (DFT_B_X[31:0]			),				
    .i_data_13  (DFT_B_Y[31:0]			),				
    .o_data_0	(ps_i_Measure_A_X_h	),				
    .o_data_1	(ps_i_Measure_A_Y_h	),				
    .o_data_2	(ps_i_Measure_B_X_h	),				
    .o_data_3	(ps_i_Measure_B_Y_h	),				
    .o_data_4	(ps_i_Measure_A_B_X_Y_l	),
	.o_data_5	(ps_i_psd_aver_flag	),				
    .o_data_6	(ps_i_DFT_A_X_h	),				
    .o_data_7	(ps_i_DFT_A_Y_h	),				
    .o_data_8	(ps_i_DFT_B_X_h	),				
    .o_data_9	(ps_i_DFT_B_Y_h	),    
    .o_data_10(ps_i_DFT_A_X_l	),				
    .o_data_11(ps_i_DFT_A_Y_l	),				
    .o_data_12(ps_i_DFT_B_X_l	),				
    .o_data_13(ps_i_DFT_B_Y_l	)   	
);


wire [31:0] ps_o_Vx_Vr_phase_inc				;
wire [31:0] ps_o_Vx_phase_mod					;
wire [31:0] ps_o_Vr_phase_mod					;
wire [31:0] ps_o_Vx_gain									;
wire [31:0] ps_o_Vr_gain									;
wire [31:0] ps_o_Vmod_phase_inc_1k8		;
wire [31:0] ps_o_Vmod_phase_mod_1k8	;
wire [31:0] ps_o_Vmod_gain_1k8				;
wire [31:0] ps_o_Vref_phase_inc				    ;
wire [31:0] ps_o_Vref_phase_mod				;


wire [15:0] Vx_gain					;
wire [15:0] Vr_gain					;		
wire [15:0] Vmod_gain_1k8;	


CDC_PS2PL CDC_PS2PL_250M(
	.i_PL_clk		(clk_250M	),
	.i_rst_n		(rst_n	),
    .i_data_0	(ps_o_Vx_Vr_phase_inc												),
    .i_data_1	(ps_o_Vx_phase_mod													),
    .i_data_2	(ps_o_Vr_phase_mod													),
    .i_data_3	(ps_o_Vx_gain																),
    .i_data_4	(ps_o_Vr_gain																),
    .i_data_5	(ps_o_Vmod_phase_inc_1k8									),
    .i_data_6	(ps_o_Vmod_phase_mod_1k8	    							),
    .i_data_7	(ps_o_Vmod_gain_1k8				    							),
    .i_data_8	(ps_o_Vref_phase_inc				        							),
    .i_data_9	(ps_o_Vref_phase_mod				    							),	
    .o_data_0	(Vx_Vr_phase_inc						),
    .o_data_1	(Vx_phase_mod						),
    .o_data_2	(Vr_phase_mod						),
    .o_data_3	(Vx_gain										),
    .o_data_4	(Vr_gain										),
    .o_data_5	(Vmod_phase_inc_1k8			),
    .o_data_6	(Vmod_phase_mod_1k8	        ),
    .o_data_7	(Vmod_gain_1k8				        ),
    .o_data_8	(Vref_phase_inc				        ),
    .o_data_9	(Vref_phase_mod				        )
);

wire [31:0] ps_o_CONTROL				;
wire [31:0] ps_o_CTRL_ANALOG		;
wire [31:0] ps_o_point_of_period	;
					   
CDC_PS2PL CDC_PS2PL_1M(
	.i_PL_clk		(clk_1M_buf	),
	.i_rst_n		(i_rst_n	),
    .i_data_0	    (ps_o_CONTROL										),
    .i_data_1	    (ps_o_CTRL_ANALOG								),
	.i_data_2	    (ps_o_point_of_period							),
	.i_data_3	    (							),
    .o_data_0	    (CONTROL										),
    .o_data_1	    (CTRL_ANALOG								),
	//.o_data_2	    (point_of_period								),
	.o_data_2	    (								),
	.o_data_3	    (								)
);

//Processing System
Top_Block_wrapper ARM_Interface (
    .ARM_clk			(clk_ARM			),
	.DDR_addr			(DDR_addr		    ),
    .DDR_ba				(DDR_ba				),
    .DDR_cas_n			(DDR_cas_n			),
    .DDR_ck_n			(DDR_ck_n			),
    .DDR_ck_p			(DDR_ck_p			),
    .DDR_cke			(DDR_cke			),
    .DDR_cs_n			(DDR_cs_n			),
    .DDR_dm				(DDR_dm				),
    .DDR_dq				(DDR_dq				),
    .DDR_dqs_n			(DDR_dqs_n			),
    .DDR_dqs_p			(DDR_dqs_p			),
    .DDR_odt			(DDR_odt			),		
    .DDR_ras_n			(DDR_ras_n			),
    .DDR_reset_n		(DDR_reset_n		),
    .DDR_we_n			(DDR_we_n			),
    .FIXED_IO_ddr_vrn	(FIXED_IO_ddr_vrn	),
    .FIXED_IO_ddr_vrp	(FIXED_IO_ddr_vrp	),
    .FIXED_IO_mio		(FIXED_IO_mio		),
    .FIXED_IO_ps_clk	(FIXED_IO_ps_clk	),
    .FIXED_IO_ps_porb	(FIXED_IO_ps_porb	),
    .FIXED_IO_ps_srstb	(FIXED_IO_ps_srstb	),
    .GPIO_I(GPIO_In),
	.GPIO_O(GPIO_Out),
    .SPI0_MISO_I(MISO_0),
    .SPI0_MOSI_O(MOSI_0),
    // .SPI0_SS_O(SS),
    .SPI0_SCLK_O(SCLK_0),
    .SPI1_MISO_I(MISO_1),
    .SPI1_MOSI_O(MOSI_1),
    // .SPI0_SS_O(SS),
    .SPI1_SCLK_O(SCLK_1),    
	.i_reg0_0				(ps_i_Measure_A_X_h	),
    .i_reg1_0				(ps_i_Measure_A_Y_h	),
    .i_reg2_0				(ps_i_Measure_B_X_h	),
    .i_reg3_0				(ps_i_Measure_B_Y_h	),
    .i_reg4_0				(ps_i_Measure_A_B_X_Y_l	),
    .i_reg5_0				(ps_i_psd_aver_flag	),
    .i_reg6_0				(LTC2378_DATA_MUL	),
    .i_reg7_0				(Measure_A_X[35:4]	),
    .i_reg8_0				(Measure_A_X[3:0]	),
    .i_reg9_0				(Measure_A_Y[35:4]	),
    .i_reg10_0			    (Measure_A_Y[3:0]	),
    .i_reg11_0			    (Vref_sin	),
    .i_reg12_0			    (Vref_cos	),
    .i_reg13_0			    (ps_i_DFT_B_Y_l	),
    .i_reg14_0			    (	),	
    .o_reg15_0			    (ps_o_Vx_Vr_phase_inc					),			
    .o_reg16_0			    (ps_o_Vx_phase_mod						),
    .o_reg17_0			    (ps_o_Vr_phase_mod						),
    .o_reg18_0			    (ps_o_Vx_gain									),
    .o_reg19_0			    (ps_o_Vr_gain									),
    .o_reg20_0			    (ps_o_Vmod_phase_inc_1k8		),
    .o_reg21_0			    (ps_o_Vmod_phase_mod_1k8		),
    .o_reg22_0			    (ps_o_Vmod_gain_1k8					),
    .o_reg23_0			    (ps_o_Vref_phase_inc				    	),
    .o_reg24_0			    (ps_o_Vref_phase_mod					),
    .o_reg25_0			    (ps_o_CONTROL			),
    .o_reg26_0			    (ps_o_CTRL_ANALOG	),
    .o_reg27_0			    (ps_o_point_of_period),
    .o_reg28_0			    (ps_o_PL_reset_n),
    .o_reg29_0			    ()    
		
	
);

endmodule
