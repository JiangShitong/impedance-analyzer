`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/20 11:21:45
// Design Name: 
// Module Name: PSD
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


module PSD(
	//input								i_clk_250M					,
	//input								i_clk_50M						,
	input								i_clk_1M							,
	input								i_clk_1M_sync				,  
	input								i_rst_n                 			,
	input [3:0]						i_coe								,
	input [1:0]						i_mod								,
	input signed [19:0]		i_data_ad_A             		,
	input signed [19:0]		i_data_ad_B					,
	input								i_ad_data_done_A		,
	input								i_ad_data_done_B		,
	input signed [15:0]		i_Vref_sin						,
	input signed [15:0]		i_Vref_cos						,
	input								i_nco_psd_flag 				,
	output  [35:0]				o_A_X            					,
	output  [35:0]				o_A_Y            					,
	output  [35:0]				o_B_X            					,
	output  [35:0]				o_B_Y								,
	output							o_psd_aver_flag			,
	output reg						o_psd_valid
    );
	
/**************************************    CDC : from AD to PSD      *****************************************/
reg [19:0] data_ad_A_0;
reg [19:0] data_ad_B_0;
(*mark_debug = "true" *)reg [19:0] data_ad_A;
(*mark_debug = "true" *)reg [19:0] data_ad_B;

always@(posedge i_clk_1M or negedge i_rst_n)
	if(!i_rst_n)	begin
		data_ad_A_0	<= 20'd0;
		data_ad_B_0 <= 20'd0;	
		data_ad_A	<= 20'd0;
		data_ad_B <= 20'd0;
		end
	else	begin
		data_ad_A_0	<= i_data_ad_A;
		data_ad_B_0 <= i_data_ad_B;
		data_ad_A	<= data_ad_A_0;
		data_ad_B <=  data_ad_B_0;		
		end

/**************************************    CDC : from NCO to PSD      *****************************************/	
reg [15:0] Vref_sin_0;
reg [15:0] Vref_cos_0;
(*mark_debug = "true" *)(* dont_touch = "true" *)reg [15:0] Vref_sin;
(*mark_debug = "true" *)(* dont_touch = "true" *)reg [15:0] Vref_cos;

always@(posedge i_clk_1M or negedge i_rst_n)
	if(!i_rst_n)	begin
		Vref_sin_0	<= 16'd0;
		Vref_cos_0 <= 16'd0;	
		Vref_sin 	<= 16'd0;
		Vref_cos <= 16'd0;
		end
	else	begin
		Vref_sin_0 <= i_Vref_sin;
		Vref_cos_0 <= i_Vref_cos;
		Vref_sin 	<= Vref_sin_0;
		Vref_cos <= Vref_cos_0;	
		end
	
/************************************** PSD : Vref * AD_data         *****************************************/			
(*mark_debug = "true" *)(* dont_touch = "true" *)wire signed [35:0] psd_mul_A_X;		
(*mark_debug = "true" *)(* dont_touch = "true" *)wire signed [35:0] psd_mul_A_Y;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire signed [35:0] psd_mul_B_X;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire signed [35:0] psd_mul_B_Y;	

MUL_AD_PSD MUL_AD_PSD_A_X(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad_A					),
	.B	 	(Vref_sin				),
	.P		(psd_mul_A_X	)
);

MUL_AD_PSD MUL_AD_PSD_A_Y(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad_A					),
	.B	 	(Vref_cos			),
	.P		(psd_mul_A_Y	)
);

MUL_AD_PSD MUL_AD_PSD_B_X(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad_B					),
	.B	 	(Vref_sin				),
	.P		(psd_mul_B_X	)
);

MUL_AD_PSD MUL_AD_PSD_B_Y(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad_B					),
	.B	 	(Vref_cos			),
	.P		(psd_mul_B_Y	)
);	
	
/**************************************     LPF : IIR by SHIFT    *****************************************/		
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] psd_IIR_A_X;		
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] psd_IIR_A_Y;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] psd_IIR_B_X;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] psd_IIR_B_Y;
wire psd_IIR_valid;		

PSD_IIR_LPF psd_iir_lpf(
	.i_clk							(i_clk_1M),
    .i_rst_n						(i_rst_n),
    .i_coefficient				(i_coe),
    .i_mod						(i_mod),
    .i_A_X							(psd_mul_A_X),
    .i_A_Y							(psd_mul_A_Y),
    .i_B_X							(psd_mul_B_X),
    .i_B_Y							(psd_mul_B_Y),	
	.o_A_X						(psd_IIR_A_X),
	.o_A_Y						(psd_IIR_A_Y),
	.o_B_X						(psd_IIR_B_X),
	.o_B_Y						(psd_IIR_B_Y),	
	.o_IIR_valid				(psd_IIR_valid)
);	
		
/**************************************     OUTPUT for ARM     *****************************************/	
PSD_AVERAGE	psd_average_128_A(
	.i_clk		(i_clk_1M	),
	.i_rst_n	(i_rst_n	),
	.i_X			(psd_IIR_A_X	),
	.i_Y			(psd_IIR_A_Y	),
	.o_X		(o_A_X	),
	.o_Y		(o_A_Y	),
	.o_flag	(o_psd_aver_flag)	
);

PSD_AVERAGE	psd_average_128_B(
	.i_clk		(i_clk_1M	),
	.i_rst_n	(i_rst_n	),
	.i_X			(psd_IIR_B_X	),
	.i_Y			(psd_IIR_B_Y	),
	.o_X		(o_B_X	),
	.o_Y		(o_B_Y	),
	.o_flag	(		)		
);
	
endmodule
