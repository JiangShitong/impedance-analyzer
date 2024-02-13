`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2019/01/23 19:06:15
// Design Name: 
// Module Name: DFT
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


module DFT(
	input	i_clk_1M,
	input i_rst_n,
	input signed [19:0] i_data_ad,
	input signed [15:0] i_Vref_sin,
	input signed [15:0] i_Vref_cos,
	input [27:0] i_point_of_period,
	output reg signed [63:0] o_X,
	output reg signed [63:0] o_Y
    );
	
	reg signed [15:0] Vref_sin_0;
	reg signed [15:0] Vref_sin;
	reg signed [15:0] Vref_cos_0;
	reg signed [15:0] Vref_cos;	

/**************************************    CDC : from NCO to PSD      *****************************************/		
	always@(posedge i_clk_1M or negedge i_rst_n)
		if(!i_rst_n)	begin
			Vref_sin_0	<= 16'd0;
            Vref_sin		<= 16'd0;
            Vref_cos_0	<= 16'd0;
            Vref_cos		<= 16'd0;
			end
		else	begin
			Vref_sin_0	<= i_Vref_sin;
            Vref_cos_0	<= i_Vref_cos;
            Vref_sin		<= Vref_sin_0;			
            Vref_cos		<= Vref_cos_0;
			end		
			
/**************************************    CDC : from AD to PSD      *****************************************/			
	reg signed [19:0] data_ad_0;
	reg signed [19:0] data_ad;	

	always@(posedge i_clk_1M or negedge i_rst_n)
		if(!i_rst_n)	begin	
			data_ad_0	<= 20'd0;
	        data_ad		<= 20'd0;
			end
		else	begin
	        data_ad_0	<= i_data_ad;
			data_ad		<= data_ad_0;
			end
	
	reg rst_flag;
	reg [27:0] point_of_period;

/**************************************          *****************************************/		
	always@(posedge i_clk_1M or negedge i_rst_n)
		if(!i_rst_n)	begin
			rst_flag <= 1'b0;
			point_of_period <= 28'd0;
			end
		else	begin
			point_of_period <= i_point_of_period;
			if(point_of_period != i_point_of_period)
				rst_flag <= 1'b1;
			else
				rst_flag <= 1'b0;
			end
			
	wire signed [35:0] dft_mul_X;
	wire signed [35:0] dft_mul_Y;
	
MUL_DFT MUL_DFT_X(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad					),
	.B	 	(Vref_sin				),
	.P		(dft_mul_X	)
);

MUL_DFT MUL_DFT_Y(
	.CLK	(i_clk_1M					),
	.SCLR	(!i_rst_n					),
	.A	 	(data_ad					),
	.B	 	(Vref_cos			),
	.P		(dft_mul_Y	)
);
	
	reg signed [35:0] dft_mul_X_buf;
	reg signed [35:0] dft_mul_Y_buf;
	
	always@(posedge i_clk_1M or negedge i_rst_n)
		if(!i_rst_n)	begin	
			dft_mul_X_buf	<= 36'd0;
	        dft_mul_Y_buf	<= 36'd0;
			end
		else	begin
	        dft_mul_X_buf	<= dft_mul_X;
			dft_mul_Y_buf	<= dft_mul_Y;
			end	
	
/**********	cnt for accunmulation;		************/
	
	reg	[27:0]	aclt_cnt;//cnt for accunmulation;
	reg	signed	[63:0]	X_buf;
	reg	signed	[63:0]	Y_buf;
	
	always@(posedge i_clk_1M or negedge i_rst_n)
	if(!i_rst_n)	begin
		aclt_cnt			<=	28'd0;
		X_buf				<=	64'd0;
		Y_buf				<=	64'd0;
		o_X					<=	64'd0;
		o_Y					<=	64'd0;
		end
	else	begin
		if((aclt_cnt >= point_of_period)||rst_flag)	begin
			aclt_cnt		<=	28'd0;
			X_buf			<=	64'd0;
			Y_buf			<=	64'd0;
			o_X				<=	X_buf;
			o_Y				<=	Y_buf;
			end
		else	begin
			aclt_cnt			<=	aclt_cnt + 28'd1;
			X_buf				<=	X_buf + dft_mul_X_buf;
			Y_buf				<=	Y_buf + dft_mul_Y_buf;
			o_X					<=	o_X;
			o_Y					<=	o_Y;
			end
		end

	
endmodule
