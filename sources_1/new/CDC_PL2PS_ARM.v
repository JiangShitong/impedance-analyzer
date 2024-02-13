`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/28 21:10:03
// Design Name: 
// Module Name: CDC_PL2PS
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


module CDC_PL2PS(
	input				i_PS_clk,
	input				i_rst_n,
    input [31:0] 	i_data_0,				
    input [31:0] 	i_data_1,				
    input [31:0] 	i_data_2,				
    input [31:0] 	i_data_3,				
    input [31:0] 	i_data_4,				
    input [31:0] 	i_data_5,				
    input [31:0] 	i_data_6,				
    input [31:0] 	i_data_7,				
    input [31:0] 	i_data_8,				
    input [31:0] 	i_data_9,
    input [31:0] 	i_data_10,				
    input [31:0] 	i_data_11,				
    input [31:0] 	i_data_12,				
    input [31:0] 	i_data_13,				
    input [31:0] 	i_data_14,		

    output reg [31:0] 	o_data_0,				
    output reg [31:0] 	o_data_1,				
    output reg [31:0] 	o_data_2,				
    output reg [31:0] 	o_data_3,				
    output reg [31:0] 	o_data_4,
    output reg [31:0] 	o_data_5,				
    output reg [31:0] 	o_data_6,				
    output reg [31:0] 	o_data_7,				
    output reg [31:0] 	o_data_8,				
    output reg [31:0] 	o_data_9,    
    output reg [31:0] 	o_data_10,				
    output reg [31:0] 	o_data_11,				
    output reg [31:0] 	o_data_12,				
    output reg [31:0] 	o_data_13,				
    output reg [31:0] 	o_data_14        
);

reg [31:0] data_0;
reg [31:0] data_1;
reg [31:0] data_2;
reg [31:0] data_3;
reg [31:0] data_4;
reg [31:0] data_5;
reg [31:0] data_6;
reg [31:0] data_7;
reg [31:0] data_8;
reg [31:0] data_9;
reg [31:0] data_10;
reg [31:0] data_11;
reg [31:0] data_12;
reg [31:0] data_13;
reg [31:0] data_14;

/*************************				***************************/	
	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			data_0 <= 32'd0;
			data_1 <= 32'd0;
			data_2 <= 32'd0;
			data_3 <= 32'd0;
			data_4 <= 32'd0;
			end
		else	begin
			data_0 <= i_data_0;
			data_1 <= i_data_1;
			data_2 <= i_data_2;
			data_3 <= i_data_3;
			data_4 <= i_data_4;
			end	

	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			data_5 <= 32'd0;
			data_6 <= 32'd0;
			data_7 <= 32'd0;
			data_8 <= 32'd0;
			data_9 <= 32'd0;
			end
		else	begin
			data_5 <= i_data_5;
			data_6 <= i_data_6;
			data_7 <= i_data_7;
			data_8 <= i_data_8;
			data_9 <= i_data_9;
			end				
	
	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			data_10 <= 32'd0;
			data_11 <= 32'd0;
			data_12 <= 32'd0;
			data_13 <= 32'd0;
			data_14 <= 32'd0;
			end
		else	begin
			data_10 <= i_data_10;
			data_11 <= i_data_11;
			data_12 <= i_data_12;
			data_13 <= i_data_13;
			data_14 <= i_data_14;
			end			

/******************************				***********/
	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			o_data_0 <= 32'd0;
			o_data_1 <= 32'd0;
			o_data_2 <= 32'd0;
			o_data_3 <= 32'd0;
			o_data_4 <= 32'd0;
			end
		else	begin
			o_data_0 <= data_0;
			o_data_1 <= data_1;
			o_data_2 <= data_2;
			o_data_3 <= data_3;
			o_data_4 <= data_4;
			end	
			
	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			o_data_5 <= 32'd0;
			o_data_6 <= 32'd0;
			o_data_7 <= 32'd0;
			o_data_8 <= 32'd0;
			o_data_9 <= 32'd0;
			end
		else	begin
			o_data_5 <= data_5;
			o_data_6 <= data_6;
			o_data_7 <= data_7;
			o_data_8 <= data_8;
			o_data_9 <= data_9;
			end	

	always@(posedge i_PS_clk or negedge i_rst_n)
		if(!i_rst_n)	begin
			o_data_10 <= 32'd0;
			o_data_11 <= 32'd0;
			o_data_12 <= 32'd0;
			o_data_13 <= 32'd0;
			o_data_14 <= 32'd0;
			end
		else	begin
			o_data_10 <= data_10;
			o_data_11 <= data_11;
			o_data_12 <= data_12;
			o_data_13 <= data_13;
			o_data_14 <= data_14;
			end				
			
endmodule
