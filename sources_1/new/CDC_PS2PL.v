`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/09/17 16:31:55
// Design Name: 
// Module Name: CDC_PS2PL
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


module CDC_PS2PL(
	input				i_PL_clk,
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

    output reg [31:0] 	o_data_0,				
    output reg [31:0] 	o_data_1,				
    output reg [31:0] 	o_data_2,				
    output reg [31:0] 	o_data_3,				
    output reg [31:0] 	o_data_4,
    output reg [31:0] 	o_data_5,				
    output reg [31:0] 	o_data_6,				
    output reg [31:0] 	o_data_7,				
    output reg [31:0] 	o_data_8,				
    output reg [31:0] 	o_data_9  
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

/*************************				***************************/	
	always@(posedge i_PL_clk or negedge i_rst_n)
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

	always@(posedge i_PL_clk or negedge i_rst_n)
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

/******************************				***********/
	always@(posedge i_PL_clk or negedge i_rst_n)
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
			
	always@(posedge i_PL_clk or negedge i_rst_n)
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
			
endmodule
