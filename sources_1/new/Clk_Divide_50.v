`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/11/12 21:50:17
// Design Name: 
// Module Name: Clk_Divide_100
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


module Clk_Divide_50(
	input 	wire				i_clk,
	input 	wire				i_rst_n,
	output 	reg			 	o_clk,
	output 	reg			 	o_sync
	);
	
reg [5:0] cnt;
always @(posedge i_clk or negedge i_rst_n)  
begin
	if(!i_rst_n)
		cnt <= 6'd0 ;
	else	if(!(cnt ^ 6'd49))
		cnt <= 6'd0;
	else
		cnt <= cnt + 6'd1;	
	end

always @(posedge i_clk or negedge i_rst_n)  
begin
	if(!i_rst_n)
		o_clk <= 1'b0;
	else
		if(!(cnt ^ 6'd24))
			o_clk <= 1'b0;
		else if(!(cnt ^ 6'd49))
			o_clk <= 1'b1;
		else 
			o_clk <=  o_clk;
	end	

always @(posedge i_clk or negedge i_rst_n)  
begin
	if(!i_rst_n)
		o_sync <= 1'b0;
	else	if(!(cnt ^ 6'd49))
		o_sync <= 1'b1;
	else 
		o_sync <=  1'b0;	
	end

	
endmodule
	
