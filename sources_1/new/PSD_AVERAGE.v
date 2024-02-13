`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/23 11:06:35
// Design Name: 
// Module Name: PSD_AVERAGE
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


module PSD_AVERAGE(
	input i_clk,
	input i_rst_n,
	input signed [35:0] i_X,
	input signed [35:0] i_Y,
	output reg [35:0] o_X,
	output reg [35:0] o_Y,
	output reg				o_flag
    );
/*******************************************************************************/	
reg [6:0] cnt;

always @(posedge i_clk or negedge i_rst_n)
    if(!i_rst_n)
        cnt <= 7'd0;
    else
        cnt <= cnt + 1'b1;
		
reg signed [35:0] X;		
reg signed [35:0] Y;		

always @(posedge i_clk or negedge i_rst_n)
    if(!i_rst_n)	begin
        X <= 36'd0;
		Y <= 36'd0;
		end
    else	begin
        X <= i_X;
		Y <= i_Y;
		end
		
/*******************************		X VALUE ACCUMULATION     ************************************************/	
reg signed [42:0] sum_X;

always @(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
        sum_X <= 43'd0;
    else if(cnt !== 7'd0)
        sum_X <= sum_X + { {7{X[35]}}, X };
    else
        sum_X <= { {7{X[35]}}, X };
	end		
			
/*******************************		Y VALUE ACCUMULATION     ************************************************/	
reg signed [42:0] sum_Y;

always @(posedge i_clk or negedge i_rst_n)
begin
    if(!i_rst_n)
        sum_Y <= 43'd0;
    else if(cnt !== 7'd0)
        sum_Y <= sum_Y + { {7{Y[35]}}, Y };
    else
        sum_Y <= { {7{Y[35]}}, Y };
	end	

/*******************************		AVERAGE OUTPUT     ************************************************/		
always @(posedge i_clk or negedge i_rst_n)
begin
	if(!i_rst_n)	begin
		o_X	<= 36'd0;
		o_Y	<= 36'd0;
		o_flag <= 1'b0;
		end
	else if(cnt !== 7'd0)	begin
		o_X	<= o_X;
		o_Y	<= o_Y;
		o_flag <= 1'b0;
		end          
   else	begin
		o_X	<= sum_X[42:7];
		o_Y	<= sum_Y[42:7];
		o_flag <= 1'b1;		
		end 
	end	
		
endmodule
