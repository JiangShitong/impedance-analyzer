`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/01/06 11:19:19
// Design Name: 
// Module Name: IO_update
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


module IO_update(
	
	input clk,
	input rstn,
	
    input ps_dds_update,
    
	output reg o_dds_update
    );
	
reg temp_dds_update;
	
always @(posedge clk or negedge rstn)	begin 
	if(!rstn)
		temp_dds_update <= 1'b1;
	
	else
		temp_dds_update <= ps_dds_update;
end

always @(posedge clk or negedge rstn)	begin 
	if(!rstn)	
		o_dds_update <= 1'b1;
	
	else	
		o_dds_update <= temp_dds_update;
end
		
endmodule
