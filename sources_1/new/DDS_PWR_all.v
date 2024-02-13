`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/01/09 17:10:12
// Design Name: 
// Module Name: DDS_PWR
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


module DDS_PWR_all(
	
	input clk,
	input rstn,
	
    input ps_dds_pwr_down,
    
	output reg o_dds1_pwr_down,
	output reg o_dds2_pwr_down,
	output reg o_dds3_pwr_down
	
    );

reg temp_1;	
reg temp_2;	
wire rising_edge_pwr;

reg [1:0] state;
reg [15:0] cnt;
reg temp_dds1_pwr_down;	
reg temp_dds2_pwr_down;	
reg temp_dds3_pwr_down;

always @(posedge clk or negedge rstn)	begin 
	if(!rstn)	
	begin
		temp_1 <= 1'b0;
		temp_2 <= 1'b0;
	end
	
	else
	begin
		temp_1 <= ps_dds_pwr_down;
		temp_2 <= temp_1;
	end
end	

assign rising_edge_pwr = ({ps_dds_pwr_down,temp_1,temp_2} == 3'b110)? 1'b1 : 1'b0;

always @(posedge clk or negedge rstn)	
begin 
	if(!rstn)	
	begin
		state <= 2'b00;
		cnt   <= 16'b0;
		temp_dds1_pwr_down <= 1'b0;
		temp_dds2_pwr_down <= 1'b0;
		temp_dds3_pwr_down <= 1'b0;
	end
	
	else
	begin
		case(state)
		2'b00:
		begin
			if(rising_edge_pwr == 1'b1)
				state <= 2'b01;
			else
				state <= state;
				
			cnt <= cnt;
			temp_dds1_pwr_down <= temp_dds1_pwr_down;
			temp_dds2_pwr_down <= temp_dds2_pwr_down;
			temp_dds3_pwr_down <= temp_dds3_pwr_down;
		end
		
		2'b01:
		begin
			if(cnt == 16'd4000)
			begin
				cnt <= 16'd0;
				state <= 2'b10;
			end
			
			else			
			begin
				cnt <= cnt + 16'd1;
				state <= state;
			end
			
			temp_dds1_pwr_down <= 1'b1;
			temp_dds2_pwr_down <= 1'b1;
			temp_dds3_pwr_down <= 1'b1;	
		end
		
		2'b10:
		begin
			state <= 2'b00;
			cnt <= 16'd0;
			temp_dds1_pwr_down <= 1'b0;
			temp_dds2_pwr_down <= 1'b0;
			temp_dds3_pwr_down <= 1'b0;	
		end
		
		2'b11:
		begin
			state <= 2'b00;
			cnt <= 16'd0;
			temp_dds1_pwr_down <= 1'b0;
			temp_dds2_pwr_down <= 1'b0;
			temp_dds3_pwr_down <= 1'b0;	
		end	
		endcase
	end

end

always @(posedge clk or negedge rstn)	begin 
	if(!rstn)	
	begin
		o_dds1_pwr_down <= 1'b0;
		o_dds2_pwr_down <= 1'b0;
		o_dds3_pwr_down <= 1'b0;
	end
	
	else
	begin
		o_dds1_pwr_down <= temp_dds1_pwr_down;
		o_dds2_pwr_down <= temp_dds2_pwr_down;
		o_dds3_pwr_down <= temp_dds3_pwr_down;
	end
end	

// always @(posedge clk or negedge rstn)	begin 
	// if(!rstn)	begin
		// o_dds1_pwr_down <= 1'b1;
		// o_dds2_pwr_down <= 1'b1;
		// o_dds3_pwr_down <= 1'b1;
	// end
	
	// else	begin
		// o_dds1_pwr_down <= temp_dds_pwr_down;
		// o_dds2_pwr_down <= temp_dds_pwr_down;
		// o_dds3_pwr_down <= temp_dds_pwr_down;
	// end
// end
	
endmodule

