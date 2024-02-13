`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/09/17 17:09:07
// Design Name: 
// Module Name: HC595_CTRL_AMP
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: The control logic is for digital board v3.0,which is different from v2.0
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module HC595_CTRL_AMP(
input               i_clk       ,
input               i_rst_n     ,
input   [3:0]       i_AMP_A      ,
input   [3:0]       i_AMP_B      ,
output  reg         o_SRCLR_n   ,
output  reg         o_RCLK      ,
output  reg         o_SER       ,
output  reg         o_SRCLK     
);

reg [7:0] AMP_A         ; 
reg [7:0] AMP_B         ;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
			AMP_A <= 8'b1111_1100;//x1
    else
			case(i_AMP_A)
				4'd0:		AMP_A <= 8'b1111_1100;//x1 							
				4'd1:		AMP_A <= 8'b0111_1101;//x2 
				4'd2:		AMP_A <= 8'b1011_1101;//x5 
				4'd3:		AMP_A <= 8'b1101_1101;//x10 
				4'd4:		AMP_A <= 8'b1101_1011;//x20 
				4'd5:		AMP_A <= 8'b1101_0111;//x50 
				4'd6:		AMP_A <= 8'b1100_1111;//x100 
				default: AMP_A <= 8'b1111_1100;//x1
			endcase
  
		
always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
			AMP_B <= 8'b1111_1100;//x1
    else
			case(i_AMP_B)
				4'd0:		AMP_B <= 8'b1111_1100;//x1 				
				4'd1:		AMP_B <= 8'b0111_1101;//x2 
				4'd2:		AMP_B <= 8'b1011_1101;//x5 
				4'd3:		AMP_B <= 8'b1101_1101;//x10 
				4'd4:		AMP_B <= 8'b1101_1011;//x20 
				4'd5:		AMP_B <= 8'b1101_0111;//x50 
				4'd6:		AMP_B <= 8'b1100_1111;//x100 
				default: AMP_B <= 8'b1111_1100;//x1
			endcase


reg [15:0] 	CONTROL_REG;
reg [5:0] 	cnt;
reg [3:0] 	state;
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		cnt			 <=  6'd0;
		state		 <=  4'd0; 
		o_SRCLK  <=  0;
		o_SER      <=  0;
		o_RCLK    <=  0;
		o_SRCLR_n   <=  1;
        end
    else	begin
		case(state)
			4'd0:	begin
				cnt         <=  0;
				o_SRCLK     <=  0;
				o_SER       <=  0;
				o_RCLK      <=  0;
				o_SRCLR_n   <=  1;
				state       <=  4'd1;
                end
			4'd1:	begin        
				CONTROL_REG <=  {AMP_B[7], AMP_B[6], AMP_B[5], AMP_B[4], AMP_B[3], AMP_B[2], AMP_B[1], AMP_B[0], AMP_A[7], AMP_A[6], AMP_A[5], AMP_A[4], AMP_A[3], AMP_A[2], AMP_A[1], AMP_A[0]};
				state       <=  4'd2;
				end
			4'd2:	begin
				o_SER       <=  CONTROL_REG[15];
				o_SRCLK     <=  0;
				state       <=  4'd3;
				end
			4'd3:	begin
				o_SRCLK     <=  1;
				CONTROL_REG <=  CONTROL_REG << 1;
				cnt         <=  cnt + 1'b1;
				if(cnt<15)
					state   <=  4'd2;
				else
					state   <=  4'd4;
				end
			4'd4:	begin
				o_SRCLK     <=  0;
				o_SER       <=  0;
				o_RCLK      <=  1;
				state       <=  4'd5;
				end
			4'd5:       state   <=  4'd0;
			default:    state   <=  4'd0;
		endcase
        end

 
endmodule