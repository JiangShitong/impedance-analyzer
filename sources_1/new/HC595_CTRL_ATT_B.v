`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Zachary Chan 
// 
// Create Date: 2018/07/26 19:55:30
// Design Name: 
// Module Name: HC595_CTRL_ATT
// Project Name:  Impedance Measurement
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


module HC595_CTRL_ATT_B(
	input               i_clk       ,
	input               i_rst_n     ,
	input   [3:0]       i_ATT_B      ,
	output  	reg         o_SRCLR_n   ,
	output  	reg         o_RCLK      ,
	output  	reg         o_SER       ,
	output  	reg         o_SRCLK     
);

reg [7:0] ATT_B;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		ATT_B <= 6'b00_0000;		//no attention
    else
		case(i_ATT_B)
			4'd0:		ATT_B <= 6'b00_0000;		//20lg(1) =  0
			4'd1:		ATT_B <= 6'b00_0010;		//20lg(2) = 6			= 2+4							-2
			4'd2:		ATT_B <= 6'b00_0110;		//20lg(5) = 14		= 2+4+8                      -2
			4'd3:		ATT_B <= 6'b00_1001;		//20lg(10) = 20		= 4+16                        -2
			4'd4:		ATT_B <= 6'b00_1100;		//20lg(20) = 26		= 2+8+16                    -2
			4'd5:		ATT_B <= 6'b11_0001;		//20lg(50) = 34		= 8+16+10                	-2
			4'd6:		ATT_B <= 6'b01_1110;		//20lg(100) = 40	= 2+4+8+16+10         -2
			4'd7:		ATT_B <= 6'b10_1100;		//20lg(200) = 46	= 16+10+20               -2
			4'd8:		ATT_B <= 6'b11_1011;		//20lg(500) = 54	= 8+16+10+20          	-2
			4'd9:		ATT_B <= 6'b11_1110;		//20lg(1000) = 60	= 2+4+8+16+10+20    -2
			default: 	ATT_B <= 6'b00_0000;		//20lg(1) =  0						
		endcase

(*mark_debug = "true" *)reg [7:0] 	CONTROL_REG;
reg [3:0] 	cnt;
reg [3:0] 	state;
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		cnt			 <=  4'd0;
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
				CONTROL_REG <=  {1'b0, ATT_B[4], ATT_B[3], ATT_B[5], ATT_B[1], ATT_B[0], ATT_B[2], 1'b0};
				state       <=  4'd2;
				end
			4'd2:	begin
				o_SER       <=  CONTROL_REG[7];
				o_SRCLK     <=  0;
				state       <=  4'd3;
				end
			4'd3:	begin
				o_SRCLK     <=  1;
				CONTROL_REG <=  CONTROL_REG << 1;
				cnt         <=  cnt + 1'b1;
				if(cnt<7)
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
