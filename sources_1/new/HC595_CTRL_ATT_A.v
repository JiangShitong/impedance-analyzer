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


module HC595_CTRL_ATT_A(
	input               i_clk       ,
	input               i_rst_n     ,
	input   [3:0]       i_ATT_A      ,
	output  	reg         o_OE_n      ,
	output  	reg         o_SRCLR_n   ,
	output  	reg         o_RCLK      ,
	output  	reg         o_SER       ,
	output  	reg         o_SRCLK     
);

reg [7:0] ATT_A;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		ATT_A <= 6'b00_0000;		//no attention
    else
		case(i_ATT_A)
			4'd0:		ATT_A <= 6'b00_0000;		//20lg(1) =  0   20,10,2,4,8,16
			4'd1:		ATT_A <= 6'b00_1100;		//20lg(2) = 6			= 2+4
			4'd2:		ATT_A <= 6'b01_1100;		//20lg(5) = 14		= 4+10	 =16=10+4+2					
			4'd3:		ATT_A <= 6'b00_1101;		//20lg(10) = 20	= 4+16 =22=16+4+2						
			4'd4:		ATT_A <= 6'b01_1001;		//20lg(20) = 26		= 16+10		=28=16+10+2				
			4'd5:		ATT_A <= 6'b01_0011;		//20lg(50) = 34		= 8+16+10 =36   =8+6+10+2                 
			4'd6:		ATT_A <= 6'b10_1101;		//20lg(100) = 40	= 4+16+20 =42   =20+2+4+16            
			4'd7:		ATT_A <= 6'b11_1001;		//20lg(200) = 46	= 16+10+20   48 =20+10+16+2              
			4'd8:		ATT_A <= 6'b11_1011;		//20lg(500) = 54	= 8+16+10+20 56=20+10+2+8+16			
			4'd9:		ATT_A <= 6'b11_1111;		//20lg(1000) = 60	= 2+4+8+16+10+20   
			default: 	ATT_A <= 6'b00_0000;		//20lg(1) =  0       			
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
		o_OE_n   <=  0;            
        end
    else	begin
		case(state)
			4'd0:	begin
				cnt         <=  0;
				o_SRCLK     <=  0;
				o_SER       <=  0;
				o_RCLK      <=  0;
				o_SRCLR_n   <=  1;
				o_OE_n      <=  0;
				state       <=  4'd1;
                end
			4'd1:	begin        
				CONTROL_REG <=  {1'b0, ATT_A[4], ATT_A[0], ATT_A[5], ATT_A[2], ATT_A[3], ATT_A[1], 1'b0};
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
