`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/27 20:18:27
// Design Name: 
// Module Name: HC5959_CTRL_ANALOG
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


module HC595_CTRL_ANALOG(
	input                 i_clk             ,
	input                 i_rst_n           ,
	input [1:0]			  i_CTRL_Vr_Sel     ,	
	input [2:0]			  i_CTRL_Rr_Sel     ,
	input [3:0]			  i_CTRL_Measure_Sel,
    input 				  i_CTRL_Bias_ON    ,
    input                 i_CTRL_Vx_Vr_AMP    ,
	output  	reg      o_OE_n             ,
	output  	reg      o_SRCLR_n          ,
	output  	reg      o_RCLK             ,
	output  	reg      o_SER              ,
	output  	reg      o_SRCLK     
);

reg  [1:0]	CTRL_Vr_Sel;	
reg  [2:0]	CTRL_Rr_Sel;
reg  [3:0]	CTRL_Measure_Sel;
reg  [1:0]	CTRL_freq_switch;
reg		    CTRL_Bias_ON;	
reg         CTRL_Vx_Vr_AMP;	

always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		CTRL_Vr_Sel				<= 2'b0;
        CTRL_Rr_Sel			    <= 3'b0;
        CTRL_Measure_Sel	<= 4'b0;
        CTRL_Bias_ON			<= 1'b0;
        CTRL_Vx_Vr_AMP	        <= 1'b1;
		end
	else	begin
		CTRL_Vr_Sel				<= i_CTRL_Vr_Sel;			
        CTRL_Rr_Sel			    <= i_CTRL_Rr_Sel;		
        CTRL_Measure_Sel	<= i_CTRL_Measure_Sel;	
        CTRL_Bias_ON			<= i_CTRL_Bias_ON;
        CTRL_Vx_Vr_AMP			<= i_CTRL_Vx_Vr_AMP;		
		end
		
		

/*****************************					**********************************/
reg	[1:0]	 freq_switch;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
			freq_switch <= 2'b00;//      high bridge
			end
    else	begin
			case(CTRL_Vr_Sel)
				2'd0:		begin
									freq_switch <= 2'b11		;//		low bridge
									end
				2'd1:		begin
									freq_switch <= 2'b00	;	//      high bridge
									end
				2'd2:		begin
									freq_switch <= 2'b11		;	//		low bridge
									end									
				2'd3:		begin
									freq_switch <= 2'b11	;		//		low bridge
									end									
				default: 	begin
									freq_switch <= 2'b11		;	//		low bridge
									end									
			endcase
			end

/*****************************					**********************************/
reg	[4:0]	 Rr_Sel				;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
			Rr_Sel <= 5'b10010;//none
    else
			case(CTRL_Rr_Sel)
				3'd0:		Rr_Sel <= 5'b10000		;	// 500R				
				3'd1:		Rr_Sel <= 5'b11010		;	// 5KR
				3'd2:		Rr_Sel <= 5'b10110		;	// 50kR
				3'd3:		Rr_Sel <= 5'b10011		;	// 50R	
				3'd4:		Rr_Sel <= 5'b00010		;	// 500kR
				default: 	Rr_Sel <= 5'b11010		;	// 5KR
			endcase
			


reg	[9:0] Measure_Sel			;
always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
			Measure_Sel <= 10'b0011_0000_00	;	// GND
    else
			case(CTRL_Measure_Sel)
				4'd0:		                        Measure_Sel <= 10'b0110_0000_00							;	//	Vx					
				4'd1:		begin                                          
								case(CTRL_Rr_Sel)                           
									3'd0:			Measure_Sel <= 10'b1001_0000_00						;	// Vr_500R	
									3'd1:			Measure_Sel <= 10'b1010_0000_00						;	// Vr_5K
									3'd2:			Measure_Sel <= 10'b0000_0000_00						;	// Vr_50kR
									3'd3:			Measure_Sel <= 10'b1011_0000_00						;	// Vr_50R
									3'd4:			Measure_Sel <= 10'b1000_0000_00						;	// Vr_500kR
									default:	    Measure_Sel <= 10'b1001_0000_00						;	// Vr_500R
									endcase
								end				
				4'd2:		                        Measure_Sel <= 10'b0011_1100_00					;	// Lp
				4'd3:		                        Measure_Sel <= 10'b0011_0110_01						;	// Lc
				4'd4:		                        Measure_Sel <= 10'b0110_1100_00					;	// Vx-Lp
				4'd5:		begin                                  
								case(CTRL_Rr_Sel)                  
									3'd0:			Measure_Sel <= 10'b1001_0110_01							;	// Vr_500R-Lc
									3'd1:			Measure_Sel <= 10'b1010_0110_01							;	// Vr_5K-Lc	
									3'd2:			Measure_Sel <= 10'b0000_0110_01							;	// Vr_50K-Lc
									3'd3:			Measure_Sel <= 10'b1001_0110_01							;	// Vr500R-Lc
									3'd4:			Measure_Sel <= 10'b1001_0110_01							;	// Vr500R-Lc
									default:	    Measure_Sel <= 10'b1001_0110_01							;	// Vr500R-Lc
									endcase                        
								end		                           
				4'd6:		                        Measure_Sel <= 10'b0011_0000_00						;	// GND
				default: 	                        Measure_Sel <= 10'b0110_0000_00						;	// Vx
			endcase	

/*****************************					**********************************/


/*****************************					**********************************/
reg			 Bias_ON		;

always @(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
			Bias_ON <= 1'b0;//bias off
    else
			case(CTRL_Bias_ON)
				1'd0:		Bias_ON <= 3'b0		;	//	bias off					
				1'd1:		Bias_ON <= 3'b1		;	// bias on
				default: 	Bias_ON <= 3'b0		;	// bias off
			endcase	
/*****************************					**********************************/

(*mark_debug = "true" *)reg [31:0] 	CONTROL_REG;
(*mark_debug = "true" *)reg [5:0] 	cnt;
reg [3:0] 	state;

always@(posedge i_clk or negedge i_rst_n)
    if(!i_rst_n)	begin
            cnt         		<=  6'd0;
            state       	<=  4'd0; 
            o_SRCLK     <=  0;
            o_SER      	<=  0;
            o_RCLK      	<=  0;
            o_SRCLR_n <=  1;
            o_OE_n      	<=  0;            
			end
    else	begin
		case(state)
			4'd0:	begin
				cnt         		<=  0;
				o_SRCLK  	<=  0;
				o_SER       	<=  0;
				o_RCLK      	<=  0;
				o_SRCLR_n <=  1;
				o_OE_n      	<=  0;
				state       	<=  4'd1;
				end
			4'd1:	begin        
				CONTROL_REG <=  {1'd0, Measure_Sel[9], 1'd1, 1'd0, Measure_Sel[8], Rr_Sel[4], Measure_Sel[7], Rr_Sel[3], Measure_Sel[6], 1'd0, Rr_Sel[2], 1'd0, Rr_Sel[1], 1'd0, Rr_Sel[0], 1'd0, 1'd0, 1'd0, freq_switch[1], freq_switch[0], 1'd0, 1'd1, Measure_Sel[5], 1'd0, 1'd0, Measure_Sel[4], 1'd1, Measure_Sel[3], 1'd1, Measure_Sel[2], Measure_Sel[1], Measure_Sel[0]};
				state       <=  4'd2;
				end
			4'd2:	begin
				o_SER       <=  CONTROL_REG[31];
				o_SRCLK     <=  0;
				state       <=  4'd3;
				end
			4'd3:	begin
				o_SRCLK     <=  1;
				CONTROL_REG <=  CONTROL_REG << 1;
				cnt         <=  cnt + 1'b1;
				if(cnt < 31)
					state   <=  4'd2;
				else
					state   <=  4'd4;
				end
			4'd4:	begin
				o_SRCLK   	<=  0;
				o_SER       	<=  0;
				o_RCLK   	<=  1;
				state       	<=  4'd5;
				end
			4'd5:  begin     
				state   <=  4'd0;
				end
		   default:    state   <=  4'd0;
           endcase
        end
		
endmodule

