`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/20 11:58:11
// Design Name: 
// Module Name: PSD_IIR_LPF
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


module PSD_IIR_LPF(
	input										i_clk							,
    input										i_rst_n						,
    input [3:0] 							i_coefficient				,
    input [1:0] 							i_mod						,
    input signed [35:0] 				i_A_X							,
    input signed [35:0] 				i_A_Y							,
    input signed [35:0] 				i_B_X							,
    input signed [35:0] 				i_B_Y							,
	output reg [35:0] 				o_A_X						,
	output reg [35:0] 				o_A_Y						,
	output reg [35:0] 				o_B_X						,
	output reg [35:0] 				o_B_Y						,
	output reg 							o_IIR_valid	
);

/**********************************		FILTER COEFFICIENT		*****************************/
(* dont_touch = "true" *)reg [3:0] coefficient_A;
(* dont_touch = "true" *)reg [3:0] coefficient_B;

always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		coefficient_A <= 4'd0;
		coefficient_B <= 4'd0;
		end
	else	begin
		coefficient_A <= i_coefficient;
		coefficient_B <= i_coefficient;
		end		

/**********************************		DATA INPUT		*****************************/
reg [35:0] A_X;
reg [35:0] A_Y;
reg [35:0] B_X;
reg [35:0] B_Y;
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		A_X <= 36'd0;
		A_Y <= 36'd0;
		B_X <= 36'd0;
		B_Y <= 36'd0;		
		end
	else	begin
		A_X <=  i_A_X;
		A_Y <=  i_A_Y;
		B_X <=  i_B_X;
		B_Y <=  i_B_Y;	
		end

/*********************************		IIR FILTER		**************************************/
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_X_0;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_X_1;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_X_2;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_X_3;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_Y_0;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_Y_1;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_Y_2;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_A_Y_3;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_X_0;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_X_1;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_X_2;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_X_3;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_Y_0;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_Y_1;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_Y_2;
(*mark_debug = "true" *)(* dont_touch = "true" *)wire [35:0] iir_shift_B_Y_3;

/*******************************************************************************/
(* dont_touch = "true" *)wire[3:0] coefficient_A_X_0;
(* dont_touch = "true" *)wire[3:0] coefficient_A_X_1;
(* dont_touch = "true" *)wire[3:0] coefficient_A_X_2;
(* dont_touch = "true" *)wire[3:0] coefficient_A_Y_0;
(* dont_touch = "true" *)wire[3:0] coefficient_A_Y_1;
(* dont_touch = "true" *)wire[3:0] coefficient_A_Y_2;
(* dont_touch = "true" *)wire[3:0] coefficient_B_X_0;
(* dont_touch = "true" *)wire[3:0] coefficient_B_X_1;
(* dont_touch = "true" *)wire[3:0] coefficient_B_X_2;
(* dont_touch = "true" *)wire[3:0] coefficient_B_Y_0;
(* dont_touch = "true" *)wire[3:0] coefficient_B_Y_1;
(* dont_touch = "true" *)wire[3:0] coefficient_B_Y_2;

IIR_SHIFT Iir_shift_A_X_0(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A),       		
	.i_data              		(A_X),        
	.o_coefficient		(coefficient_A_X_0),
	.o_data                    (iir_shift_A_X_0)
);

IIR_SHIFT Iir_shift_A_X_1(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_X_0),       		
	.i_data              		(iir_shift_A_X_0), 
	.o_coefficient		(coefficient_A_X_1),	
	.o_data                    (iir_shift_A_X_1)
);

IIR_SHIFT Iir_shift_A_X_2(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_X_1),       		
	.i_data              		(iir_shift_A_X_1), 
	.o_coefficient		(coefficient_A_X_2),	
	.o_data                    (iir_shift_A_X_2)
);

IIR_SHIFT Iir_shift_A_X_3(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_X_2),       		
	.i_data              		(iir_shift_A_X_2),  
	.o_coefficient		(	),	
	.o_data                    (iir_shift_A_X_3)
);
	
/*******************************************************************************/
IIR_SHIFT Iir_shift_A_Y_0(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A),       		
	.i_data              		(A_Y),     
	.o_coefficient		(coefficient_A_Y_0),		
	.o_data                    (iir_shift_A_Y_0)
);

IIR_SHIFT Iir_shift_A_Y_1(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_Y_0),       		
	.i_data              		(iir_shift_A_Y_0),  
	.o_coefficient		(coefficient_A_Y_1),			
	.o_data                    (iir_shift_A_Y_1)
);

IIR_SHIFT Iir_shift_A_Y_2(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_Y_1),       		
	.i_data              		(iir_shift_A_Y_1),  
	.o_coefficient		(coefficient_A_Y_2),			
	.o_data                    (iir_shift_A_Y_2)
);

IIR_SHIFT Iir_shift_A_Y_3(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_A_Y_2),       		
	.i_data              		(iir_shift_A_Y_2), 
	.o_coefficient		(			),			
	.o_data                    (iir_shift_A_Y_3)
);
	
/*******************************************************************************/
IIR_SHIFT Iir_shift_B_X_0(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B),       		
	.i_data              		(B_X),      
	.o_coefficient		(coefficient_B_X_0),			
	.o_data                    (iir_shift_B_X_0)
);

IIR_SHIFT Iir_shift_B_X_1(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_X_0),       		
	.i_data              		(iir_shift_B_X_0),   
	.o_coefficient		(coefficient_B_X_1),			
	.o_data                    (iir_shift_B_X_1)
);

IIR_SHIFT Iir_shift_B_X_2(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_X_1),       		
	.i_data              		(iir_shift_B_X_1),
	.o_coefficient		(coefficient_B_X_2),			
	.o_data                    (iir_shift_B_X_2)
);

IIR_SHIFT Iir_shift_B_X_3(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_X_2),       		
	.i_data              		(iir_shift_B_X_2),  
	.o_coefficient		(		),			
	.o_data                    (iir_shift_B_X_3)
);
	
/*******************************************************************************/
IIR_SHIFT Iir_shift_B_Y_0(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B),       		
	.i_data              		(B_Y),          
	.o_coefficient		(coefficient_B_Y_0),			
	.o_data                    (iir_shift_B_Y_0)
);

IIR_SHIFT Iir_shift_B_Y_1(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_Y_0),       		
	.i_data              		(iir_shift_B_Y_0),  
	.o_coefficient		(coefficient_B_Y_1),		
	.o_data                    (iir_shift_B_Y_1)
);

IIR_SHIFT Iir_shift_B_Y_2(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_Y_1),       		
	.i_data              		(iir_shift_B_Y_1),   
	.o_coefficient		(coefficient_B_Y_2),		
	.o_data                    (iir_shift_B_Y_2)
);

IIR_SHIFT Iir_shift_B_Y_3(
	.i_clk              			(i_clk),  
	.i_rst_n             		(i_rst_n),       		  
	.i_coefficient			(coefficient_B_Y_2),       		
	.i_data              		(iir_shift_B_Y_2),     
	.o_coefficient		(		),		
	.o_data                    (iir_shift_B_Y_3)
);	

/*******************************************************************************/
//This implies filter slope. 
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	begin
		o_A_X <= 48'd0;
	    o_A_Y <= 48'd0;
	    o_B_X <= 48'd0;
	    o_B_Y <= 48'd0;
		o_IIR_valid <= 1'b0;
		end
	else	begin
		case(i_mod)
			2'b00:	begin		//6dB
				o_A_X <= iir_shift_A_X_0;
	            o_A_Y <= iir_shift_A_Y_0;
	            o_B_X <= iir_shift_B_X_0;
	            o_B_Y <= iir_shift_B_Y_0;
				o_IIR_valid <= 1'b1;
				end
			2'b01:	begin		//12dB
				o_A_X <= iir_shift_A_X_1;
	            o_A_Y <= iir_shift_A_Y_1;
	            o_B_X <= iir_shift_B_X_1;
	            o_B_Y <= iir_shift_B_Y_1;
				o_IIR_valid <= 1'b1;
				end	
			2'b10:	begin		//18dB
				o_A_X <= iir_shift_A_X_2;
	            o_A_Y <= iir_shift_A_Y_2;
	            o_B_X <= iir_shift_B_X_2;
	            o_B_Y <= iir_shift_B_Y_2;
				o_IIR_valid <= 1'b1;
				end	
			2'b11:	begin		//24dB
				o_A_X <= iir_shift_A_X_3;
	            o_A_Y <= iir_shift_A_Y_3;
	            o_B_X <= iir_shift_B_X_3;
	            o_B_Y <= iir_shift_B_Y_3;
				o_IIR_valid <= 1'b1;
				end	
			default:	begin
				o_A_X <= 48'd0;
	            o_A_Y <= 48'd0;
	            o_B_X <= 48'd0;
	            o_B_Y <= 48'd0;
				o_IIR_valid <= 1'b0;
				end	
			endcase
		end






endmodule
