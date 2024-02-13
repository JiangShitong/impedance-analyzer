`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/20 13:48:46
// Design Name: 
// Module Name: IIR_SHIFT
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


module IIR_SHIFT(
	input 									i_clk              		,
	input 									i_rst_n             	,
	input [3:0] 						i_coefficient		,		
	input signed [35:0] 			i_data              	,    
	output  reg [3:0] 					o_coefficient		,		
	output reg signed[35:0] 	o_data               
    );
	
/**********************************************************************/	
reg signed [35:0] data;

always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		data <= 36'd0;
	else
		data <= i_data;	
	
///*********************************** IIR SHIFT ***********************************/	
//reg signed [35:0] filter_out_10us;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_10us <= 36'd0;
//	else
//		filter_out_10us <= {{4{data[35]}}, data[35:3]} - {{4{filter_out_10us[35]}}, filter_out_10us[35:3]} + {filter_out_10us[35], filter_out_10us};	
//	
//reg signed [35:0] filter_out_30us;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_30us <= 36'd0;
//	else
//		filter_out_30us <= {{6{data[35]}}, data[35:5]} - {{6{filter_out_30us[35]}}, filter_out_30us[35:5]} + {filter_out_30us[35], filter_out_30us};
//	
//reg signed [35:0] filter_out_100us;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_100us <= 36'd0;
//	else
//		filter_out_100us <= {{8{data[35]}}, data[35:7]} - {{8{filter_out_100us[35]}}, filter_out_100us[35:7]} + {filter_out_100us[35], filter_out_100us};
//
//reg signed [35:0] filter_out_300us;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_300us <= 36'd0;
//	else
//		filter_out_300us <= {{9{data[35]}}, data[35:8]} - {{9{filter_out_300us[35]}}, filter_out_300us[35:8]} + {filter_out_300us[35], filter_out_300us};	
//
//reg signed [35:0] filter_out_1ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_1ms <= 36'd0;
//	else
//		filter_out_1ms <= {{11{data[35]}}, data[35:10]} - {{11{filter_out_1ms[35]}}, filter_out_1ms[35:10]} + {filter_out_1ms[35], filter_out_1ms};
//
//reg signed [35:0] filter_out_3ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_3ms <= 36'd0;
//	else
//		filter_out_3ms <= {{13{data[35]}}, data[35:12]} - {{13{filter_out_3ms[35]}}, filter_out_3ms[35:12]} + {filter_out_3ms[35], filter_out_3ms};
//
//reg signed [35:0] filter_out_10ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_10ms <= 36'd0;
//	else
//		filter_out_10ms <= {{14{data[35]}}, data[35:13]} - {{14{filter_out_10ms[35]}}, filter_out_10ms[35:13]} + {filter_out_10ms[35], filter_out_10ms};
//
//reg signed [35:0] filter_out_30ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_30ms <= 36'd0;
//	else
//		filter_out_30ms <= {{16{data[35]}}, data[35:15]} - {{16{filter_out_30ms[35]}}, filter_out_30ms[35:15]} + {filter_out_30ms[35], filter_out_30ms};		
//
//reg signed [35:0] filter_out_100ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_100ms <= 36'd0;
//	else
//		filter_out_100ms <= {{18{data[35]}}, data[35:17]} - {{18{filter_out_100ms[35]}}, filter_out_100ms[35:17]} + {filter_out_100ms[35], filter_out_100ms};
//
//reg signed [35:0] filter_out_300ms;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_300ms <= 36'd0;
//	else
//		filter_out_300ms <= {{19{data[35]}}, data[35:18]} - {{19{filter_out_300ms[35]}}, filter_out_300ms[35:18]} + {filter_out_300ms[35], filter_out_300ms};				
//		
//reg signed [35:0] filter_out_1s;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_1s <= 36'd0;
//	else
//		filter_out_1s <= {{21{data[35]}}, data[35:20]} - {{21{filter_out_1s[35]}}, filter_out_1s[35:20]} + {filter_out_1s[35], filter_out_1s};
//
//reg signed [35:0] filter_out_3s;
//always@(posedge i_clk or negedge i_rst_n)
//	if(!i_rst_n)
//		filter_out_3s <= 36'd0;
//	else
//		filter_out_3s <= {{23{data[35]}}, data[35:22]} - {{23{filter_out_3s[35]}}, filter_out_3s[35:22]} + {filter_out_3s[35], filter_out_3s};	
//	
reg [3:0] coefficient;
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	
		coefficient <= 4'd0;
	else
		coefficient <= i_coefficient;
		
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)	
		o_coefficient <= 4'd0;
	else
		o_coefficient <= coefficient;
		
reg [35:0] shift_data;	
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		shift_data <= 36'd0;
	else
		case(coefficient)
			4'b0001:	shift_data <= {{4{data[35]}}, data[35:3]} - {{4{shift_data[35]}}, shift_data[35:3]} + {shift_data[35], shift_data};	//10us 
			4'b0010: 	shift_data <= {{6{data[35]}}, data[35:5]} - {{6{shift_data[35]}}, shift_data[35:5]} + {shift_data[35], shift_data};  //30us
			4'b0011: 	shift_data <= {{8{data[35]}}, data[35:7]} - {{8{shift_data[35]}}, shift_data[35:7]} + {shift_data[35], shift_data};  //100us
			4'b0100: 	shift_data <= {{9{data[35]}}, data[35:8]} - {{9{shift_data[35]}}, shift_data[35:8]} + {shift_data[35], shift_data};	  //300us
			4'b0101: 	shift_data <= {{11{data[35]}}, data[35:10]} - {{11{shift_data[35]}}, shift_data[35:10]} + {shift_data[35], shift_data};  //1ms
			4'b0110: 	shift_data <= {{13{data[35]}}, data[35:12]} - {{13{shift_data[35]}}, shift_data[35:12]} + {shift_data[35], shift_data};  //3ms
			4'b0111: 	shift_data <= {{14{data[35]}}, data[35:13]} - {{14{shift_data[35]}}, shift_data[35:13]} + {shift_data[35], shift_data};  //10ms
			4'b1000: 	shift_data <= {{16{data[35]}}, data[35:15]} - {{16{shift_data[35]}}, shift_data[35:15]} + {shift_data[35], shift_data};		  //30ms
			4'b1001: 	shift_data <= {{18{data[35]}}, data[35:17]} - {{18{shift_data[35]}}, shift_data[35:17]} + {shift_data[35], shift_data};  //100ms
			4'b1010: 	shift_data <= {{19{data[35]}}, data[35:18]} - {{19{shift_data[35]}}, shift_data[35:18]} + {shift_data[35], shift_data};	  //300ms ,default
			4'b1011: 	shift_data <= {{21{data[35]}}, data[35:20]} - {{21{shift_data[35]}}, shift_data[35:20]} + {shift_data[35], shift_data};  //1s
			4'b1100: 	shift_data <= {{23{data[35]}}, data[35:22]} - {{23{shift_data[35]}}, shift_data[35:22]} + {shift_data[35], shift_data};  //3s
			default:	shift_data <= {{19{data[35]}}, data[35:18]} - {{19{shift_data[35]}}, shift_data[35:18]} + {shift_data[35], shift_data};
		endcase	
	
always@(posedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		o_data <= 36'd0;
	else
		o_data <= shift_data;
	
	
	
endmodule
