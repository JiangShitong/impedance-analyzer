`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/11/29 09:42:09
// Design Name: 
// Module Name: LTC2378
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


module LTC2378(
input               i_rst_n             	,
input               i_SDO               	, //AD Serial Data Out
input               i_clk               	, //50MHz is the highest clk frequency because tDSDO
input               i_BUSY					, //1: start conversion; 0: finish conversion

output reg          o_CNV              	    , //AD Convert Enable, Posedge 
output  		    o_SCK              	    , //AD Serial Clock In
output reg			o_AD_data_syn      	    ,
output reg [19:0]   o_AD_data          	    , //AD Data
output reg          o_AD_Overload,
output reg				o_flag
);

//T(cycle) = T(BUSY_H) + T(Acquire) + T(Conversion)
//T(cycle) = 1/1Msps = 1000ns       Sample Rate, 80 clock periods.     
//T(BUSY_H) = 1/80Msps = 12.5ns     Max = 13ns, Total need 25ns, 1 clock period(80MHz)
//T(Acquire) =                      Min = 312ns     continue for 24 periods(80MHz)
//T(Conversion) =                   615~675ns       continue for 50 or 54 clock periods(80MHz)
reg sdo_data;
always@(negedge i_clk or negedge i_rst_n)
	if(!i_rst_n)
		sdo_data <= 1'd0;
	else
		sdo_data <= i_SDO;


reg [6:0]CNV_cnt;
always@(posedge i_clk or negedge i_rst_n)
    if(!i_rst_n)    begin
        o_CNV   <= 1'd0;
        CNV_cnt <= 7'd0;
		o_flag <=1'b0;
        end
    else if(CNV_cnt == 7'd0)  begin
        o_CNV <= 1'd1;                  //CNV posedge, Transmit to ADC, it will last 1 peirod, then ADC return a BUSY_H  
        CNV_cnt <= CNV_cnt + 1'd1;
		o_flag <=1'b0;
        end
	else if(CNV_cnt == 7'd10)    begin
        o_CNV <= 1'd0;
        CNV_cnt <= CNV_cnt + 1'd1;
		o_flag <=1'b0;
        end
	else if(CNV_cnt >= 7'd89)    begin
        o_CNV <= 1'd0;
        CNV_cnt <= 7'd0;
		o_flag <=1'b1;
        end
	else begin
        CNV_cnt <= CNV_cnt +1'd1;
		o_flag <=1'b0;
        end
		  
reg	[19:0]   SDO_data_reg         ; 
reg	[4:0]    SDO_counter     ; 
reg          SDO_data_done   ; 
reg [3:0]    BUSY_state;
reg ready;
		  
always@(negedge i_clk or negedge i_rst_n)
    if (!i_rst_n)  
	    begin
        BUSY_state <= 4'd0;
        ready <= 1'd1;
        SDO_data_reg <= 20'd0;
        SDO_data_done <= 1'b0;
        SDO_counter <= 5'd0;
        end 
    else if ((ready == 1'd1)&&(SDO_counter == 5'd0))   
	    begin                     //Wait for rest 62ns
        BUSY_state <= {BUSY_state[2:0],i_BUSY};         //ADC will Transmit BUSY_L after conversion automatically
        if(BUSY_state == 4'd12)                         //4'd12 = 1100, from now on These Data will Reset
		     ready <=1'd0;
			 SDO_data_done <= 1'b0 ;
    		 SDO_data_reg	   <= 20'b0;                
    		 SDO_counter   <= 5'b0 ;			 
		 end
	else if ((ready == 1'd0)&&(SDO_counter <= 5'd19))
	    begin
        BUSY_state <= {BUSY_state[2:0],i_BUSY};		   //Receive BUSY_H from ADC, as well as Serial Data
        SDO_data_reg <= {SDO_data_reg[18:0], sdo_data}; //After 20 clock periods, Serial Data Received Finished, Wait for rest 62ns
        SDO_counter <= SDO_counter + 1'b1;		
        if(SDO_counter == 5'd19)
        ready <= 1'd1;
		end
	else if(SDO_counter <= 5'd24)
	    begin
	    SDO_data_reg <= {SDO_data_reg[18:0], sdo_data}; //After 20 clock periods, Serial Data Received Finished, Wait for rest 62ns
	    SDO_counter <= SDO_counter + 1'b1;
	    end
	else
        begin             		            
            SDO_data_done <= 1'b1;  //Meanwhile Data will Transmit into FPGA
            SDO_counter   <= 5'b0 ;			
        end 

assign o_SCK = ready|i_clk;  

//output data          		
always@(posedge i_clk or negedge i_rst_n ) 
    if(!i_rst_n)
        o_AD_data_syn <= 0;
    else  
        o_AD_data_syn <= SDO_data_done   ;
        
always@(posedge i_clk or negedge i_rst_n) 
    if(!i_rst_n)    
        o_AD_data <= 20'd0;
    else if(SDO_data_done)  begin
        o_AD_data <= SDO_data_reg ;	
        end
    else begin
        o_AD_data <= o_AD_data    ;
        end

//over load
always@(posedge i_clk or negedge i_rst_n )
    if(!i_rst_n) 
    		o_AD_Overload <= 1'b0;
    else if ((o_AD_data==20'b1000_0000_0000_0000_0000)||(o_AD_data==20'b0111_1111_1111_1111_1111))  
    		o_AD_Overload <= 1'b1;
	else	o_AD_Overload <= 1'b0; 


        
endmodule
