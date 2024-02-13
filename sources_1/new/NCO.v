`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2018/12/19 18:09:50
// Design Name: 
// Module Name: NCO
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


module NCO(
	input					i_clk_250M							,
	input					i_rst_n								,		
	input [31:0]  			i_Vx_Vr_phase_inc		,		//DDS Frequency Control, Set by USER
	input [31:0]  			i_Vx_phase_mod  		,		//DDS Frequency Control, Set by USER
	input [31:0]  			i_Vr_phase_mod				, 		//DDS Phase Control, Set by USER
	input [31:0]			i_Vmod_phase_inc_1k8				,		//DDS Phase Control, Set by Algorithm	
	input [31:0]			i_Vmod_phase_mod_1k8				,		//DDS Phase Control, Set by Algorithm	
	input [31:0]			i_Vref_phase_inc				,		//DDS Phase Control, Set by Algorithm	
	input [31:0]			i_Vref_phase_mod				,		//DDS Phase Control, Set by Algorithm	
	
	output reg				o_nco_psd_flag					,
	//output reg 			o_fb_digital						,
	output reg signed [15:0]		o_Vx_sin						,
	output reg signed [15:0]		o_Vmod_sin				,
	output reg signed	[15:0]		o_Vr_sin						,	
	output reg signed [15:0]		o_Vref_sin					,
	output reg signed [15:0]		o_Vref_cos
    );
	
	/************************************		Generate the DDS	*******************************************/	
reg [31:0] Vx_Vr_phase_inc	;
reg [31:0] Vx_phase_mod		;
reg [31:0] Vr_phase_mod		;
reg [31:0] Vmod_phase_inc_1k8			;
reg [31:0] Vmod_phase_mod_1k8			;
reg [31:0] Vref_phase_inc	;
reg [31:0] Vref_phase_mod	;

always@(posedge i_clk_250M or negedge i_rst_n)
	if(!i_rst_n)	begin
		Vx_Vr_phase_inc				<= 32'd0;
		Vx_phase_mod					<= 32'd0;
		Vr_phase_mod					<= 32'd0;
		Vmod_phase_inc_1k8		<= 32'd0;
		Vmod_phase_mod_1k8	<= 32'd0;			
		Vref_phase_inc					<= 32'd0;
		Vref_phase_mod				<= 32'd0;
		end
	else	begin
		Vx_Vr_phase_inc				<= i_Vx_Vr_phase_inc	;
		Vx_phase_mod					<= i_Vx_phase_mod  	;
		Vr_phase_mod					<= i_Vr_phase_mod		;
		Vmod_phase_inc_1k8		<= i_Vmod_phase_inc_1k8	;
		Vmod_phase_mod_1k8	<= i_Vmod_phase_mod_1k8;			
		Vref_phase_inc					<= i_Vref_phase_inc;
		Vref_phase_mod				<= i_Vref_phase_mod;
		end
		
wire [31:0] Vx_data;
wire [31:0] Vr_data;
wire [31:0] Vmod_data_1k8;		
(* dont_touch = "true" *)wire [31:0] Vref_data;
		
DDS dds_Vx(
	.aclk(i_clk_250M),
	.aresetn(i_rst_n),
	.s_axis_phase_tvalid(1'b1),
	.s_axis_phase_tdata(Vx_phase_mod),
	.s_axis_config_tvalid(1'b1),
	.s_axis_config_tdata(Vx_Vr_phase_inc),
	.m_axis_data_tvalid(),
	.m_axis_data_tdata(Vx_data)
);	

DDS dds_Vr(
	.aclk(i_clk_250M),
	.aresetn(i_rst_n),
	.s_axis_phase_tvalid(1'b1),
	.s_axis_phase_tdata(Vr_phase_mod),
	.s_axis_config_tvalid(1'b1),
	.s_axis_config_tdata(Vx_Vr_phase_inc),
	.m_axis_data_tvalid(),
	.m_axis_data_tdata(Vr_data)
);	
	
DDS dds_Vmod(
	.aclk(i_clk_250M),
	.aresetn(i_rst_n),
	.s_axis_phase_tvalid(1'b1),
	.s_axis_phase_tdata(Vmod_phase_mod_1k8),
	.s_axis_config_tvalid(1'b1),
	.s_axis_config_tdata(Vmod_phase_inc_1k8),
	.m_axis_data_tvalid(),
	.m_axis_data_tdata(Vmod_data_1k8)
);		
		
DDS dds_Vref(
	.aclk(i_clk_250M),
	.aresetn(i_rst_n),
	.s_axis_phase_tvalid(1'b1),
	.s_axis_phase_tdata(Vref_phase_mod),
	.s_axis_config_tvalid(1'b1),
	.s_axis_config_tdata(Vref_phase_inc),
	.m_axis_data_tvalid(),
	.m_axis_data_tdata(Vref_data)
);				
	
/**************************************     output data    *****************************************/		
reg [15:0] Vx_sin			;	
reg [15:0] Vr_sin			;	
reg [15:0] Vmod_sin	;	
reg [15:0] Vmod_cos	;	
(* dont_touch = "true" *)reg [15:0] Vref_sin		;	
(* dont_touch = "true" *)reg [15:0] Vref_cos		;	

always@(posedge i_clk_250M or negedge i_rst_n)
	if(!i_rst_n)	begin
		Vx_sin		<= 16'd0;			
		Vr_sin		<= 16'd0;
		end
	else	begin
		Vx_sin		<= Vx_data[31:16];		
		Vr_sin		<= Vr_data[31:16];
		end

always@(posedge i_clk_250M or negedge i_rst_n)
	if(!i_rst_n)	begin		
		Vmod_sin	<= 16'd0;
		Vmod_cos	<= 16'd0;
		Vref_sin		<= 16'd0;
		Vref_cos		<= 16'd0;
		end
	else	begin
		Vmod_sin	<= Vmod_data_1k8[31:16];
		Vmod_cos	<= Vmod_data_1k8[15:0];
		Vref_sin		<= Vref_data[31:16];
		Vref_cos		<= Vref_data[15:0];
		end

/**************************************     output data for AD9783  *****************************************/	
always@(posedge i_clk_250M or negedge i_rst_n)
begin
	if(!i_rst_n)	begin
		o_Vx_sin		<= 16'd0;
	    o_Vmod_sin	<= 16'd0;	
	    o_Vr_sin			<= 16'd0;
		end
	else if(i_Vmod_phase_inc_1k8 !== 32'd0)	begin
		o_Vx_sin		<= Vx_sin;
	    o_Vmod_sin	<= Vmod_sin;	
	    o_Vr_sin			<= Vr_sin;		
		end	
	else	begin
		o_Vx_sin		<= Vx_sin;
	    o_Vmod_sin	<= 16'h7FFF;	
	    o_Vr_sin			<= Vr_sin;		
		end	
end	

/**************************************     output data for PSD   *****************************************/	
always@(posedge i_clk_250M or negedge i_rst_n)
	if(!i_rst_n)	begin
		o_Vref_sin <= 16'd0;
	    o_Vref_cos <= 16'd0;	
		end	
	else	begin
		o_Vref_sin <= Vref_sin;
	    o_Vref_cos <= Vref_cos;	
		end	

reg 	[7:0] cnt;	
always@(posedge i_clk_250M or negedge i_rst_n)
begin
	if(!i_rst_n)	begin
		o_nco_psd_flag <= 1'b0;
		cnt = 8'd0;
		end
	else if(cnt !== 8'd249)	begin
		o_nco_psd_flag <= 1'b0;
		cnt <= cnt + 8'd1;
		end
	else	begin
		o_nco_psd_flag <= 1'b1;
		cnt <= 8'd0;
		end
	end		

endmodule