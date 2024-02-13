/*
 * UART.c
 *
 *  Created on: 2018Äê12ÔÂ30ÈÕ
 *      Author: Administrator
 */

#include <stdlib.h>
#include <stdio.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xparameters_ps.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xbasic_types.h"
#include "UART.h"
#include "xgpiops.h"
#include "sleep.h"
#include "ARM_interface.h"

#define	DDS3_PWR_DOWN_valid		XGpioPs_WritePin(&psGpioInstancePtr,75,1)
//Function
void DDS2_Write();
void DDS3_Write();
void DDS1_Write();

//GPIO
extern XGpioPs psGpioInstancePtr;
#define	DDS_PWR_DOWN_valid		XGpioPs_WritePin(&psGpioInstancePtr,90,1)
#define	DDS_PWR_DOWN_invalid	XGpioPs_WritePin(&psGpioInstancePtr,90,0)
#define	DDS3_RST_valid		XGpioPs_WritePin(&psGpioInstancePtr, 74, 1)
#define	DDS3_RST_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 74, 0)

//UART
extern XUartPs Uart_Ps;
extern XScuGic InterruptController;	/* Instance of the Interrupt Controller */
extern u8 Send_Buffer [];	/* Buffer for Transmitting Data */
extern u8 Recv_Buffer[];	/* Buffer for Receiving Data */
extern u8 Received_Count;//END POINT
extern int TotalSentCount;
extern int TotalSentCount;
u8 CMD_Buffer[256] ="";

//the variable of DDS,AD9508 and AD9552 
volatile Xfloat32 Vx_Vr_freq = 1000.0;
volatile Xfloat32 Vmod_freq = 1000.0;
extern double DDS_output_gain;
extern double DDS3_output_gain;
extern double AD9552_Freq_out;
extern int AD9508_DIVIDER;


extern unsigned char DDS1_FTW0[5];
extern unsigned char DDS2_FTW0[5];
extern unsigned char DDS3_FTW0[5];

extern unsigned char DDS1_ASF[3];
extern unsigned char DDS2_ASF[3];
extern unsigned char DDS3_ASF[3];

extern unsigned char DDS1_POW0[3];
extern unsigned char DDS2_POW0[3];
extern unsigned char DDS3_POW0[3];


extern unsigned char DDS3_CFR1[5];


extern u32 DDS1_DDS2_freq_tunning_word;
extern u32 DDS3_freq_tunning_word;

extern u32 DDS1_amplitude_factor;
extern u32 DDS2_amplitude_factor;
extern u32 DDS3_amplitude_factor;

extern u32 DDS1_phase_factor;
extern u32 DDS2_phase_factor;
extern u32 DDS3_phase_factor;






/***** those variable from former UART.c, not used now *****/
u32 Vx_Vr_phase_inc;
u32 Vx_gain;
u32 Vx_phase_mod;
u32 Vr_gain;
u32 Vr_phase_mod;
u32 Vmod_phase_inc;
u32 Vmod_gain;
u32 Vmod_phase_mod;
u32 Vref_phase_inc;
u32 Vref_phase_mod;
u32 DGP_control;
volatile u32 IA_CTRL_Analog;
volatile u8 CTRL_Rr_Sel;
u32 Vx_Vr_gain;
extern double RAD2DEG;

volatile u8 sequence_1;
volatile u8 sequence_2;
volatile u16 Rr;
volatile u8 FUNC_Term_Temp;

volatile u8 Master_switch_flag = 0;

volatile u8	DAC_Output_Temp;
volatile u8 DAC_Phase_Temp ;
volatile Xfloat32 AUX_DA1_float = 0.0;
volatile Xfloat32 AUX_DA2_float = 0.0;
volatile Xfloat32 AUX_DA3_float = 0.0;
volatile Xfloat32 AUX_DA4_float = 0.0;
volatile Xfloat32 Vx_phase_mod_float = 0.0;
volatile Xfloat32 Vr_phase_mod_float = 0.0;
volatile Xfloat32 Vmod_phase_mod_float = 0.0;
volatile Xfloat32 Vref_phase_mod_float = 0.0;
volatile unsigned char freq_mode_flag = 0;
volatile u8 freq_mixed_flag = 0;
volatile int DFT_delay_us = 1200;//set initial dft_delay_us 1200


/* freq sweep, wgq add */
volatile Xfloat32 freq_sweep_array[SWEEP_MAXLINE] = {0.0};
// volatile Xfloat32 freq_sweep_array[SWEEP_MAXLINE] = {1000.0,1500.0,5000.0,10000.0,1000000.0,5000000.0,10000000.0,0.0};
// volatile Xfloat32 freq_sweep_array[SWEEP_MAXLINE] = {10001.0,15000.0,100000.0,500000.0,1000000.0,5000000.0,10000000.0,10000.0,0.0};
volatile Xfloat32 amplitude_sweep_array[SWEEP_MAXLINE] = {0.0};
// volatile Xfloat32 amplitude_sweep_array[SWEEP_MAXLINE] = {0.25,0.5,0.75,1.0,1.25,1.5,1.75,2.0,2.25,2.5,2.75,3.0,0.0};
extern u32 sweep_freq_times;
extern Xfloat32 *amp_sweep_pointer_temp;
extern u8 Workspace;
volatile u8 sweep_freq_conduct_temp = 0;
volatile u8 sweep_freq_type = 0;
volatile Xfloat32 sweep_freq_start = 0;
volatile Xfloat32 sweep_freq_stop = 0;
volatile u32 sweep_freq_points = 0;
volatile u8 sweep_amp_conduct_temp = 0;
volatile u8 sweep_amp_type = 0;
volatile Xfloat32 sweep_amp_start = 0;
volatile Xfloat32 sweep_amp_stop = 0;
volatile u32 sweep_amp_points = 0;

volatile u32 point_of_period = 0;
volatile u8  cycles = 1;
volatile u32 Sweep_Freq_Time_count = 0;
volatile u32 Sweep_Freq_Step_count = 0;
volatile Xfloat32 Sweep_Freq_Freq_temp = 1000;
unsigned long long Sweep_Freq_Freq_data = 0;
volatile u32 Global_GainTC_Sensitivity = 25; //100mv
volatile u8 Global_GainTC_TimeConstant = 10; //300ms
volatile u8 Global_GainTC_Fliter = 2; //12db/oct
volatile u8 Global_GainTC_Synchronous = 1; //off

/*--------  for measure Rr_X, Rr_Y  --------*/
extern long double Rr_X;
extern long double Rr_Y;

//*******************the items from FPGA to ARM***************************//
extern double PSD_A_X_display;
extern double PSD_A_Y_display;
extern double PSD_B_X_display;
extern double PSD_B_Y_display;
extern double PSD_A_R;
extern double PSD_B_R;
extern double PSD_A_Vpp;
extern double PSD_B_Vpp;
extern double PSD_A_Xita;
extern double PSD_B_Xita;
extern double DFT_A_R;
extern double DFT_B_R;
extern double DFT_A_Vpp;
extern double DFT_B_Vpp;
extern double DFT_A_Xita;
extern double DFT_B_Xita;
extern double DFT_A_X;
extern double DFT_B_X;
extern double DFT_A_Y;
extern double DFT_B_Y;
extern double DFT_A_X_display;
extern double DFT_B_X_display;
extern double DFT_A_Y_display;
extern double DFT_B_Y_display;

extern unsigned long long DFT_A_X_h, DFT_B_X_h, DFT_A_Y_h, DFT_B_Y_h, DFT_A_X_l, DFT_B_X_l, DFT_A_Y_l, DFT_B_Y_l;

extern unsigned char PSD_aver_flag;
extern unsigned long long PSD_aver_cnt;
extern double high_freq_moving_aver_result_X;
extern double high_freq_moving_aver_data_X_1st;
extern double high_freq_moving_aver_sum_X;
extern u8 high_freq_moving_aver_cnt_X;
extern u8 high_freq_moving_aver_cnt_Y;
extern u8 low_freq_moving_aver_cnt_X;
extern u8 low_freq_moving_aver_cnt_Y;
extern u8 low_freq_Cal_rep_state;
extern u8 high_freq_Cal_repetitive_state;
extern double high_freq_DUT_X;
extern double low_freq_DUT_X;
extern double low_freq_DUT_Y;
volatile Xfloat32 Sweep_mode_Vx_Vr_freq_display = 0.0;//because Vx_Vr_freq will be set to the freq of next sweep in freq sweep mode, so the current freq must be storged in Sweep_mode_Vx_Vr_freq_display in advance.
volatile Xfloat32 Sweep_mode_Vx_amp_display = 0.0;
extern u8 Measure_Mode;

static u8 Parameter_Buff[35] = {0};
volatile u16 Command = 0;
volatile u8 Fifo_start = 0;
volatile u8 Fifo_end = 0;
volatile u8 Fifo_end_temp = 0;
volatile u8 Fifo_cnt = 0;
volatile u16 Fifo_Len = 0;
volatile u8 Command_start = 0;
volatile u16 Command_End = 0;
volatile u8 Command_cnt = 0;
volatile u8 Command_len = 0;

volatile u32 IDN_Char_Value = 6030000;

volatile Xuint8 Global_RefPhase_Source = EXTERNAL;
volatile Xuint8 Global_RefPhase_Slope = TTL;
volatile Xfloat32 Global_RefPhase_Phase = 0.0;
volatile Xfloat32 Internal_Freqvalue = 1000;
unsigned long long FPGA_cordic_inc_reg = 0;
unsigned int Sync_Sample_intenal_reg = 0;
Xuint8 RefPhase_Phase_Arg[8] = {'+','0',0};
Xuint8 RefPhase_Freq_Arg[10] = {'0','.','5',0};
Xuint8 RefPhase_SineOutput_Arg[6] = {'1',0};
Xuint8 EnterBitCount = 1;
Xuint8 isPoint = 0;
Xuint8 isE = 0;
Xuint8 isSIGN = 0;

/**/
//extern double Vx_X_single_0 ;
//extern double Vx_Y_single_0 ;
//extern double Vr_X_single_0 ;
//extern double Vr_Y_single_0 ;
//extern double Lp_X_single_0 ;
//extern double Lp_Y_single_0 ;
//extern double Vx_X_single_1 ;
//extern double Vx_Y_single_1 ;
//extern double Lp_X_single_1 ;
//extern double Lp_Y_single_1 ;
//extern double Vr_X_single_1 ;
//extern double Vr_Y_single_1 ;
//extern double Lp_X_single_2 ;
//extern double Lp_Y_single_2 ;
//extern double DUT_single_X;
//extern double DUT_single_Y;
extern double Vx_X_single;
extern double Vx_Y_single;
extern double Vr_X_single;
extern double Vr_Y_single;
extern double Lp_X_single;
extern double Lp_Y_single;
extern double DUT_X;
extern double DUT_Y;

extern double A_single;
extern double B_single;
extern double C_single;
extern double D_single;

extern double DUT_display_fitting_1;
extern double DUT_display_fitting_2;
extern double DUT_display_1;
extern double DUT_display_2;
extern u8 Data_interact_ON_flag;

extern unsigned char Phase_modulated_balanced_done;
extern unsigned char Amplitude_modulated_balanced_done;
extern u8 Phase_modulated_state;
extern u8 Amplitude_modulated_state;
extern u8 balanced_cnt;
extern Xfloat64 Phase_modulated_range;
extern double Balance_Vr_phase_mod;
extern Xfloat64 Lp_value_balanced;
extern u8 test_flag;
extern double phase_modulated_Lp_value_0;
extern double phase_modulated_Lp_value_1;
extern double phase_modulated_Lp_value_2;

extern double Vr_amplitude_modulated_min;
extern double Vr_amplitude_modulated_max;
extern u32 Balance_Vr_gain;

extern double DFT_Lp_X_val[10];
extern double DFT_Lp_Y_val[10];
extern double DFT_Vr_X_val[10];
extern double DFT_Vr_Y_val[10];
extern double DFT_Vx_X_val[10];
extern double DFT_Vx_Y_val[10];

extern double PSD_Lp_X_val[10];
extern double PSD_Lp_Y_val[10];
extern double PSD_Vr_X_val[10];
extern double PSD_Vr_Y_val[10];
extern double PSD_Vx_X_val[10];
extern double PSD_Vx_Y_val[10];


extern double Vr_adj_X_cal;
extern double Vr_adj_Y_cal;
extern double Vr_adj_X_mod;
extern double Vr_adj_Y_mod;

extern double coef_matrix_a_X[5];
extern double coef_matrix_a_Y[5];
extern double coef_matrix_b_X[5];
extern double coef_matrix_b_Y[5];

extern double temp_mul_order_X[5];
extern double temp_mul_order_Y[5];

extern double Vr_adj_Vpp_mod_1st;
extern double Vr_adj_phase_mod_1st;
extern double Vr_adj_Vpp_mod_2nd;
extern double Vr_adj_phase_mod_2nd;

volatile int PSD_delay_us;

extern double Vr_set_X_1st[5];
extern double Vr_set_Y_1st[5];

extern double FPGA_Lp_Vpp_1st_balanced;
extern double balanced_1st_dir;
extern double Vr_adj_phase_start_2nd;
extern double Vr_adj_phase_start_1st;
extern double Vr_adj_phase_start_delta;
extern double Vr_adj_Vpp_start_2nd;
extern double Vr_step_X;
extern double Vr_step_Y;

extern double Vr_adj_Vpp_cal_2nd;
extern double Vr_adj_X_mod_2nd;
extern double Vr_adj_Y_mod_2nd;
extern double Vr_adj_X_cal_2nd;
extern double Vr_adj_Y_cal_2nd;

extern double Lp_X_start_1st;
extern double Lp_Y_start_1st;

extern double DFT_Lp_Vpp_1st_balanced;
extern double Vx_Vpp, Vr_Vpp, Lp_Vpp;
extern double Vx_Vrms, Vr_Vrms, Lp_Vrms;
extern double Vx_Xita, Vr_Xita, Lp_Xita;
extern double Cal_PSD_A_Vrms;
extern double Cal_PSD_B_Vrms;
//	Tony add
extern double moving_aver_result_X;
extern double moving_aver_result_Y;

extern Xfloat32 *freq_sweep_pointer_temp;

//about Stray_Z
volatile double Stray_Z;
volatile double Stray_fitting_coefficient_A = 0.0000100864;
volatile double Stray_fitting_coefficient_B = 0.00000000140018;
extern u8 Enable_Auto_Rr_flag;//1 enable Auto Rr, 0 disable Auto Rr.
extern u8 Measure_Term;
extern Xfloat32 Z_Cb_module;






extern double debug_flag;






//XUartPs Uart_PS;
//XScuGic InterruptController;	/* Instance of the Interrupt Controller */

void Cal_single_low_freq_initialize(void);
extern void Cal_initialize(void);
extern void Repeat_balance_cal_initialize(void);
void Fill_freq_sweep_array_start_sweep(u8 sweep_freq_type_temp, Xfloat32 sweep_freq_start_temp, Xfloat32 sweep_freq_stop_temp, u32 sweep_freq_points_temp);
void Fill_amp_sweep_array_start_sweep(u8 sweep_amp_type_temp, Xfloat32 sweep_amp_start_temp, Xfloat32 sweep_amp_stop_temp, u32 sweep_amp_points_temp);


void UART(void)
{
	UART_main();
}
void UART_main(void)
{
	u8 CMD_Buffer_Cnt = 0;
	u8 CMD_Fifo_start = 0;
	u8 CMD_Fifo_end = 0;
	if(Recv_Buffer[Received_Count] == 0x0D || Recv_Buffer[Received_Count] == ';')
	{
	if(Fifo_start != Received_Count)
	{
		Fifo_end = Received_Count;
		if(Fifo_start < Fifo_end)
		{
			CMD_Fifo_end = Fifo_end - Fifo_start;
			for(CMD_Buffer_Cnt=0; CMD_Buffer_Cnt+Fifo_start <= Fifo_end; CMD_Buffer_Cnt++)
				CMD_Buffer[CMD_Buffer_Cnt]=	Recv_Buffer[CMD_Buffer_Cnt+Fifo_start];
		}
		else if(Fifo_start > Fifo_end)
		{
			CMD_Fifo_end = Fifo_end+256-Fifo_start;      //the cmd length
			Fifo_end_temp=255-Fifo_start+1;              //the start of the next length
			for(CMD_Buffer_Cnt=0; CMD_Buffer_Cnt+Fifo_start <= 255; CMD_Buffer_Cnt++)
			{
				CMD_Buffer[CMD_Buffer_Cnt]=	Recv_Buffer[CMD_Buffer_Cnt+Fifo_start];
			}
			for(CMD_Buffer_Cnt=0; CMD_Buffer_Cnt <= Fifo_end; CMD_Buffer_Cnt++)
			{
				CMD_Buffer[CMD_Buffer_Cnt + Fifo_end_temp ]= Recv_Buffer[CMD_Buffer_Cnt];
			}
		}
		Fifo_start = Fifo_end;             //execute the cmd end the Fifo_start
		if(CMD_Fifo_end>=4) //*IDN?; at least 6 byte;eg.0,1,2,3,4,5;
		{
			for(CMD_Buffer_Cnt=CMD_Fifo_start; CMD_Buffer_Cnt<=CMD_Fifo_end; CMD_Buffer_Cnt++)
			{
				if(CMD_Buffer[CMD_Buffer_Cnt] == 0x0D || CMD_Buffer[CMD_Buffer_Cnt] == ';')  //there is a CMD in it,then get it.
				{
					if(CMD_Buffer_Cnt-CMD_Fifo_start>=4)   //a CMD is length more than 5 byte
					{
						Command_start = CMD_Fifo_start  ;
						Command_End   = CMD_Buffer_Cnt    ;
						for(Command_cnt = Command_start; Command_cnt<=Command_End ; Command_cnt++ )      //stop point is Command_End
						{
							if((CMD_Buffer[Command_cnt]>='A' && CMD_Buffer[Command_cnt]<='Z') || CMD_Buffer[Command_cnt]=='*')  //start if com start? abandom other char
							{
								if(Command_End-Command_cnt>=4)   //get CMD
								{
									if(CMD_Buffer[Command_cnt+4]>='A' && CMD_Buffer[Command_cnt+4]<='Z' )  //start if com start? abandom other char
										continue;
									else
									{
										UART_Instrution_Cmp(Command_cnt,Command_End);
										break;//after get the CMD
									}
								}
								else
									break;    //no CMD
							}
						}
					}
					if(CMD_Buffer_Cnt==255)
					{
						CMD_Fifo_start = 0;
						break;
					}
					else
						CMD_Fifo_start = CMD_Buffer_Cnt+1;
				}
			}
		//				CMD_Fifo_start = CMD_Fifo_end;   //no mather there is CMD or not
		}
	}

	}
}

void UART_Instrution_Cmp(u16 Command_index_start,u16 Command_index_end)
{
	u16 Instrution_index = 0 ;
	u16 Parameter_index = 0 ;
	u8  Para_Index = 0 ;
	u8  Instruction_Buff[5]={0};

	for(Instrution_index=Command_index_start  ,Parameter_index=0; Instrution_index<=Command_index_start+3;   Instrution_index++)
	{
		Instruction_Buff[Parameter_index++] = CMD_Buffer[Instrution_index];
	}
	for(Instrution_index=Command_index_start+4,Parameter_index=0; Instrution_index<Command_index_end; Instrution_index++)
	{
		Parameter_Buff[Parameter_index++]   = CMD_Buffer[Instrution_index];
	}
	Command_len = Command_index_end- Command_index_start +1;

	switch(Instruction_Buff[0])
	{
		case 'A':
			switch(Instruction_Buff[1])
			{
				case 'G' :  if(Instruction_Buff[2] == 'A' && Instruction_Buff[3] == 'N' ){Command = AGAN ; Instrution_Parameter_Parse();} break;
				case 'R' :  if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'V' ){Command = ARSV ; Instrution_Parameter_Parse();} break;
				case 'P' :	if(Instruction_Buff[2] == 'H' && Instruction_Buff[3] == 'S' ){Command = APHS ; Instrution_Parameter_Parse();} break;
				case 'S' :	if(Instruction_Buff[2] == 'C' && Instruction_Buff[3] == 'L' ){Command = ASCL ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'E':
				if(Instruction_Buff[1] == 'Q')
				{
					if(Instruction_Buff[2] == 'C' && Instruction_Buff[3] == 'D' ){Command = EQCD ;Instrution_Parameter_Parse();}
					else if(Instruction_Buff[2] == 'C' && Instruction_Buff[3] == 'S' ){Command = EQCS ;Instrution_Parameter_Parse();}
				}
				break;
		case 'F':
			switch(Instruction_Buff[1])
			{
				case 'M' :  if(Instruction_Buff[2] == 'O' && Instruction_Buff[3] == 'D' ){Command = FMOD ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'U' && Instruction_Buff[3] == 'L' ){Command = FMUL ; Instrution_Parameter_Parse();} break;
				case 'R' :  if(Instruction_Buff[2] == 'E' && Instruction_Buff[3] == 'Q' ){Command = FREQ ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'A' && Instruction_Buff[3] == 'M' ){Command = FRAM ; Instrution_Parameter_Parse();} break;
				case 'P' :	if(Instruction_Buff[2] == 'O' && Instruction_Buff[3] == 'P' ){Command = FPOP ; Instrution_Parameter_Parse();} break;
				case 'U' :	if(Instruction_Buff[2] == 'N' && Instruction_Buff[3] == 'C' ){Command = FUNC ; Instrution_Parameter_Parse();} break;
				default: break;
			}
		case 'G':
			if(Instruction_Buff[1] == 'N' && Instruction_Buff[2] == 'O' && Instruction_Buff[3] == 'V' ){Command = GNOV ; Instrution_Parameter_Parse();} break;
		case 'H' :	//"HARM"
			if(Instruction_Buff[1] == 'A' && Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'M' ){Command = HARM ; Instrution_Parameter_Parse();} break;

		case 'I' :	//"ISRC","IGND","ICPL","ILIN","INOV"
			switch(Instruction_Buff[1])
			{
				case 'S' :  if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'C' ){Command = ISRC ; Instrution_Parameter_Parse();} break;
				case 'G' :  if(Instruction_Buff[2] == 'N' && Instruction_Buff[3] == 'D' ){Command = IGND ; Instrution_Parameter_Parse();} break;
				case 'C' :	if(Instruction_Buff[2] == 'P' && Instruction_Buff[3] == 'L' ){Command = ICPL ; Instrution_Parameter_Parse();} break;
				case 'L' :	if(Instruction_Buff[2] == 'I' && Instruction_Buff[3] == 'N' ){Command = ILIN ; Instrution_Parameter_Parse();} break;
				case 'N' :	if(Instruction_Buff[2] == 'O' && Instruction_Buff[3] == 'V' ){Command = INOV ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'O' :	//"OUTP","OFLT","OFSL","OEXP","OAUX"
			switch(Instruction_Buff[1])
			{
				case 'U' :  if(Instruction_Buff[2] == 'T' && Instruction_Buff[3] == 'P' ){Command = OUTP ; Instrution_Parameter_Parse();} break;
				case 'F' :  if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'T' ){Command = OFLT ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'L' ){Command = OFSL ; Instrution_Parameter_Parse();} break;
				case 'E' :	if(Instruction_Buff[2] == 'X' && Instruction_Buff[3] == 'P' ){Command = OEXP ; Instrution_Parameter_Parse();} break;
				case 'A' :	if(Instruction_Buff[2] == 'U' && Instruction_Buff[3] == 'X' ){Command = OAUX ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'P' :  //"PHAS","PAUS"
			switch(Instruction_Buff[1])
			{
				case 'H' :  if(Instruction_Buff[2] == 'A' && Instruction_Buff[3] == 'S' ){Command = PHAS ; Instrution_Parameter_Parse();} break;
				case 'A' :  if(Instruction_Buff[2] == 'U' && Instruction_Buff[3] == 'S' ){Command = PAUS ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'R' :	//"RALL","RSLP","RMOD","RSET","REST"
			switch(Instruction_Buff[1])
			{
				case 'A' :  if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'L' ){Command = RALL ; Instrution_Parameter_Parse();} break;
				case 'S' :  if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'P' ){Command = RSLP ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'E' && Instruction_Buff[3] == 'T' ){Command = RSET ; Instrution_Parameter_Parse();} break;
				case 'M' :	if(Instruction_Buff[2] == 'O' && Instruction_Buff[3] == 'D' ){Command = RMOD ; Instrution_Parameter_Parse();} break;
				case 'E' :	if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'T' ){Command = REST ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'S' :	//"SNAP","SENS","SLVL","SYNC","SSET","SRAT","STRD","SPTS","SWPT","SLLM","SULM","SSLL","SSLG","STLM","SWRM",
					//"SWVT","SVLL","SVUL","SVSL","SVSG","SVTM","SVRM","SLEN","SSLE","STRG","SPRM","SPED"
			switch(Instruction_Buff[1])
			{
							//"SFMD", "SFRE" for setting the freq of Vmod and Vref
				case 'F' :  if(Instruction_Buff[2] == 'M' && Instruction_Buff[3] == 'D' ){Command = SFMD ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'E' ){Command = SFRE ; Instrution_Parameter_Parse();} break;

				case 'N' :  if(Instruction_Buff[2] == 'A' && Instruction_Buff[3] == 'P' ){Command = SNAP ; Instrution_Parameter_Parse();} break;
				case 'E' :  if(Instruction_Buff[2] == 'N' && Instruction_Buff[3] == 'S' ){Command = SENS ; Instrution_Parameter_Parse();} break;
				case 'L' :	if(Instruction_Buff[2] == 'V' && Instruction_Buff[3] == 'L' ){Command = SLVL ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'M' ){Command = SLLM ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'E' && Instruction_Buff[3] == 'N' ){Command = SLEN ; Instrution_Parameter_Parse();} break;
				case 'Y' :	if(Instruction_Buff[2] == 'N' && Instruction_Buff[3] == 'C' ){Command = SYNC ; Instrution_Parameter_Parse();} break;
				case 'S' :  if(Instruction_Buff[2] == 'E' && Instruction_Buff[3] == 'T' ){Command = SSET ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'L' ){Command = SSLL ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'G' ){Command = SSLG ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'E' ){Command = SSLE ; Instrution_Parameter_Parse();} break;
				case 'R' :  if(Instruction_Buff[2] == 'A' && Instruction_Buff[3] == 'T' ){Command = SRAT ; Instrution_Parameter_Parse();} break;
				case 'T' :	if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'D' ){Command = STRD ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'G' ){Command = STRG ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'M' ){Command = STLM ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'H' && Instruction_Buff[3] == 'R' ){Command = STHR ; Instrution_Parameter_Parse();} break;
				case 'P' :	if(Instruction_Buff[2] == 'T' && Instruction_Buff[3] == 'S' ){Command = SPTS ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'M' ){Command = SPRM ; Instrution_Parameter_Parse();}

							else if(Instruction_Buff[2] == 'H' && Instruction_Buff[3] == 'S' ){Command = SPHS ; Instrution_Parameter_Parse();}

							else if(Instruction_Buff[2] == 'E' && Instruction_Buff[3] == 'D' ){Command = SPED ; Instrution_Parameter_Parse();} break;
				case 'W' :	if(Instruction_Buff[2] == 'P' && Instruction_Buff[3] == 'T' ){Command = SWPT ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'M' ){Command = SWRM ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'V' && Instruction_Buff[3] == 'T' ){Command = SWVT ; Instrution_Parameter_Parse();} break;
				case 'U' :	if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'M' ){Command = SULM ; Instrution_Parameter_Parse();} break;
				case 'V' :	if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'L' ){Command = SVLL ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'U' && Instruction_Buff[3] == 'L' ){Command = SVUL ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'L' ){Command = SVSL ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'G' ){Command = SVSG ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'T' && Instruction_Buff[3] == 'M' ){Command = SVTM ; Instrution_Parameter_Parse();}
							else if(Instruction_Buff[2] == 'R' && Instruction_Buff[3] == 'M' ){Command = SVRM ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		case 'T' :	//"TRCA"
			if(Instruction_Buff[1] == 'R' && Instruction_Buff[2] == 'C' && Instruction_Buff[3] == 'A' ){Command = TRCA ; Instrution_Parameter_Parse();} break;
		case '*' :  //"*RST","*IDN"
			switch(Instruction_Buff[1])
			{
				case 'R' :  if(Instruction_Buff[2] == 'S' && Instruction_Buff[3] == 'T' ){Command = _RST ; Instrution_Parameter_Parse();} break;
				case 'I' :  if(Instruction_Buff[2] == 'D' && Instruction_Buff[3] == 'N' ){Command = _IDN ; Instrution_Parameter_Parse();} break;
				case 'P' :  if(Instruction_Buff[2] == 'L' && Instruction_Buff[3] == 'L' ){Command = _PLL ; Instrution_Parameter_Parse();} break;
				default: break;
			}
			break;
		default:  break;
	}
	for (Para_Index = 0; Para_Index < 35; Para_Index++)
	{
		Parameter_Buff[Para_Index] = 0;
	}
}

void Instrution_Parameter_Parse(void)
{
	u8 Para_Index = 0;
	u8 cmd_opt_type=0;
	u8 inquire_num=0;
	for(Para_Index=0;Para_Index<=Command_len-5;Para_Index++)
	{
		if(Parameter_Buff[Para_Index] == ' ')
		{
			continue;
		}
		else
		{
			if(Parameter_Buff[Para_Index] == '?')
			{
				cmd_opt_type=1;
				if(Command==HARM || Command== FPOP|| Command==OEXP || Command==OUTP ||Command == EQCD || Command== EQCS || Command== SSLE || Command== OAUX || Command== FRAM)
				{
					inquire_num = atoi((char *)&Parameter_Buff[Para_Index+1]);
				}
				else if(Command==SNAP || Command==TRCA)
				{
					inquire_num = Para_Index + 1;
				}
				break;
			}
			else
			{
				if(((Parameter_Buff[Para_Index] >= '0') && (Parameter_Buff[Para_Index] <= '9')) || (Parameter_Buff[Para_Index]=='-') || //
					(Parameter_Buff[Para_Index] == '+') || (Parameter_Buff[Para_Index] == 'U') || (Parameter_Buff[Para_Index] == 'L')) //'U' = 'U'NLOCK, 'L' = 'L'OCK
				{
					cmd_opt_type=2;
					break;
				}
				else
				{
					break;
				}
			}
		}
	}
    Execute_Instrution(Command,cmd_opt_type,inquire_num);
}


void QueryIntArg(u32 query_i_data, u8 EndChar)
{
	u8 output_value[10];
	sprintf((char *)output_value, "%d", query_i_data);
	Send_Data2PC_str((char *)output_value);
	if(EndChar==UEOS)
		Send_Data2PC(0x0D);
}


void QueryFloatArg(Xfloat32 query_f_data, u8 param_type, u8 EndChar)
{
	u8 output_value[14]={0};
	if(param_type==0)
		sprintf((char *)output_value, "%.2f",query_f_data);
	else if(param_type==1)
		sprintf((char *)output_value, "%.3f",query_f_data);
	else
		sprintf((char *)output_value, "%6.5e",query_f_data);
	Send_Data2PC_str((char *)output_value);
    if(EndChar==UEOS)
        Send_Data2PC(0x0D);
	else if(EndChar==UTOK)
		Send_Data2PC(',');
}

void Send_Data2PC(char data)
{
	int i;
	xil_printf("%c", data);
}

void Send_Data2PC_str(char *str)
{
	int Index = 0;
	int i;
	for (Index = 0; str[Index] != '\0'; Index++)
	{
		Send_Buffer [Index] = str[Index];
	}
	XUartPs_Send(&Uart_Ps, Send_Buffer , Index);
	while (1)
	{
		if (TotalSentCount == Index)
		{
			TotalSentCount = 0;
			Uart_Ps.SendBuffer.NextBytePtr = NULL;
			Uart_Ps.SendBuffer.RemainingBytes = 0U;
			Uart_Ps.SendBuffer.RequestedBytes = 0U;
			break;
		}
	}
}

void Execute_Instrution(u16 cmd_flag,u8 cmd_opt_type, u8 inquire)
{
	int i;

    u32 Option_Int_Value=0;
    u32 time_coefficient = 0xFFFFFF0F;
    u32 filter_slop = 0xFFFFFFF7;
    Xfloat32 Option_float_Value=0.0;
//    Xfloat32 Vx_Vr_freq = 1000.0;
	switch(cmd_flag)
	{
		case RALL:
		{
			if(cmd_opt_type==1)
			{
				
				//QueryFloatArg(PSD_aver_cnt,EXP5P,UTOK);
//				if(freq_mixed_flag)
//				{
//					QueryFloatArg(high_freq_DUT_X,EXP5P,UTOK);
//					QueryFloatArg(high_freq_DUT_Y,EXP5P,UTOK);
//					QueryFloatArg(FPGA_high_freq_DUT_X,EXP5P,UTOK);	//	Tony add
//					QueryFloatArg(FPGA_high_freq_DUT_Y,EXP5P,UTOK);	//	Tony add
//
//				}
//				else
//				{
//					QueryFloatArg(low_freq_DUT_X,EXP5P,UTOK);
//					QueryFloatArg(low_freq_DUT_Y,EXP5P,UTOK);
//				}

				//QueryFloatArg(Rr,EXP5P,UTOK);

//				if(freq_mixed_flag)
//				{
//					QueryFloatArg(DUT_high_freq_display_1,EXP5P,UTOK);
//					QueryFloatArg(DUT_high_freq_display_2,EXP5P,UTOK);
//					QueryFloatArg(FPGA_DUT_high_freq_display_1,EXP5P,UTOK);  	//	Tony add
//					QueryFloatArg(FPGA_DUT_high_freq_display_2,EXP5P,UTOK);  	//	Tony add
//				}
//				else
//				{
//					QueryFloatArg(DUT_low_freq_display_1,EXP5P,UTOK);
//					QueryFloatArg(DUT_low_freq_display_2,EXP5P,UTOK);
//				}

//				if(freq_mixed_flag)
//				{
//				QueryFloatArg(A_rep,EXP5P,UTOK);
//				QueryFloatArg(B_rep,EXP5P,UTOK);
//				QueryFloatArg(C_rep,EXP5P,UTOK);
//				QueryFloatArg(D_rep,EXP5P,UTOK);		//Rr
//				}
//				else
//				{
//					QueryFloatArg(A_single,EXP5P,UTOK);
//					QueryFloatArg(B_single,EXP5P,UTOK);
//					QueryFloatArg(C_single,EXP5P,UTOK);
//					QueryFloatArg(D_single,EXP5P,UTOK);		//Rr
//				}
//

//				QueryFloatArg(DFT_Lp_Vpp_1st_balanced,EXP5P,UTOK);	//item1
				QueryFloatArg(DUT_display_1,EXP5P,UTOK);	//item1
				QueryFloatArg(DUT_display_2,EXP5P,UTOK);			//item2

				// QueryFloatArg(Vr_adj_phase_start_2nd,EXP5P,UTOK);	//item3
				// QueryFloatArg(Vr_adj_phase_start_1st,EXP5P,UTOK);		//item4

				// QueryFloatArg(Vr_adj_phase_start_delta,EXP5P,UTOK);				//item5
				// QueryFloatArg(Vr_adj_Y_cal_2nd,EXP5P,UTOK);				//item6

				// QueryFloatArg(Vr_adj_Vpp_mod_2nd,EXP5P,UTOK);			//item7
				// QueryFloatArg(Vr_adj_phase_mod_2nd,EXP5P,UTOK);			//item8

				// QueryFloatArg(Vr_adj_Vpp_mod_1st,EXP5P,UTOK);			//item9
				// QueryFloatArg(Vr_adj_phase_mod_1st,EXP5P,UTOK);			//item10

				QueryFloatArg(Rr_X,EXP5P,UTOK);	//item3
				QueryFloatArg(Rr_Y,EXP5P,UTOK);		//item4

				QueryFloatArg(Z_Cb_module,EXP5P,UTOK);				//item5
				QueryFloatArg(Vx_Y_single,EXP5P,UTOK);				//item6

				QueryFloatArg(Lp_X_single,EXP5P,UTOK);			//item7
				QueryFloatArg(Lp_Y_single,EXP5P,UTOK);			//item8

				// QueryFloatArg(Vr_adj_Vpp_mod_1st,EXP5P,UTOK);			//item9
				// QueryFloatArg(Vr_adj_phase_mod_1st,EXP5P,UTOK);			//item10

				QueryFloatArg(Stray_Z,EXP5P,UTOK);			//item9
				QueryFloatArg(DUT_X,EXP5P,UTOK);			//item10

				QueryFloatArg(DUT_display_fitting_1,EXP5P,UTOK);			//item11
				QueryFloatArg(DUT_display_fitting_2,EXP5P,UTOK);			//item12

				QueryFloatArg(Vx_Vpp,EXP5P,UTOK);			//item13
				QueryFloatArg(Vr_Vpp,EXP5P,UTOK);			//item14

				QueryFloatArg(Sweep_mode_Vx_amp_display,EXP5P,UTOK);			//item15
				QueryFloatArg(sweep_freq_times,EXP5P,UTOK);			//item16

				QueryFloatArg(Sweep_mode_Vx_Vr_freq_display,EXP5P,UTOK);			//item17
				// QueryFloatArg(DFT_Lp_Y_val[3],EXP5P,UTOK);			//item18
				QueryFloatArg(Lp_Vpp,EXP5P,UTOK);			//item18

				// QueryFloatArg(DFT_Lp_X_val[4],EXP5P,UTOK);			//item19
				QueryFloatArg(Rr,EXP5P,UTOK);			//item19
				// QueryFloatArg(DFT_Lp_Y_val[4],EXP5P,UTOK);			//item20
				QueryFloatArg(DUT_Y,EXP5P,UTOK);			//item20

				// QueryFloatArg(sin(271/RAD2DEG),EXP5P,UTOK);			//item21
				QueryFloatArg(DFT_A_X_display,EXP5P,UTOK);			//item21
				// QueryFloatArg(cos(271/RAD2DEG),EXP5P,UTOK);			//item22
				QueryFloatArg(DFT_A_Y_display,EXP5P,UTOK);			//item22

				QueryFloatArg(DFT_A_Xita,EXP5P,UTOK);		//item23

//				QueryFloatArg(PSD_Vr_X_val[0],EXP5P,UTOK);	//item1
//				QueryFloatArg(PSD_Vr_Y_val[0],EXP5P,UTOK);			//item2
//
//				QueryFloatArg(PSD_Vr_X_val[1],EXP5P,UTOK);	//item3
//				QueryFloatArg(PSD_Vr_Y_val[1],EXP5P,UTOK);		//item4
//
//				QueryFloatArg(PSD_Vr_X_val[2],EXP5P,UTOK);				//item5
//				QueryFloatArg(PSD_Vr_Y_val[2],EXP5P,UTOK);				//item6
//
//				QueryFloatArg(PSD_Vr_X_val[3],EXP5P,UTOK);			//item7
//				QueryFloatArg(PSD_Vr_Y_val[3],EXP5P,UTOK);			//item8
//
//				QueryFloatArg(PSD_Vr_X_val[4],EXP5P,UTOK);			//item9
//				QueryFloatArg(PSD_Vr_Y_val[4],EXP5P,UTOK);			//item10
//
//				QueryFloatArg(PSD_Vr_X_val[5],EXP5P,UTOK);			//item11
//				QueryFloatArg(PSD_Vr_Y_val[5],EXP5P,UTOK);			//item12
//
//				QueryFloatArg(PSD_Vr_X_val[6],EXP5P,UTOK);			//item13
//				QueryFloatArg(PSD_Vr_Y_val[6],EXP5P,UTOK);			//item14
//
//				QueryFloatArg(PSD_Vr_X_val[7],EXP5P,UTOK);			//item15
//				QueryFloatArg(PSD_Vr_Y_val[7],EXP5P,UTOK);			//item16
//
//				QueryFloatArg(PSD_Vr_X_val[8],EXP5P,UTOK);			//item17
//				QueryFloatArg(PSD_Vr_Y_val[8],EXP5P,UTOK);			//item18
//
//				QueryFloatArg(PSD_Vr_X_val[9],EXP5P,UTOK);			//item19
//				QueryFloatArg(PSD_Vr_Y_val[9],EXP5P,UTOK);			//item20
//
//				QueryFloatArg(PSD_Vr_X_val[10],EXP5P,UTOK);			//item21
//				QueryFloatArg(PSD_Vr_Y_val[10],EXP5P,UTOK);			//item22
//
//				QueryFloatArg(PSD_B_Vpp,EXP5P,UTOK);		//item23
                Data_interact_ON_flag = 0;//The data is received and the interaction is over
			}
			//delay_us(3);
			usleep(3);
		}
			break;
		case SNAP:
            // Master_switch_flag = 1;
            // Workspace = DETERMINE_WORK_MODE;
            // Measure_Term = STRAY_C_MEASRUE_TERM;
			break;
		case OUTP:
			break;
		case TRCA:
			break;

//***************Set the ATT & AMP ************************//
		case SENS:
			if(cmd_opt_type==1)
			 {
				 QueryIntArg(Global_GainTC_Sensitivity, UEOS);
			 }
			else if(cmd_opt_type==2)
			 {
				 Option_Int_Value = atoi((char *)Parameter_Buff);
				 Global_GainTC_Sensitivity = Option_Int_Value;
				 IA_DGP_control();
			 }
			 break;

        case FMOD:
	         break;

//***************Set the phase of Vx ************************//
		case PHAS:
			if(cmd_opt_type==1)
			   {
					QueryFloatArg(Global_RefPhase_Phase, FLOAT2P, UEOS);
			   }
			else if(cmd_opt_type==2)
			   {
					Option_float_Value = atof((char *)Parameter_Buff);
					if(Option_float_Value > 360.0)
						{
							Option_float_Value = 360.0;
						}
					else if(Option_float_Value < 0.0)
						{
							Option_float_Value = 0.0;
						}
			   }
			Vx_phase_mod_float = Option_float_Value;
			Vx_phase_mod = (u32)(Option_float_Value * FPGA_PHASE_COE);				// 	DATA = phase/(2^32)*360
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
			break;

//***************Set the phase of Vr & Vmod & Vref************************//
		case SPHS:
            if(cmd_opt_type==1) //?
            {
         	   switch(inquire)
					{
						case 1:QueryFloatArg(Vr_phase_mod_float, FLOAT3P, UEOS); break;
						case 2:QueryFloatArg(Vmod_phase_mod_float, FLOAT3P, UEOS); break;
						case 3:QueryFloatArg(Vref_phase_mod_float, FLOAT3P, UEOS); break;
//						case 4:QueryFloatArg(Vmod_gain, FLOAT3P, UEOS); break;
						default:break;
					}
            }
            else if(cmd_opt_type==2)
                 String_Tokens_Separate((char *)Parameter_Buff,1,10);
            break;


/***** Set the frequency of DDS *****/
		case FREQ:
			{Option_float_Value = atof((char*)Parameter_Buff);

			/***** limit the freq from 0.1Hz to 40MHz *****/
			if(Option_float_Value > 40000000)
				Option_float_Value = 40000000;
			else if (Option_float_Value < 0.1)
				Option_float_Value = 0.1;

			/***** tranlate the labview parameter to FTW register variable *****/
			Vx_Vr_freq = Option_float_Value;
			DDS1_DDS2_freq_tunning_word = (Vx_Vr_freq/AD9552_Freq_out*(AD9508_DIVIDER+1))*pow(2,32);
			DDS1_FTW0[4] = DDS1_DDS2_freq_tunning_word;
			DDS1_FTW0[3] = DDS1_DDS2_freq_tunning_word>>8;
			DDS1_FTW0[2] = DDS1_DDS2_freq_tunning_word>>16;
			DDS1_FTW0[1] = DDS1_DDS2_freq_tunning_word>>24;
			DDS2_FTW0[4] = DDS1_FTW0[4];
			DDS2_FTW0[3] = DDS1_FTW0[3];
			DDS2_FTW0[2] = DDS1_FTW0[2];
			DDS2_FTW0[1] = DDS1_FTW0[1];


            /*--------  record fitting Stray_Z in current freq  --------*/
            if(Vx_Vr_freq < 10000000.0)
            {
                Stray_Z = 1.0/(Stray_fitting_coefficient_A + Stray_fitting_coefficient_B * Vx_Vr_freq);
            }
            else
            {
                Stray_Z = 86.0;
            }

			if(Vx_Vr_freq >= HIGH_LOW_FREQ_LIMIT)
			{
				freq_mode_flag = 0;		//	low freq
				freq_mixed_flag = 1;	//	mixed frequency
			}
			else
			{
				freq_mode_flag = 0;		//	low freq
				freq_mixed_flag = 0;	//	dont mixed frequency
			}

			switch(freq_mixed_flag)
			{
				/*			dont mixed Frequency		*/
				case 0:
					Vx_Vr_phase_inc = (Vx_Vr_freq * FPGA_FREQ_COE);
					Vref_phase_inc = Vx_Vr_phase_inc;									// fixed the freq of Vref to 1.8KHz;
					Vmod_phase_inc = 0;				// fixed the freq of Vmod, freq_Vmod = 0Hz
					Vmod_freq = 0;
					point_of_period = 1000000.0 / Vx_Vr_freq * cycles;
					DFT_delay_us = 1000000.0 / Vx_Vr_freq * cycles * 3;
					break;

				/*			 mixed Frequency		*/
				case 1:
					Vx_Vr_phase_inc = (Vx_Vr_freq * FPGA_FREQ_COE);
					Vref_phase_inc = 30924;									// fixed the freq of Vref to 1.8KHz;
					Vmod_phase_inc = Vx_Vr_phase_inc + 30924;				// fixed the freq of Vmod, freq_Vmod = freq_Vx + 1.8KHz
					Vmod_freq = Vx_Vr_freq+1800;
					point_of_period = 1000000.0 / 1800.0 * cycles;
					DFT_delay_us = 1000000.0 / 1800.0 * cycles * 3;
					break;

				default:
					Vx_Vr_phase_inc = (Vx_Vr_freq * FPGA_FREQ_COE);
					Vref_phase_inc = 30924;									// fixed the freq of Vref to 1.8KHz;
					Vmod_phase_inc = Vx_Vr_phase_inc + 30924;				// fixed the freq of Vmod, freq_Vmod = freq_Vx + 1.8KHz
					Vmod_freq = Vx_Vr_freq+1800;
					point_of_period = (u32)(1000000.0 / 1800.0 * cycles);
					DFT_delay_us = 1000000.0 / Vx_Vr_freq * cycles * 3;
					break;
			}

			/***** DDS configuration *****/
			DDS1_POW0[1]=0x00;
			DDS1_POW0[2]=0x00;
			DDS2_POW0[1]=0x00;
			DDS2_POW0[2]=0x00;
			DDS3_POW0[1]=0x00;
			DDS3_POW0[2]=0x00;

			DDS1_Write();
			usleep(10);
			DDS2_Write();
			usleep(10);
			DDS3_freq_tunning_word = (Vmod_freq/AD9552_Freq_out*(AD9508_DIVIDER+1))*pow(2,32);
			DDS3_FTW0[4] = DDS3_freq_tunning_word;
			DDS3_FTW0[3] = DDS3_freq_tunning_word>>8;
			DDS3_FTW0[2] = DDS3_freq_tunning_word>>16;
			DDS3_FTW0[1] = DDS3_freq_tunning_word>>24;
			DDS1_ASF[2] = 0x94;
			DDS1_ASF[1] = 0x13;
			DDS3_Write();
			usleep(10);


			//to vivado nco
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG15_OFFSET), Vx_Vr_phase_inc);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG20_OFFSET), Vmod_phase_inc);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG23_OFFSET), Vref_phase_inc);
			/***** synchronize all DDS by hardware Power_down pin *****/
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));//jst add
			usleep(100);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));//jst add
			//usleep(100);

			break;
			}

//***************Set the frequency of Vmod************************//
		case SFMD:
//			{Option_float_Value = atof((char*)Parameter_Buff);
//			if(Option_float_Value > 10000000)
//				Option_float_Value = 10000000;
//			else if (Option_float_Value < 20.0)
//				Option_float_Value = 20.0;
////			Vmod_phase_inc = (Option_float_Value * FPGA_FREQ_COE);
//			Vmod_phase_inc = Vx_Vr_phase_inc + 30924;				// fixed the freq of Vmod, freq_Vmod = freq_Vx + 1.8KHz
//			// XIL_OUT32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG20_OFFSET), Vmod_phase_inc);
			break;
//			}

//***************Set the frequency of Vmod************************//
		case SFRE:
//			{Option_float_Value = atof((char*)Parameter_Buff);
//			if(Option_float_Value > 10000000)
//				Option_float_Value = 10000000;
//			else if (Option_float_Value < 20.0)
//				Option_float_Value = 20.0;
////			Vref_phase_inc = (Option_float_Value * FPGA_FREQ_COE);
////			Vref_phase_inc = 30924;									// fixed the freq of Vref to 1.8KHz;
////			// XIL_OUT32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG23_OFFSET), Vref_phase_inc);
			break;
//			}

		case RSLP:
			break;
		case HARM:
			break;
		case SLVL:
			break;

//***************Control the switches on Analog board, including Vr source, Rr selecting, measurement items, low/high mode and the Bias current************************//
		case ISRC:
			if(cmd_opt_type==1)
						   {
						      QueryIntArg(IA_CTRL_Analog, UEOS);
						   }
						 else if(cmd_opt_type==2)
						   {
						      Option_Int_Value = atoi((char *)Parameter_Buff);

						      IA_CTRL_Analog = Option_Int_Value;
						      CTRL_Rr_Sel = (IA_CTRL_Analog >> 2) & 0x007;
						      Rr_Sel(CTRL_Rr_Sel);
						      Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

						   }

			break;

//***************for the single/repeative measure ************************//
//		case IGND:
////			if(cmd_opt_type==1)
////						   {
////						      QueryIntArg(Single_measure_flag, UEOS);
////						   }
////			else if(cmd_opt_type==2)
////						   {
////						      Option_Int_Value = atoi((char *)Parameter_Buff);
////						      Single_measure_flag = Option_Int_Value;
////						   }
//			Cal_single_initialize();
//			Single_measure_flag = 1;
//			Repetitive_measure_flag = 0;
//			break;
//
//		case ICPL:
////			if(cmd_opt_type==1)
////						   {
////						      QueryIntArg(Repetitive_measure_flag, UEOS);
////						   }
////			else if(cmd_opt_type==2)
////						   {
////						      Option_Int_Value = atoi((char *)Parameter_Buff);
////						      Repetitive_measure_flag = Option_Int_Value;
////						   }
//			Cal_repetitive_initialize();
//			Repetitive_measure_flag = !Repetitive_measure_flag;
//			Single_measure_flag = 0;
//			break;

//***************for the single/repeative measure ************************//
		case IGND:
			if(cmd_opt_type==1)
						   {
						      QueryIntArg(Master_switch_flag, UEOS);
						   }
			else if(cmd_opt_type==2)
			{
    			Master_switch_flag = 1;
                Workspace = DETERMINE_WORK_MODE;
                Measure_Mode = SINGLE_MEASURE_MODE;
                Cal_initialize();
			}
			break;

		case ICPL:
			if(cmd_opt_type==1)
						   {
							  // QueryIntArg(Repetitive_balance_flag, UEOS);
						   }
			else if(cmd_opt_type==2)
			{
                if (Measure_Mode == NONE_MEASURE_MODE) {
                    Master_switch_flag = 1;
                    Workspace = DETERMINE_WORK_MODE;
                    Measure_Mode = REPEAT_MEASURE_MODE;
                    Cal_initialize();
                }else if (Measure_Mode == REPEAT_MEASURE_MODE) {
                    Master_switch_flag = 0;
                    Workspace = NULL_WORKSPACE;
                    Measure_Mode = NONE_MEASURE_MODE;
                }


                //Tony add FOR initialize the DDS
                if(Measure_Mode == REPEAT_MEASURE_MODE)
                {
                    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
                    usleep(2);
                    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
                    usleep(1);
                }

			}
			break;






		case ILIN:
            if (Enable_Auto_Rr_flag == 0)
            {
                Enable_Auto_Rr_flag = 1;
            }else
            {
                Enable_Auto_Rr_flag = 0;
            }
			break;
		case RMOD:
            //cant use!
			break;

//***************Set the Timeconstant of IIR************************//
		case OFLT:
			if(cmd_opt_type==1)
			   {
			      QueryIntArg(Global_GainTC_TimeConstant, UEOS);
			   }
			 else if(cmd_opt_type==2)
			   {
			      Option_Int_Value = atoi((char *)Parameter_Buff);
				  Global_GainTC_TimeConstant = Option_Int_Value;
				  IA_DGP_control();
			   }
			break;

//***************Set the OFSL of IIR************************//
		case OFSL:
			if(cmd_opt_type==1)
			   {
			      QueryIntArg(Global_GainTC_Fliter, UEOS);
			   }
			 else if(cmd_opt_type==2)
			   {
			      Option_Int_Value = atoi((char *)Parameter_Buff);
			      Global_GainTC_Fliter = Option_Int_Value;
				  IA_DGP_control();
			   }
			break;
//***************	************************//
		case SYNC:
			break;
		case AGAN:
			break;
		case ARSV:
			break;
		case APHS:
			break;
		case ASCL:
			break;
		case FPOP:
			break;
		case OEXP:
			break;
		case SSET:
			break;
		case RSET:
			break;
		case SRAT:
			break;
		case STRD:
			break;
		case SPTS:
			break;
		case REST:
			break;
		case _RST:
			break;
		case _IDN:
		{
			if(cmd_opt_type==1)
			{
				Send_Data2PC_str("SSI IA-OE7005,SN:L,Ver1.000\n");
			}
			break;
		}
		case SWPT:
            if (Measure_Term == DUT_MEASRUE_TERM)
            {
                Measure_Term = Rr_MEASRUE_TERM;
            }else
            {
                Measure_Term = DUT_MEASRUE_TERM;
            }
			break;
//		case SLLM:
//		{
//			Option_Int_Value = atoi((char*)Parameter_Buff);
//			if(Option_Int_Value == 1)
//			{
//				s_r_r50  = 1;
//				s_r_r500 = 0;
//				s_r_r5k  = 0;
//				s_r_r50k = 0;
//				s_c_c16  = 0;
//				s_c_c2p2 = 1;
//				s_c_c4c7 = 0;
//				RES = 50;
//			}
//			else if(Option_Int_Value == 2)
//			{
//				s_r_r50  = 0;
//				s_r_r500 = 1;
//				s_r_r5k  = 0;
//				s_r_r50k = 0;
//				s_c_c16  = 0;
//				s_c_c2p2 = 1;
//				s_c_c4c7 = 0;
//				RES = 500;
//			}
//			else if(Option_Int_Value == 3)
//			{
//				s_r_r50  = 0;
//				s_r_r500 = 0;
//				s_r_r5k  = 1;
//				s_r_r50k = 0;
//				s_c_c16  = 0;
//				s_c_c2p2 = 1;
//				s_c_c4c7 = 0;
//				RES = 5000;
//			}
//			else if(Option_Int_Value == 4)
//			{
//				s_r_r50  = 0;
//				s_r_r500 = 0;
//				s_r_r5k  = 0;
//				s_r_r50k = 1;
//				s_c_c16  = 0;
//				s_c_c2p2 = 0;
//				if(freq < 10)
//					s_c_c4c7 = 0;
//				else
//					s_c_c4c7 = 0;
//				RES = 50000;
//			}
//			else
//			{
//				s_r_r50  = 0;
//				s_r_r500 = 0;
//				s_r_r5k  = 1;
//				s_r_r50k = 0;
//				s_c_c16  = 0;
//				s_c_c2p2 = 1;
//				s_c_c4c7 = 0;
//				RES = 5000;
//			}
//			break;}

		case SULM:
//		{
//			Option_Int_Value = atoi((char*)Parameter_Buff);
//			V_1_1 = ((Option_Int_Value&0x00000001) == 0);
//			V_1_2 = ((Option_Int_Value&0x00000002) == 0);
//			V_1_5 = ((Option_Int_Value&0x00000004) == 0);
//			V_1_10 = ((Option_Int_Value&0x00000008) == 0);
//			V_2_1 = ((Option_Int_Value&0x00000010) == 0);
//			V_2_2 = ((Option_Int_Value&0x00000020) == 0);
//			V_2_5 = ((Option_Int_Value&0x00000040) == 0);
//			V_2_10 = ((Option_Int_Value&0x00000080) == 0);
//			V_3_1 = ((Option_Int_Value&0x00000100) == 0);
//			V_3_2 = ((Option_Int_Value&0x00000200) == 0);
//			V_3_5 = ((Option_Int_Value&0x00000400) == 0);
//			V_3_10 = ((Option_Int_Value&0x00000800) == 0);
//			break;}
			break;
		case SSLL:
//		{
//			Option_Int_Value = atoi((char*)Parameter_Buff);
//			I_1_1 = ((Option_Int_Value&0x00000001) == 0);
//			I_1_2 = ((Option_Int_Value&0x00000002) == 0);
//			I_1_5 = ((Option_Int_Value&0x00000004) == 0);
//			I_1_10 = ((Option_Int_Value&0x00000008) == 0);
//			I_2_1 = ((Option_Int_Value&0x00000010) == 0);
//			I_2_2 = ((Option_Int_Value&0x00000020) == 0);
//			I_2_5 = ((Option_Int_Value&0x00000040) == 0);
//			I_2_10 = ((Option_Int_Value&0x00000080) == 0);
//			I_3_1 = ((Option_Int_Value&0x00000100) == 0);
//			I_3_2 = ((Option_Int_Value&0x00000200) == 0);
//			I_3_5 = ((Option_Int_Value&0x00000400) == 0);
//			I_3_10 = ((Option_Int_Value&0x00000800) == 0);
//			break;}
			break;
		case SSLG:
//		{
//			Option_Int_Value = atoi((char*)Parameter_Buff);
//			V_AMP = Option_Int_Value;
//			break;}
			break;
		case STLM:
//		{
//			Option_Int_Value = atoi((char*)Parameter_Buff);
//			I_AMP = Option_Int_Value;
//			break;}
			break;
		case SWRM:
            String_Tokens_Separate((char *)Parameter_Buff,1,7);
			break;
		case SWVT:
            String_Tokens_Separate((char *)Parameter_Buff,1,8);
			break;
		case SVLL:
			break;
		case SVUL:
			break;
		case SVSL:
			break;
		case SVSG:
			break;
		case SVTM:
			break;
		case SVRM:
			break;
		case SLEN:
			break;
		case SSLE:
			break;
		case STRG:
			break;
		case SPRM:
			break;
		case PAUS:
			break;
		case OAUX:
//		{	Option_float_Value = atof((char*)Parameter_Buff);
//		    AUX_DA1_float = Option_float_Value;
//		    AUX_DA_SEL = 1;
//		    delay_us(3);
//			break;}
               if(cmd_opt_type==1) //?
               {
            	   switch(inquire)
					{
						case 1:QueryFloatArg(AUX_DA1_float, FLOAT3P, UEOS); break;
						case 2:QueryFloatArg(AUX_DA2_float, FLOAT3P, UEOS); break;
						case 3:QueryFloatArg(AUX_DA3_float, FLOAT3P, UEOS); break;
//						case 4:QueryFloatArg(Vmod_gain, FLOAT3P, UEOS); break;
						default:break;
					}
               }
               else if(cmd_opt_type==2)
                    String_Tokens_Separate((char *)Parameter_Buff,1,9);
			break;



		case FUNC:
//		{	Option_float_Value = atof((char*)Parameter_Buff);
//		    AUX_DA1_float = Option_float_Value;
//		    AUX_DA_SEL = 1;
//		    delay_us(3);
//			break;}
               if(cmd_opt_type==1) //?
               {
            	   switch(inquire)
					{
						case 1:QueryFloatArg(sequence_1, EXP5P, UEOS); break;
						case 2:QueryFloatArg(sequence_2, EXP5P, UEOS); break;
//						case 4:QueryFloatArg(Vmod_gain, FLOAT3P, UEOS); break;
						default:break;
					}
               }
               else if(cmd_opt_type==2)
                    String_Tokens_Separate((char *)Parameter_Buff,1,11);
			break;


		case EQCD:
			break;
		case EQCS:
			break;
		case SPED:
			break;
			break;
		case INOV:
			break;
		case GNOV:
			break;
		case _PLL:
			break;
		case FRAM:
			break;
		case FMUL:
			break;
		case STHR:
			break;
		default:break;

	}
}

u8 String_Tokens_Separate(char *string, u8 Tokens_len, u8 token_cmd)
{
    char tokens_count=0;
    char tokens_char=0;
    u16 i=0;
    char *token=NULL;
    char *buf_token[14]={0};
    for(token=strtok(string,","); token!= NULL;token = strtok(NULL,","),tokens_count++)// ,Send_Data2PC_str(token)
    {
        buf_token[tokens_count]=token;
        if(tokens_count==Tokens_len)break;
    }
		switch(token_cmd)
		{
			case 1:
//						tokens_char = atoi(buf_token[0]);
//						if(tokens_char==1||tokens_char==2)
//						{
//								Global_Output_Channel = tokens_char;
//							  if(Global_Output_Channel==1)
//							  {
//									 Global_Output_CH1_Source  = atof(buf_token[1]);
//									 if(Global_Output_Speed == 1)
//									 {
//											 if(Global_Output_CH1_Source > 2)
//													Global_Output_CH1_Source = 2;
//											 else if(Global_Output_CH1_Source < 0)
//													Global_Output_CH1_Source = 0;
//											 OffsetExpand_para_send();
//									 }
//									 else
//									 {
//											 if(Global_Output_CH1_Source>16)
//													Global_Output_CH1_Source=16;
//											 else if(Global_Output_CH1_Source<0)
//													Global_Output_CH1_Source=0;
//									 }
//									 SaveSettingData(130, Global_Output_CH1_Source);
//							  }
//							  else
//							  {
//									 Global_Output_CH2_Source  = atof(buf_token[1]);
//									 if(Global_Output_Speed == 1)
//									 {
//											 if(Global_Output_CH2_Source > 2)
//													Global_Output_CH2_Source = 2;
//											 else if(Global_Output_CH2_Source < 0)
//													Global_Output_CH2_Source = 0;
//											 OffsetExpand_para_send();
//									 }
//									 else
//									 {
//											 if(Global_Output_CH2_Source>16)
//													Global_Output_CH2_Source=16;
//											 else if(Global_Output_CH2_Source<0)
//													Global_Output_CH2_Source=0;
//									 }
//									 SaveSettingData(131, Global_Output_CH2_Source);
//							  }
//						}
					 break;
			case 2:

//					 tokens_char = atoi(buf_token[0]);
//					 if(tokens_char<10)
//					 {
//								 OffsetExpand_Source = tokens_char;
//
//								 Offset_value     = atof(buf_token[1]);
//								 Expand_value     = atoi(buf_token[2]);
//								 if(Offset_value>100.0)
//											Offset_value=100.0;
//								 else if(Offset_value < -100.0)
//											Offset_value=-100.0;
//								 if(Expand_value<1)
//											Expand_value=1;
//								 else if(Expand_value>256)
//											Expand_value=256;
//								 OffsetExpand_Offset_Assign();
//								 OffsetExpand_Expand_Assign();
//								 if(Global_Output_Speed == 1 && OffsetExpand_Source<2)
//										OffsetExpand_para_send();
//						}
//					 break;
			case 3:
//						if(tokens_count>=1)
//						{
//								for(i=0;i<tokens_count;i++)
//								{
//										tokens_char = atoi(buf_token[i]);
//										if(tokens_char<=0||tokens_char>22)
//										return 0;
//								}
//								for(i=0;i<tokens_count;i++)
//								{
//										switch(atoi(buf_token[i]))
//										{
//												case 1:  QueryFloatArg(FPGA_X,          EXP5P,   NEOS); break;
//												case 2:  QueryFloatArg(FPGA_Y,          EXP5P,   NEOS); break;
//												case 3:  QueryFloatArg(FPGA_R,          EXP5P,   NEOS); break;
//												case 4:  QueryFloatArg(FPGA_Xita,       FLOAT3P, NEOS); break;
//												case 5: if(Global_RefPhase_Source==1)
//																		QueryFloatArg(FPGA_Freq, FLOAT3P, NEOS);
//																else if(Global_RefPhase_Source==2)
//																		QueryFloatArg(Internal_Freqvalue, FLOAT3P, NEOS);
//																else
//																		QueryFloatArg(Sweep_Freq_Freq_temp, FLOAT3P, NEOS);
//																break;
//												case 6:  QueryFloatArg(FPGA_X_HARM1,    EXP5P,   NEOS); break;
//												case 7:  QueryFloatArg(FPGA_Y_HARM1,    EXP5P,   NEOS); break;
//												case 8:  QueryFloatArg(FPGA_R_HARM1,    EXP5P,   NEOS); break;
//												case 9:  QueryFloatArg(FPGA_Xita_HARM1, FLOAT3P, NEOS); break;
//												case 10: QueryFloatArg(FPGA_X_HARM2,    EXP5P,   NEOS); break;
//												case 11: QueryFloatArg(FPGA_Y_HARM2,    EXP5P,   NEOS); break;
//												case 12: QueryFloatArg(FPGA_R_HARM2,    EXP5P,   NEOS); break;
//												case 13: QueryFloatArg(FPGA_Xita_HARM2, FLOAT3P, NEOS); break;
//												case 14: QueryFloatArg(FPGA_NOISE,      EXP5P,   NEOS); break;
//												case 15: QueryIntArg  (LMS_w1,                   NEOS); break;
//												case 16: QueryIntArg  (LMS_w2,                   NEOS); break;
//// 												case 15: QueryFloatArg(AD1,             FLOAT3P, NEOS); break;
//// 												case 16: QueryFloatArg(AD2,             FLOAT3P, NEOS); break;
//												case 17: QueryFloatArg(AD3,             FLOAT3P, NEOS); break;
//												case 18: QueryFloatArg(AD4,             FLOAT3P, NEOS); break;
//												case 19: QueryFloatArg(Equation1,       EXP5P,   NEOS); break;
//												case 20: QueryFloatArg(Equation2,       EXP5P,   NEOS); break;
//												case 21: QueryFloatArg(Equation3,       EXP5P,   NEOS); break;
//												case 22: QueryFloatArg(Equation4,       EXP5P,   NEOS); break;
//												default:break;
//
//										}
//										if(i!=(tokens_count-1))
//										Send_Data2PC(',');
//								}
//								Send_Data2PC(0X0D);
//						}
						break;
			case 4:
//						if(tokens_count>=2)
//						{
//								u8 Buf_Ch=0;
//								u8 Buf_Query_finish=1;
//								u16 Buf_star=0,Buf_len=0;
//								Buf_Ch   = atoi(buf_token[0]);
//								Buf_star = atoi(buf_token[1]);
//								Buf_len  = atoi(buf_token[2]);
//								if((Buf_star+Buf_len<=Sample_Current_Points)&&(Sample_Current_Points>0))
//								{
//										if(Buf_Ch>0&&Buf_Ch<=4)
//										{
//												if(Buf_len-Buf_star<=1024)
//												{
//														USART_Read_Sram_Buffer(Buf_Ch, Buf_star, Buf_len);
//														for(i=0;i<Buf_len;i++)
//														{
//																QueryFloatArg(SampleBuf1pStart[i], EXP5P, NEOS);
//																Send_Data2PC(',');
//														}
//														IWDG->KR = ((uint16_t)0xAAAA);
//												}
//												else //>1024
//												{
//														while(Buf_Query_finish)
//														{
//																if(Buf_len>1024)
//																{
//																		USART_Read_Sram_Buffer(Buf_Ch, Buf_star, 1024);
//																		for(i=0;i<1024;i++)
//																		{
//																				QueryFloatArg(SampleBuf1pStart[i], EXP5P, NEOS);
//																				Send_Data2PC(',');
//																		}
//																		Buf_star+=1024;
//																		Buf_len -=1024;
//																}
//																else //(Buf_len - Buf_star < 1024)
//																{
//																		USART_Read_Sram_Buffer(Buf_Ch, Buf_star, Buf_len);
//																		for(i=0;i<Buf_len;i++)
//																		{
//																				QueryFloatArg(SampleBuf1pStart[i], EXP5P, NEOS);
//																				Send_Data2PC(',');
//																		}
//																		Buf_Query_finish = 0;
//																}
//																IWDG->KR = ((uint16_t)0xAAAA);
//														}
//
//												}
//												Send_Data2PC(0X0D);
//										}
//								}
//						}
						break;
			case 5:
//						 Buffer1_Source_Count  = atoi(buf_token[0]);
//						 Buffer2_Source_Count  = atoi(buf_token[1]);
//						 if(Buffer1_Source_Count>3)
//									Buffer1_Source_Count=3;

//						 if(Buffer2_Source_Count>3)
//									Buffer2_Source_Count=3;
						break;
			case 6:
//						Harm_param_source = atoi(buf_token[0]);
//						if(Harm_param_source == 1)
//						{
//								Harm_Value1  = atoi(buf_token[1]);
//								if(Harm_Value1 == 0)
//										Harm_Value1 = 1;
//								else if(Harm_Value1>32767)
//										Harm_Value1 = 32767;
//								Send_RefPhase_Harm_Value1();
//								unINT16U.iData   = Harm_Value1;
//								SaveMultiByteSettingData(38, unINT16U.byData, 2);
//						}
//						else if(Harm_param_source == 2)
//						{
//								Harm_Value2  = atoi(buf_token[1]);
//								if(Harm_Value2 == 0)
//										Harm_Value2 = 1;
//								else if(Harm_Value2>32767)
//										Harm_Value2 = 32767;
//								Send_RefPhase_Harm_Value2();
//								unINT16U.iData   = Harm_Value2;
//								SaveMultiByteSettingData(40, unINT16U.byData, 2);
//						}
						break;
			case 7:
                     sweep_amp_conduct_temp = atoi(buf_token[0]);
					 switch(sweep_amp_conduct_temp)
					 {
					    case 1:
							 	 	sweep_amp_type = atoi(buf_token[1]);
									break;
						case 2:
							 	 	sweep_amp_start = atof(buf_token[1]);
									break;
						case 3:
							 	 	sweep_amp_stop = atof(buf_token[1]);
									break;
						case 4:
							 	 	sweep_amp_points = atoi(buf_token[1]);
                                    Fill_amp_sweep_array_start_sweep(sweep_amp_type,sweep_amp_start,sweep_amp_stop,sweep_amp_points);
									break;
						default:break;
					 }
					 break;
			case 8:
                     sweep_freq_conduct_temp = atoi(buf_token[0]);
					 switch(sweep_freq_conduct_temp)
					 {
					    case 1:
							 	 	sweep_freq_type = atoi(buf_token[1]);
									break;
						case 2:
							 	 	sweep_freq_start = atof(buf_token[1]);
									break;
						case 3:
							 	 	sweep_freq_stop = atof(buf_token[1]);
									break;
						case 4:
							 	 	sweep_freq_points = atoi(buf_token[1]);
                                    Fill_freq_sweep_array_start_sweep(sweep_freq_type,sweep_freq_start,sweep_freq_stop,sweep_freq_points);
									break;
						default:break;
					 }
					 break;

//***********************************************set the DAC gain output********************************************************//
		case 9:
				 DAC_Output_Temp = atoi(buf_token[0]);
				 switch(DAC_Output_Temp)
				 {
					 /***** Set the amplitude of DDS *****/
					 case 1:

								AUX_DA1_float = atof(buf_token[1]);

								/***** limit the amplitude percentage from 0% to 100% *****/
								 if(AUX_DA1_float>5.0)
									 AUX_DA1_float=5.0;
								 else if(AUX_DA1_float<0.0)
									 AUX_DA1_float=0.0;

								 Vx_gain = (u32)(AUX_DA1_float * FPGA_GAIN_COE);
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG18_OFFSET), Vx_gain);

								/***** tranlate the labview parameter to ASF register variable *****/
								DDS_output_gain = (double)(AUX_DA1_float);
								DDS1_amplitude_factor = (AUX_DA1_float/5.22)*16383;	//full scale output is 1.044 Vrms when 50R load
								DDS1_ASF[2] = DDS1_amplitude_factor;
								DDS1_ASF[1] = DDS1_amplitude_factor>>8;

								/***** DDS configuration *****/
								DDS1_Write();
								usleep(50);

								if(Vx_Vr_freq >= 10000)
								{
									DDS3_ASF[2] = 0x9e;
									DDS3_ASF[1] = 0x13;
									DDS3_Write();
									usleep(50);
								}
								else
								{
									DDS3_FTW0[4] = 0;
									DDS3_FTW0[3] = 0;
									DDS3_FTW0[2] = 0;
									DDS3_FTW0[1] = 0;
									DDS3_POW0[1]= 0x00;		//default deg=0
									DDS3_POW0[2]= 0x00;		//default deg=0
									DDS3_ASF[1]= 0x13;		//default deg=0
									DDS3_ASF[2]= 0x9e;		//default deg=0
									DDS3_Write();
									usleep(10);
								}



								/***** synchronize all DDS by hardware Power_down pin *****/
								Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
								usleep(100);
								Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
								usleep(100);
/*								DDS3_RST_valid;
								usleep(500);
								DDS3_RST_invalid;*/

								break;

					 /***** Set the freq_divider of AD9508 *****/
					 case 2:
								 AUX_DA2_float = atof(buf_token[1]);
									/***** limit the amplitude percentage from 0% to 100% *****/
									 if(AUX_DA2_float>5.0)
										 AUX_DA2_float=5.0;
									 else if(AUX_DA2_float<0.0)
										 AUX_DA2_float=0.0;

										/***** tranlate the labview parameter to ASF register variable *****/
										DDS2_amplitude_factor = (AUX_DA2_float/5.22)*16383;	//full scale output is 1.044 Vrms when 50R load
										DDS2_ASF[2] = DDS2_amplitude_factor;
										DDS2_ASF[1] = DDS2_amplitude_factor>>8;

										/***** DDS configuration *****/
										DDS2_Write();
										usleep(10);
										Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
										usleep(1000);
										Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
										usleep(100);

/*								 **** limit the freq_divider from 0 to 1024 ****
								 if(AUX_DA2_float>1024)
									 AUX_DA2_float=1024;
								 else if(AUX_DA2_float<0.0)
									 AUX_DA2_float=0.0;
								 AD9508_DIVIDER = (int)AUX_DA2_float;

								 **** AD9508 configuration ****
								 AD9508_Write();
								 usleep(10);*/

								 break;

					 case 3:
								 AUX_DA3_float = atof(buf_token[1]);
								 if(AUX_DA3_float>5.0)
									 AUX_DA3_float=5.0;
								 else if(AUX_DA3_float<0.0)
									 AUX_DA3_float=0.0;



								 // fixed Vmod to 4Vpp
								 // XIL_OUT32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG22_OFFSET), Vmod_gain);
//									 unFloat.fData = DA2;
//									 SaveMultiByteSettingData(209, unFloat.byData, 4);
								 break;
					 case 4:
								 AUX_DA4_float = atof(buf_token[1]);
								 if(AUX_DA4_float>5.0)
									 AUX_DA4_float=5.0;
								 else if(AUX_DA4_float<0.0)
									 AUX_DA4_float=0.0;
//									 // XIL_OUT32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG22_OFFSET), Vmod_gain);
//									 unFloat.fData = DA4;
//									 SaveMultiByteSettingData(217, unFloat.byData, 4);
								 break;
								 default:break;
				 }
				 break;

//***************Set the phase of Vr & Vmod & Vref************************//
			case 10:
				 DAC_Phase_Temp = atoi(buf_token[0]);
				 switch(DAC_Phase_Temp)
				 {
					 case 1:
						 	 	 Vr_phase_mod_float = atof(buf_token[1]);
								 if(Vr_phase_mod_float>360.0)
									 Vr_phase_mod_float=360.0;
								 else if(Vr_phase_mod_float<0.0)
									 Vr_phase_mod_float=0.0;
								 Vr_phase_mod = (u32)(Vr_phase_mod_float * FPGA_PHASE_COE);
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);

//									 unFloat.fData = DA1;
//									 SaveMultiByteSettingData(205, unFloat.byData, 4);
								 break;
					 case 2:
						 	 	 Vmod_phase_mod_float = atof(buf_token[1]);
								 if(Vmod_phase_mod_float>360.0)
									 Vmod_phase_mod_float=360.0;
								 else if(Vmod_phase_mod_float<0.0)
									 Vmod_phase_mod_float=0.0;
								 Vmod_phase_mod = 0;											//fixed the phase of Vmod to 0;
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
//									 unFloat.fData = DA2;
//									 SaveMultiByteSettingData(209, unFloat.byData, 4);
								 break;
					 case 3:
						 	 	 Vref_phase_mod_float = atof(buf_token[1]);
								 if(Vref_phase_mod_float>360.0)
									 Vref_phase_mod_float=360.0;
								 else if(Vref_phase_mod_float<0.0)
									 Vref_phase_mod_float=0.0;
								 Vref_phase_mod = 0;											//fixed the phase of Vref to 0;
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);

                                 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
                                 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
								 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);
                                 usleep(2);
                                 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
//									 unFloat.fData = DA3;
//									 SaveMultiByteSettingData(213, unFloat.byData, 4);
								 break;
					 case 4:
						 	 	 cycles = atoi(buf_token[1]);

//									 AUX_DA4_float=0;
////									 Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG22_OFFSET), Vmod_gain);
////									 unFloat.fData = DA4;
////									 SaveMultiByteSettingData(217, unFloat.byData, 4);
								 break;
								 default:break;
				 }
				 break;

			case 11:
				FUNC_Term_Temp = atoi(buf_token[0]);
                switch(FUNC_Term_Temp)
                {
                    case 1:
                        sequence_1 = atoi(buf_token[1]);
                        break;
                    case 2:
                        sequence_2 = atoi(buf_token[1]);
                        break;
                    case 3:
                        Measure_Term = atoi(buf_token[1]);
                        break;
                    default:break;
                }
                break;

		}



    return 0;
}

//*******************************************set the AMP & ATT & IIR(TimeConstance & OFSL)************************************************************//
/*void IA_DGP_control(void)
	{
		DGP_control = Global_GainTC_Sensitivity + (Global_GainTC_TimeConstant<<16) + (Global_GainTC_Fliter<<18) ;
		//XIL_OUT32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG25_OFFSET), DGP_control);
	}*/
void IA_DGP_control(void)
	{
		DGP_control = Global_GainTC_Sensitivity + (Global_GainTC_TimeConstant<<16) + (Global_GainTC_Fliter<<20) ;
		Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG25_OFFSET), DGP_control);
	}

/***********	Tony alter	for longer delay	*****/
void delay_us(u32 timeus)
{
	timeus*=70;
	for(;timeus>=1;timeus--);
}

void Rr_Sel(u8 CTRL_Rr_Sel)
{
	switch(CTRL_Rr_Sel)
	{
		case 0:
			Rr = 500;
			break;
		case 1:
			Rr = 5000;
			break;
		case 2:
			Rr = 50000;
			break;
		case 3:
			Rr = 5000;
			break;
		case 4:
			Rr = 5000;
			break;
		default:
			Rr = 5000;
			break;
	}
}

void Fill_freq_sweep_array_start_sweep(u8 sweep_freq_type_temp, Xfloat32 sweep_freq_start_temp, Xfloat32 sweep_freq_stop_temp, u32 sweep_freq_points_temp)
{
    u32 i;
    u8 sweep_freq_data_true_flag = 1;
    Xfloat32 freq_sweep_inc;
    if (sweep_freq_type_temp)
    {
        freq_sweep_inc = (log10(sweep_freq_stop_temp) - log10(sweep_freq_start_temp)) / (sweep_freq_points_temp - 1);
        for (i = 0 ; i <= sweep_freq_points_temp - 1 ; i++ )
        {
            freq_sweep_array[i] = floor(sweep_freq_start_temp * pow(10.0,(freq_sweep_inc * i)));
        }
    }
    else
    {
        freq_sweep_inc = (sweep_freq_stop_temp - sweep_freq_start_temp) / (sweep_freq_points_temp - 1);
        for (i = 0 ; i <= sweep_freq_points_temp - 1 ; i++ )
        {
            freq_sweep_array[i] = floor(sweep_freq_start_temp + freq_sweep_inc * i);
        }
    }
    if (sweep_freq_data_true_flag)
    {
        if (Measure_Mode == NONE_MEASURE_MODE) {
            Master_switch_flag = 1;
            Workspace = DETERMINE_WORK_MODE;
            Measure_Mode = FREQ_SWEEP_MEASURE_MODE;
            sweep_freq_times = 0;
            Cal_initialize();
        }
        else if (Measure_Mode == FREQ_SWEEP_MEASURE_MODE) {
            Master_switch_flag = 0;
            Workspace = NULL_WORKSPACE;
            Measure_Mode = NONE_MEASURE_MODE;
            Data_interact_ON_flag = 0;
            for (freq_sweep_pointer_temp = freq_sweep_array ; *freq_sweep_pointer_temp != 0.0 ; freq_sweep_pointer_temp++ ) {
                *freq_sweep_pointer_temp = 0.0;
            }
            freq_sweep_pointer_temp = freq_sweep_array;
        }
    }
}

void Fill_amp_sweep_array_start_sweep(u8 sweep_amp_type_temp, Xfloat32 sweep_amp_start_temp, Xfloat32 sweep_amp_stop_temp, u32 sweep_amp_points_temp)
{
    u32 i;
    u8 sweep_amp_data_true_flag = 1;
    Xfloat32 amp_sweep_inc;
    if (sweep_amp_type_temp)
    {
        amp_sweep_inc = (log10(sweep_amp_stop_temp) - log10(sweep_amp_start_temp)) / (sweep_amp_points_temp - 1);
        for (i = 0 ; i <= sweep_amp_points_temp - 1 ; i++ )
        {
            amplitude_sweep_array[i] = sweep_amp_start_temp * pow(10.0,(amp_sweep_inc * i));
        }
    }
    else
    {
        amp_sweep_inc = (sweep_amp_stop_temp - sweep_amp_start_temp) / (sweep_amp_points_temp - 1);
        for (i = 0 ; i <= sweep_amp_points_temp - 1 ; i++ )
        {
            amplitude_sweep_array[i] = sweep_amp_start_temp + amp_sweep_inc * i;
        }
    }
    if (sweep_amp_data_true_flag)
    {
        if (Measure_Mode == NONE_MEASURE_MODE) {
            Master_switch_flag = 1;
            Workspace = DETERMINE_WORK_MODE;
            Measure_Mode = AMP_SWEEP_MEASURE_MODE;
            sweep_freq_times = 0;
            Cal_initialize();
        }
        else if (Measure_Mode == AMP_SWEEP_MEASURE_MODE) {
            Master_switch_flag = 0;
            Workspace = NULL_WORKSPACE;
            Measure_Mode = NONE_MEASURE_MODE;
            Data_interact_ON_flag = 0;
            for (amp_sweep_pointer_temp = amplitude_sweep_array ; *amp_sweep_pointer_temp != 0.0 ; amp_sweep_pointer_temp++ ) {
                *amp_sweep_pointer_temp = 0.0;
            }
            amp_sweep_pointer_temp = amplitude_sweep_array;
        }
    }
}



