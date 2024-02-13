/*
 * Cal_DUT.c
 *
 *      Author: Zachary
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "ARM_interface.h"
#include "UART.h"
#include <sleep.h>

#define   FPGA_FREQ_COE   17.179869184  //clk=250M  FPGA_FREQ_CODE=(2^32-1)/250M	zdw
#define   FPGA_GAIN_COE   13107  //16bits DAC  FPGA_GAIN_COE=(2^16-1)/5	zdw
#define   FPGA_PHASE_COE   11930464.71  //32bits  FPGA_PHASE_COE=(2^32-1)/360	zdw
#define   SYNC_SAMPLE_CODE 244140.625  //250000000/1024 = 244140.625

#define Matrix_Dimension 5	//attention!!
#define Solve_Dimension 3	//attention!!

volatile double high_freq_Vr_X_single_0 = 0.0;
volatile double high_freq_Vr_Y_single_0 = 0.0;
volatile double high_freq_Lp_X_single_0 = 0.0;
volatile double high_freq_Lp_Y_single_0 = 0.0;

//	Tony add
volatile double PSD_high_freq_Vr_X_single_0 = 0.0;
volatile double PSD_high_freq_Vr_Y_single_0 = 0.0;
volatile double PSD_high_freq_Lp_X_single_0 = 0.0;
volatile double PSD_high_freq_Lp_Y_single_0 = 0.0;

volatile double FPGA_A_Cal = 0.0;
volatile double FPGA_B_Cal = 0.0;
volatile double FPGA_C_Cal = 0.0;
volatile double FPGA_D_Cal = 0.0;

//	Tony add
volatile double high_freq_DUT_X = 0.0;
volatile double high_freq_DUT_Y = 0.0;
volatile double DUT_high_freq_display_1 = 0.0;
volatile double DUT_high_freq_display_2 = 0.0;
volatile u8 high_freq_Cal_single_state = 0;
volatile u8 high_freq_Cal_repetitive_state = 0;
volatile u32 high_freq_Cal_Vx_gain = 0;
volatile u32 high_freq_Cal_Vr_gain = 0;

volatile double high_freq_DUT_X_aver = 0.0;
volatile double high_freq_DUT_Y_aver = 0.0;
volatile u8 high_freq_cnt = 0;
volatile double high_freq_sum_X = 0.0;
volatile double high_freq_sum_Y = 0.0;

volatile u8 high_freq_moving_aver_cnt_X =  0;
volatile double high_freq_moving_aver_data_X_1st = 0.0;
volatile double high_freq_moving_aver_sum_X = 0.0;
volatile double high_freq_moving_aver_result_X = 0.0;
volatile u8 high_freq_moving_aver_cnt_Y =  0;
volatile double high_freq_moving_aver_data_Y_1st = 0.0;
volatile double high_freq_moving_aver_sum_Y = 0.0;
volatile double high_freq_moving_aver_result_Y = 0.0;

extern double DFT_A_X_display;
extern double DFT_A_Y_display;
extern double DFT_B_X_display;
extern double DFT_B_Y_display;

//	Tony add
extern double PSD_A_X_display;
extern double PSD_A_Y_display;
extern double PSD_B_X_display;
extern double PSD_B_Y_display;
extern double DFT_A_Vpp;
extern double DFT_A_Xita;
extern double DFT_B_Vpp;
extern double DFT_B_Xita;
extern double PSD_A_Vpp;
extern double PSD_A_Xita;
extern double PSD_B_Vpp;
extern double PSD_B_Xita;

volatile double PSD_high_freq_Vr_Vpp_single_0 =0.0;
volatile double PSD_high_freq_Lp_Vpp_single_0 =0.0;

volatile double high_freq_Vr_Vpp_single_0 =0.0;
volatile double high_freq_Lp_Vpp_single_0 =0.0;

volatile u8 linear_cnt = 0;

volatile double DFT_Lp_X_val[10];
volatile double DFT_Lp_Y_val[10];
volatile double DFT_Vr_X_val[10];
volatile double DFT_Vr_Y_val[10];
volatile double DFT_Vx_X_val[10];
volatile double DFT_Vx_Y_val[10];

volatile double PSD_Lp_X_val[10];
volatile double PSD_Lp_Y_val[10];
volatile double PSD_Vr_X_val[10];
volatile double PSD_Vr_Y_val[10];
volatile double PSD_Vx_X_val[10];
volatile double PSD_Vx_Y_val[10];


volatile double Vr_set_X_delta_1st[10];
volatile double Vr_set_Y_delta_1st[10];

volatile double Vr_adj_X_cal_1st = 0.0;
volatile double Vr_adj_Y_cal_1st = 0.0;
volatile double Vr_adj_Vpp_cal_1st = 0.0;
volatile double Vr_adj_X_mod_1st = 0.0;
volatile double Vr_adj_Y_mod_1st = 0.0;
volatile double Vr_adj_Vpp_mod_1st = 0.0;
volatile double Vr_adj_phase_mod_1st = 0.0;

volatile double Vr_adj_X_cal_2nd = 0.0;
volatile double Vr_adj_Y_cal_2nd = 0.0;
volatile double Vr_adj_Vpp_cal_2nd = 0.0;
volatile double Vr_adj_X_mod_2nd = 0.0;
volatile double Vr_adj_Y_mod_2nd = 0.0;
volatile double Vr_adj_Vpp_mod_2nd = 0.0;
volatile double Vr_adj_phase_mod_2nd = 0.0;

volatile double coef_matrix_a_X_1st[5];
volatile double coef_matrix_a_Y_1st[5];
volatile double coef_matrix_a_X_2nd[5];
volatile double coef_matrix_a_Y_2nd[5];
volatile double coef_matrix_b_X_2nd[5];
volatile double coef_matrix_b_Y_2nd[5];
volatile double coef_matrix_b_X_1st[5];
volatile double coef_matrix_b_Y_1st[5];

volatile double temp_mul_order_X[5] = {1.0,0.0,0.0,0.0,0.0};
volatile double temp_mul_order_Y[5] = {0.0,0.0,0.0,0.0,0.0};


volatile u8 balanced_1st_dir = 0;
volatile double PSD_Lp_X_none_balanced = 0.0;
volatile double PSD_Lp_Y_none_balanced = 0.0;
volatile double Vr_adj_Vpp_start_2nd = 0.0;
volatile double Vr_adj_phase_start_2nd = 0.0;
volatile double Vr_adj_phase_start_1st = 0.0;
const double Vr_step = 0.25;
extern u8 repeat_aver_compl_cnt_X;
extern u8 repeat_aver_compl_cnt_Y;

volatile double Vr_adj_phase_temp = 0.0;
volatile double Vr_adj_Vpp_temp = 0.0;
volatile double Vr_adj_X_temp = 0.0;
volatile double Vr_adj_Y_temp = 0.0;
volatile double Vr_adj_X_start_2nd = 0.0;
volatile double Vr_adj_Y_start_2nd = 0.0;
volatile double Vr_step_X = 0.0;
volatile double Vr_step_Y = 0.0;

volatile double Lp_X_start_1st;
volatile double Lp_Y_start_1st;

volatile double Vr_adj_phase_start_delta;

volatile double DFT_Lp_Vpp_1st_balanced;
volatile double DFT_Lp_X_1st_balanced;
volatile double DFT_Lp_Y_1st_balanced;


extern u8 Master_switch_flag;
extern u32 IA_CTRL_Analog;
extern Xfloat32 Vx_Vr_freq;
extern u8 sequence_1;
extern u8 sequence_2;
extern u16 Rr;
extern double RAD2DEG;
extern unsigned char balanced_done;							//the flag of balance in 3 division
extern u8 Cal_single_state;
extern u8 moving_aver_cnt_X;
extern u8 moving_aver_cnt_Y;
extern u8 repeat_balance_aver_cnt_X;
extern u8 repeat_balance_aver_cnt_Y;

/* for sweep */
volatile u8 Measure_Mode = NONE_MEASURE_MODE;
volatile u8 Measure_Term = DUT_MEASRUE_TERM;
extern Xfloat32 freq_sweep_array[SWEEP_MAXLINE];
extern Xfloat32 amplitude_sweep_array[SWEEP_MAXLINE];
volatile Xfloat32 *freq_sweep_pointer_temp = freq_sweep_array;
volatile Xfloat32 *amp_sweep_pointer_temp = amplitude_sweep_array;
extern u8 Cal_Rr_single_state;

/*--------  for measure stray Cb  --------*/
volatile Xfloat32 Measure_stray_Cb_freq_array[17] = {10000.0,30000.0,50000.0,100000.0,300000.0,500000.0,800000.0,1000000.0,3000000.0,5000000.0,8000000.0,10000000.0,15000000.0,20000000.0,25000000.0,30000000.0,0.0};
volatile Xfloat32 *Measure_stray_Cb_freq_pointer = Measure_stray_Cb_freq_array;
volatile u8 Measure_stray_Cb_delay_flag = 0;

extern u8  cycles;
extern int DFT_delay_us;
extern unsigned char freq_mode_flag;
extern u8 freq_mixed_flag;
extern u32 point_of_period;
extern u32 Vx_Vr_phase_inc;
extern u32 Vmod_phase_inc;
extern u32 Vref_phase_inc;
extern u32 Vx_gain;
extern Xfloat32 Sweep_mode_Vx_Vr_freq_display;
extern Xfloat32 Sweep_mode_Vx_amp_display;
extern u32 Vx_phase_mod;
extern u32 Vr_phase_mod;
extern u32 Vmod_phase_mod;
extern u32 Vref_phase_mod;
volatile u32 sweep_freq_times = 0;
extern u8 Balance_dft_delay_flag;
extern u8 Workspace;

extern double A_single;
extern double B_single;
extern double C_single;
extern double D_single;
extern double DUT_X;
extern double DUT_Y;
extern u8 aver_A_Single_cnt;
extern u8 aver_B_Single_cnt;
extern u8 aver_C_Single_cnt;
extern u8 aver_D_Single_cnt;

extern u32 Vr_gain;
extern u8 Auto_Rr_state;
extern u8 Auto_Rr_dft_delay_flag;
extern u8 Cal_dft_delay_flag;
extern u8 moving_aver_compl_cnt_X;
extern u8 moving_aver_compl_cnt_Y;

volatile u8 Enable_Auto_Rr_flag = 0;//1 enable Auto Rr, 0 disable Auto Rr.
extern double Stray_Z;
extern double Stray_fitting_coefficient_A;
extern double Stray_fitting_coefficient_B;

/*--------  for measure Rr  --------*/
extern u8 Cal_Rr_dft_delay_flag;

extern u8 Data_interact_ON_flag;//it is the flag that emsures successful interaction with the Labivew.


void Cal_single_high_freq(void);
void Cal_repetitive_high_freq(void);
//void delay_us(int timeus);
void Moving_Aver_X_high_freq(double data, u16 moving_aver_number);
void Moving_Aver_Y_high_freq(double data, u16 moving_aver_number);

void FPGA_Moving_Aver_X_high_freq(double data, u16 moving_aver_number);
void FPGA_Moving_Aver_Y_high_freq(double data, u16 moving_aver_number);

void Balanced(void);	//Tony add
void nonlinear_solve(double *val_X, double *val_Y, double *result_X, double *result_Y, double *coef_X, double *coef_Y, int sovle_dimension);	//Tony add
void complex_product(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
void complex_plus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
void Set_flag_and_Prepare_to_enter_Auto_Rr();
void Set_flag_and_Prepare_to_enter_Balance_Algorithm();
void Set_flag_and_Prepare_to_enter_Measure_stray_C();







//add by jst for debug
volatile double PSD_Vr_X_val__debug_1st[10];
volatile double PSD_Vr_Y_val__debug_1st[10];
volatile double PSD_Vr_X_val__debug_2nd[10];
volatile double PSD_Vr_Y_val__debug_2nd[10];
volatile double PSD_Lp_X_val__debug_1st[10];
volatile double PSD_Lp_Y_val__debug_1st[10];
volatile double PSD_Lp_X_val__debug_2nd[10];
volatile double PSD_Lp_Y_val__debug_2nd[10];
volatile double PSD_Vr_Xita_debug_1st[10];
volatile double PSD_Vr_Xita_debug_2nd[10];
volatile double PSD_Lp_Xita_debug_1st[10];
volatile double PSD_Lp_Xita_debug_2nd[10];
volatile double PSD_Vr_Vpp_debug_1st[10];
volatile double PSD_Vr_Vpp_debug_2nd[10];
volatile double PSD_Lp_Vpp_debug_1st[10];
volatile double PSD_Lp_Vpp_debug_2nd[10];
extern int AD9508_DIVIDER;
extern double AD9552_Freq_out;
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
extern Xfloat32 Vmod_freq;
extern double debug_flag;
extern void 	DDS3_Write();
extern void 	DDS1_Write();
extern void 	DDS2_Write();
extern Xfloat32 AUX_DA1_float;








void Cal_DUT_high_freq(void)
{
		Cal_single_high_freq();
}

//BALANCED ALGORITHM function
void Cal_single_high_freq(void)
{
	u8 cnt_mul = 0;
//	double temp_mul_order_X[5] = {1.0,0.0,0.0,0.0,0.0};
//	double temp_mul_order_Y[5] = {0.0,0.0,0.0,0.0,0.0};
	double temp_mul_res_X = 0.0;
	double temp_mul_res_Y = 0.0;

	switch(high_freq_Cal_single_state)
	{
		/*	Initialize the Vx, set the V ; analog switch to measure Vr	*/
		case 0:
			/*	initial the variable	*/
			Vr_adj_X_cal_1st = 0.0;
			Vr_adj_Y_cal_1st = 0.0;
			Vr_adj_Vpp_cal_1st = 0.0;
			Vr_adj_X_mod_1st = 0.0;
			Vr_adj_Y_mod_1st = 0.0;
			Vr_adj_Vpp_mod_1st = 0.0;

			Vr_adj_X_cal_2nd = 0.0;
			Vr_adj_Y_cal_2nd = 0.0;
			Vr_adj_Vpp_cal_2nd = 0.0;
			Vr_adj_X_mod_2nd = 0.0;
			Vr_adj_Y_mod_2nd = 0.0;
			Vr_adj_Vpp_mod_2nd = 0.0;
        
            
			/*	Set the Vx to default gain : Vx_gain = user_set, Set the Vr to the initial value : Vr_gain = 0*/
//			high_freq_Cal_Vx_gain = (u32) (3 * 13107);
			high_freq_Cal_Vx_gain = Vx_gain;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG18_OFFSET), high_freq_Cal_Vx_gain);

			DDS1_amplitude_factor = (u32)((high_freq_Cal_Vx_gain/5.22/13107)*16383);	//full scale output is 1.044 Vrms when 50R load
			DDS1_ASF[2] = DDS1_amplitude_factor;
			DDS1_ASF[1] = DDS1_amplitude_factor>>8;


			high_freq_Cal_Vr_gain = (u32) (0);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);
			DDS2_ASF[2] = 0x00;
			DDS2_ASF[1] = 0x00;
			DDS1_POW0[1]=0x00;		//default deg=0
			DDS1_POW0[2]=0x00;		//default deg=0
			DDS2_POW0[1]=0x00;		//default deg=0
			DDS2_POW0[2]=0x00;		//default deg=0
			DDS2_Write();
			usleep(100);
			DDS1_Write();
			usleep(100);


			high_freq_Cal_Vr_gain = (u32) (0 * 13107);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);

            /*--------  set the freq used sweep freq mode, wgq add  --------*/
            // Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG15_OFFSET), Vx_Vr_phase_inc);
            // Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG20_OFFSET), Vmod_phase_inc);
            // Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG23_OFFSET), Vref_phase_inc);
            // Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG22_OFFSET), 55650);
            // Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG27_OFFSET), point_of_period);

			/*	analog switch to measure Vr		*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 32;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

			/*	initialize the DDS	 */
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vx
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vr
			usleep(10);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
			usleep(10000);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
			/*	into the 1st balance state	*/
			high_freq_Cal_single_state ++;
			break;

		/*	1st balance : Read the Vr Value from Chan.A &	Read the Lp Value from Chan.B & switch the analog switch to Lp	*/
		case 1:

			/*	Read the PSD result of Vr	*/
			PSD_high_freq_Vr_X_single_0 = PSD_A_X_display;
			PSD_high_freq_Vr_Y_single_0 = -1*PSD_A_Y_display;
			PSD_high_freq_Vr_Vpp_single_0 = PSD_A_Vpp;

			/*	Read the DFT result of Vr	*/
			high_freq_Vr_X_single_0 = DFT_A_X_display;
			high_freq_Vr_Y_single_0 = -1*DFT_A_Y_display;
			high_freq_Vr_Vpp_single_0 = DFT_A_Vpp;

			/*	Read the PSD result of Lp	*/
			PSD_high_freq_Lp_X_single_0 = PSD_B_X_display;
			PSD_high_freq_Lp_Y_single_0 = -1*PSD_B_Y_display;
			PSD_high_freq_Lp_Vpp_single_0 = PSD_B_Vpp;

			/*	Read the DFT result of Lp	*/
			high_freq_Lp_X_single_0 = DFT_B_X_display;
			high_freq_Lp_Y_single_0 = -1*DFT_B_Y_display;
			high_freq_Lp_Vpp_single_0 = DFT_B_Vpp;

			/*	note down the PSD result	*/
			PSD_Lp_X_val[linear_cnt] = PSD_high_freq_Lp_X_single_0;
			PSD_Lp_Y_val[linear_cnt] = PSD_high_freq_Lp_Y_single_0;
			PSD_Vr_X_val[linear_cnt] = PSD_high_freq_Vr_X_single_0;
			PSD_Vr_Y_val[linear_cnt] = PSD_high_freq_Vr_Y_single_0;

			/*	note down the DFT result	*/
			DFT_Lp_X_val[linear_cnt] = high_freq_Lp_X_single_0;
			DFT_Lp_Y_val[linear_cnt] = high_freq_Lp_Y_single_0;
			DFT_Vr_X_val[linear_cnt] = high_freq_Vr_X_single_0;
			DFT_Vr_Y_val[linear_cnt] = high_freq_Vr_Y_single_0;

			Vr_set_X_delta_1st[linear_cnt] = (0.4*linear_cnt);
			Vr_set_Y_delta_1st[linear_cnt] = (0+0 * linear_cnt);

			linear_cnt++;	//count down the linear test time





			//add by jst for debug
			PSD_Vr_Xita_debug_1st[linear_cnt] = atan2(PSD_high_freq_Vr_Y_single_0, PSD_high_freq_Vr_X_single_0) * RAD2DEG;
			PSD_Lp_Xita_debug_1st[linear_cnt] = atan2(PSD_high_freq_Lp_Y_single_0, PSD_high_freq_Lp_X_single_0) * RAD2DEG;
			PSD_Vr_Vpp_debug_1st[linear_cnt] = sqrt(pow(PSD_high_freq_Vr_X_single_0,2.0)+pow(PSD_high_freq_Vr_Y_single_0,2.0));
			PSD_Lp_Vpp_debug_1st[linear_cnt] = sqrt(pow(PSD_high_freq_Lp_X_single_0,2.0)+pow(PSD_high_freq_Lp_Y_single_0,2.0));






			//if sample finish (3 times)
			if(linear_cnt == Solve_Dimension)
			{
				temp_mul_order_X[0] = 1.0;
				temp_mul_order_Y[0] = 0.0;

				/*****	PSD calculate the relationship between delta Vr_AD and delta Lp, the relationship between  delta Vr_AD and delta Vr_DA	*****/
				// nonlinear_solve(DFT_Lp_X_val, DFT_Lp_Y_val, DFT_Vr_X_val, DFT_Vr_Y_val, coef_matrix_a_X_1st, coef_matrix_a_Y_1st, Solve_Dimension);
				nonlinear_solve(PSD_Lp_X_val, PSD_Lp_Y_val, PSD_Vr_X_val, PSD_Vr_Y_val, coef_matrix_a_X_1st, coef_matrix_a_Y_1st, Solve_Dimension);

				for(cnt_mul=0;cnt_mul<Solve_Dimension;cnt_mul++)	//Tony alter: can be optimize
				{
					complex_product(*(coef_matrix_a_X_1st+cnt_mul), *(coef_matrix_a_Y_1st+cnt_mul), *(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), &temp_mul_res_X, &temp_mul_res_Y);
					complex_plus(Vr_adj_X_cal_1st, Vr_adj_Y_cal_1st, temp_mul_res_X, temp_mul_res_Y, &Vr_adj_X_cal_1st, &Vr_adj_Y_cal_1st);
//					complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), -1*(*DFT_Lp_X_val), -1*(*DFT_Lp_Y_val), temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
					complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), 0, 0, temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
				}

				Vr_adj_Vpp_cal_1st = sqrt(pow(Vr_adj_X_cal_1st,2)+pow(Vr_adj_Y_cal_1st,2));
				// Vr_adj_phase_cal_1st = atan2((0-Vr_adj_Y_cal_1st),Vr_adj_X_cal_1st)* RAD2DEG;
				// if(Vr_adj_phase_cal_1st<0)
					// Vr_adj_phase_cal_1st = 360 + Vr_adj_phase_cal_1st;

				temp_mul_order_X[0] = 1.0;
				temp_mul_order_Y[0] = 0.0;

				// nonlinear_solve(DFT_Vr_X_val, DFT_Vr_Y_val, Vr_set_X_delta_1st, Vr_set_Y_delta_1st, coef_matrix_b_X_1st, coef_matrix_b_Y_1st, Solve_Dimension);
				nonlinear_solve(PSD_Vr_X_val, PSD_Vr_Y_val, Vr_set_X_delta_1st, Vr_set_Y_delta_1st, coef_matrix_b_X_1st, coef_matrix_b_Y_1st, Solve_Dimension);
				for(cnt_mul=0;cnt_mul<Solve_Dimension;cnt_mul++)	//Tony alert: can be optimize
				{
					complex_product(*(coef_matrix_b_X_1st+cnt_mul), *(coef_matrix_b_Y_1st+cnt_mul), *(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), &temp_mul_res_X, &temp_mul_res_Y);
					complex_plus(Vr_adj_X_mod_1st, Vr_adj_Y_mod_1st, temp_mul_res_X, temp_mul_res_Y, &Vr_adj_X_mod_1st, &Vr_adj_Y_mod_1st);
					complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), Vr_adj_X_cal_1st, Vr_adj_Y_cal_1st, temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
				}

				//calculate the Vr set value for Lp decrease to zero
				Vr_adj_Vpp_mod_1st = sqrt(pow(Vr_adj_X_mod_1st,2)+pow((0+Vr_adj_Y_mod_1st),2));
				Vr_adj_phase_mod_1st = atan2((0+Vr_adj_Y_mod_1st),Vr_adj_X_mod_1st)* RAD2DEG;
				if(Vr_adj_phase_mod_1st<0)
					Vr_adj_phase_mod_1st = 360 + Vr_adj_phase_mod_1st;

				high_freq_Cal_Vr_gain = (u32) (Vr_adj_Vpp_mod_1st * 13107);
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);	//set the Vr gain
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(Vr_adj_phase_mod_1st * FPGA_PHASE_COE));	//set the Vr phase





				DDS2_amplitude_factor = (u32)((Vr_adj_Vpp_mod_1st/5.22)*16383);	//full scale output is 1.044 Vrms when 50R load
				DDS2_ASF[2] = DDS2_amplitude_factor;
				DDS2_ASF[1] = DDS2_amplitude_factor>>8;

				DDS2_phase_factor=(u32)(Vr_adj_phase_mod_1st*45.5083);
				DDS2_POW0[2] =DDS2_phase_factor;
				DDS2_POW0[1] =DDS2_phase_factor>>8;

				DDS2_Write();
				usleep(100);
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
			    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vx
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vr
			    usleep(10);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
                usleep(10000);
    			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));



























				/*	into the 2nd balance state	*/
				linear_cnt = 0;
				high_freq_Cal_single_state++;
				//note down the none balanced Lp
				PSD_Lp_X_none_balanced = PSD_Lp_X_val[0];
				PSD_Lp_Y_none_balanced = PSD_Lp_Y_val[0];
				//note down the none balanced Lp of dft
				// DFT_Lp_X_none_balanced = DFT_Lp_X_val[0];
				// DFT_Lp_Y_none_balanced = DFT_Lp_Y_val[0];

			}


			else
			{
				/*	refresh the Vr : Vr_gain = 0.75*n ; Vr_phase = 45งางๅ+90งางๅ*n	*/
				high_freq_Cal_Vr_gain = (u32) ((0+3/(Solve_Dimension-1)*linear_cnt) * 13107);
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);










				DDS2_amplitude_factor = (u32)((high_freq_Cal_Vr_gain/13107/5.22)*16383*0.4);	//full scale output is 1.044 Vrms when 50R load
				DDS2_ASF[2] = DDS2_amplitude_factor;
				DDS2_ASF[1] = DDS2_amplitude_factor>>8;
				DDS2_POW0[2] = 0x00;
				DDS2_POW0[1] = 0x00;
				/***** DDS configuration *****/
				DDS2_Write();
				usleep(50);



















				/*	initialize the DDS	 */
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vx
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vr
				usleep(10);
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
				usleep(10000);
				Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));

				//usleep(1000);
				//keep the state in the sample state
				high_freq_Cal_single_state = high_freq_Cal_single_state;
				/*	end of the balance	*/


			}

			break;

			/*	2nd balance : Read the Vr Value from Chan.A &	Read the Lp Value from Chan.B & switch the analog switch to Lp	*/
			case 2:

				/*	Read the PSD result of Vr	*/
				PSD_high_freq_Vr_X_single_0 = PSD_A_X_display;
				PSD_high_freq_Vr_Y_single_0 = -1*PSD_A_Y_display;
				PSD_high_freq_Vr_Vpp_single_0 = PSD_A_Vpp;

				/*	Read the DFT result of Vr	*/
				high_freq_Vr_X_single_0 = DFT_A_X_display;
				high_freq_Vr_Y_single_0 = -1*DFT_A_Y_display;
				high_freq_Vr_Vpp_single_0 = DFT_A_Vpp;

				/*	Read the PSD result of Lp	*/
				PSD_high_freq_Lp_X_single_0 = PSD_B_X_display;
				PSD_high_freq_Lp_Y_single_0 = -1*PSD_B_Y_display;
				PSD_high_freq_Lp_Vpp_single_0 = PSD_B_Vpp;

				/*	Read the DFT result of Lp	*/
				high_freq_Lp_X_single_0 = DFT_B_X_display;
				high_freq_Lp_Y_single_0 = -1*DFT_B_Y_display;
				high_freq_Lp_Vpp_single_0 = DFT_B_Vpp;

				/*	note down the PSD result	*/
				PSD_Lp_X_val[linear_cnt] = PSD_high_freq_Lp_X_single_0;
				PSD_Lp_Y_val[linear_cnt] = PSD_high_freq_Lp_Y_single_0;
				PSD_Vr_X_val[linear_cnt] = PSD_high_freq_Vr_X_single_0;
				PSD_Vr_Y_val[linear_cnt] = PSD_high_freq_Vr_Y_single_0;

				/*	note down the DFT result	*/
				DFT_Lp_X_val[linear_cnt] = high_freq_Lp_X_single_0;
				DFT_Lp_Y_val[linear_cnt] = high_freq_Lp_Y_single_0;
				DFT_Vr_X_val[linear_cnt] = high_freq_Vr_X_single_0;
				DFT_Vr_Y_val[linear_cnt] = high_freq_Vr_Y_single_0;

				Vr_set_X_delta_1st[linear_cnt] = Vr_adj_X_mod_1st + linear_cnt*Vr_step_X;
				Vr_set_Y_delta_1st[linear_cnt] = Vr_adj_Y_mod_1st + linear_cnt*Vr_step_Y;

				linear_cnt++;	//count down the linear test time
















				//add by jst for debug
				PSD_Vr_Xita_debug_2nd[linear_cnt] = atan2(PSD_high_freq_Vr_Y_single_0, PSD_high_freq_Vr_X_single_0) * RAD2DEG;
				PSD_Lp_Xita_debug_2nd[linear_cnt] = atan2(PSD_high_freq_Lp_Y_single_0, PSD_high_freq_Lp_X_single_0) * RAD2DEG;
				PSD_Vr_Vpp_debug_2nd[linear_cnt]=sqrt(pow(PSD_high_freq_Vr_X_single_0,2.0)+pow(PSD_high_freq_Vr_Y_single_0,2.0));
				PSD_Lp_Vpp_debug_2nd[linear_cnt]=sqrt(pow(PSD_high_freq_Lp_X_single_0,2.0)+pow(PSD_high_freq_Lp_Y_single_0,2.0));
























				//if sample finish (4 times)
				if(linear_cnt == (Solve_Dimension-1))
				{
					temp_mul_order_X[0] = 1.0;
					temp_mul_order_Y[0] = 0.0;

					/*****	PSD calculate the relationship between delta Vr_AD and delta Lp, the relationship between  delta Vr_AD and delta Vr_DA	*****/
					// nonlinear_solve(DFT_Lp_X_val, DFT_Lp_Y_val, DFT_Vr_X_val, DFT_Vr_Y_val, coef_matrix_a_X_2nd, coef_matrix_a_Y_2nd, (Solve_Dimension-1));
					nonlinear_solve(PSD_Lp_X_val, PSD_Lp_Y_val, PSD_Vr_X_val, PSD_Vr_Y_val, coef_matrix_a_X_2nd, coef_matrix_a_Y_2nd, (Solve_Dimension-1));
					for(cnt_mul=0;cnt_mul<Solve_Dimension-1;cnt_mul++)	//Tony alter: can be optimize
					{
						complex_product(*(coef_matrix_a_X_2nd+cnt_mul), *(coef_matrix_a_Y_2nd+cnt_mul), *(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), &temp_mul_res_X, &temp_mul_res_Y);
						complex_plus(Vr_adj_X_cal_2nd, Vr_adj_Y_cal_2nd, temp_mul_res_X, temp_mul_res_Y, &Vr_adj_X_cal_2nd, &Vr_adj_Y_cal_2nd);
//						complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), -1*(*PSD_Lp_X_val), -1*(*PSD_Lp_Y_val), temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
						complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), 0, 0, temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
					}

					Vr_adj_Vpp_cal_2nd = sqrt(pow(Vr_adj_X_cal_2nd,2)+pow(Vr_adj_Y_cal_2nd,2));

					temp_mul_order_X[0] = 1.0;
					temp_mul_order_Y[0] = 0.0;

					// nonlinear_solve(DFT_Vr_X_val, DFT_Vr_Y_val, Vr_set_X_delta_1st, Vr_set_Y_delta_1st, coef_matrix_b_X_2nd, coef_matrix_b_Y_2nd, (Solve_Dimension-1));
					nonlinear_solve(PSD_Vr_X_val, PSD_Vr_Y_val, Vr_set_X_delta_1st, Vr_set_Y_delta_1st, coef_matrix_b_X_2nd, coef_matrix_b_Y_2nd, (Solve_Dimension-1));
					for(cnt_mul=0;cnt_mul<Solve_Dimension-1;cnt_mul++)	//Tony alter: can be optimize
					{
						complex_product(*(coef_matrix_b_X_2nd+cnt_mul), *(coef_matrix_b_Y_2nd+cnt_mul), *(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), &temp_mul_res_X, &temp_mul_res_Y);
						complex_plus(Vr_adj_X_mod_2nd, Vr_adj_Y_mod_2nd, temp_mul_res_X, temp_mul_res_Y, &Vr_adj_X_mod_2nd, &Vr_adj_Y_mod_2nd);
						complex_product(*(temp_mul_order_X+cnt_mul), *(temp_mul_order_Y+cnt_mul), Vr_adj_X_cal_2nd, Vr_adj_Y_cal_2nd, temp_mul_order_X+cnt_mul+1, temp_mul_order_Y+cnt_mul+1);
					}

//					Vr_adj_X_mod_2nd = Vr_adj_X_start_2nd + Vr_adj_X_mod_2nd;
//					Vr_adj_Y_mod_2nd = Vr_adj_Y_start_2nd + Vr_adj_Y_mod_2nd;
					Vr_adj_X_mod_2nd = Vr_adj_X_mod_2nd;
					Vr_adj_Y_mod_2nd = Vr_adj_Y_mod_2nd;
					Vr_adj_Vpp_mod_2nd = sqrt(pow(Vr_adj_X_mod_2nd,2)+pow(Vr_adj_Y_mod_2nd,2));
					Vr_adj_phase_mod_2nd = atan2(Vr_adj_Y_mod_2nd,Vr_adj_X_mod_2nd)* RAD2DEG;

					high_freq_Cal_Vr_gain = (u32) (Vr_adj_Vpp_mod_2nd * 13107);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);
















					DDS2_amplitude_factor = (u32)((Vr_adj_Vpp_mod_2nd/5.22)*16383);	//full scale output is 1.044 Vrms when 50R load
					DDS2_ASF[2] = DDS2_amplitude_factor;
					DDS2_ASF[1] = DDS2_amplitude_factor>>8;




















					if(Vr_adj_phase_mod_2nd<0)
						Vr_adj_phase_mod_2nd = 360 + Vr_adj_phase_mod_2nd;
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(Vr_adj_phase_mod_2nd * FPGA_PHASE_COE));










					DDS2_phase_factor=(u32)(Vr_adj_phase_mod_2nd*45.5083);
					DDS2_POW0[2] =DDS2_phase_factor;
					DDS2_POW0[1] =DDS2_phase_factor>>8;
					/***** DDS configuration *****/
					DDS2_Write();
					//usleep(100);



					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
					usleep(10);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
					usleep(10000);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));
					//usleep(1000);







					/*	end of the balance	*/
					linear_cnt = 0;
					high_freq_Cal_single_state = 0;


                    switch (Measure_Term) {
                        case DUT_MEASRUE_TERM:
                            /*--------  mesure Vx, for cal_dut  --------*/
                            IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
                            IA_CTRL_Analog += 0;
                            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
                            Workspace = CAL_DUT;
                            Master_switch_flag = 1;
                            Cal_dft_delay_flag = 0;
                            break;
                        case Rr_MEASRUE_TERM:
                            /*--------  mesure Vx, for cal_Rr  --------*/
                            IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
                            IA_CTRL_Analog += 0;
                            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
                            Workspace = CAL_Rr;
                            Master_switch_flag = 1;
                            Cal_Rr_dft_delay_flag = 0;
                            break;
                        default:
                            break;
                    }
				}

				else
				{
        			/*****	PSD result to determine the direction of 2nd balance	*****/
        			if(PSD_Lp_X_none_balanced>=0)
        			{
        				if(PSD_Lp_Y_none_balanced>=0)
        					balanced_1st_dir = 1;
        				else
        					balanced_1st_dir = 4;
        			}
        			else
        			{
        				if(PSD_Lp_Y_none_balanced>=0)
        					balanced_1st_dir = 2;
        				else
        					balanced_1st_dir = 3;
        			}

        			//determine the Vr begin phase
        			switch(balanced_1st_dir)
        			{
        				//1st quadrant
        				case 1:
        					if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]>=0))	//1st quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st;
        					else if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]<0))	//4th quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 90;
        					else if((PSD_Lp_X_val[0]<0) && (PSD_Lp_Y_val[0]<0))	//3rd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 180;
        					else	//2nd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 270;
        				break;

        				//2nd quadrant
        				case 2:
        					if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]>=0))	//1st quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 90;
        					else if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]<0))	//4th quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 180;
        					else if((PSD_Lp_X_val[0]<0) && (PSD_Lp_Y_val[0]<0))	//3rd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 270;
        					else	//2nd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st;
        				break;

        				//3rd quadrant
        				case 3:
        					if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]>=0))	//1st quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 180;
        					else if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]<0))	//4th quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 270;
        					else if((PSD_Lp_X_val[0]<0) && (PSD_Lp_Y_val[0]<0))	//3rd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st;
        					else	//2nd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 90;
        				break;

        				//4th quadrant
        				case 4:
        					if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]>=0))	//1st quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 270;
        					else if((PSD_Lp_X_val[0]>=0) && (PSD_Lp_Y_val[0]<0))	//4th quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st;
        					else if((PSD_Lp_X_val[0]<0) && (PSD_Lp_Y_val[0]<0))	//3rd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 90;
        					else	//2nd quadrant
        						Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st - 180;
        				break;

        				default:
        					Vr_adj_phase_start_2nd = Vr_adj_phase_mod_1st;
        				break;
        			}

        			if(Vr_adj_phase_start_2nd<0)
        				Vr_adj_phase_start_2nd = Vr_adj_phase_start_2nd + 360;
        			else if(Vr_adj_phase_start_2nd>360)
        				Vr_adj_phase_start_2nd = Vr_adj_phase_start_2nd - 360;

        			//determine the Vr step value
        			Vr_step_X = Vr_step * cos(Vr_adj_phase_start_2nd/RAD2DEG);
        			Vr_step_Y = Vr_step * sin(Vr_adj_phase_start_2nd/RAD2DEG);

        			//determine the Vr begin value and phase
                       //Vr_adj_phase_start_2nd = Vr_adj_phase_start_1st;	//Tony add for test
        			Vr_adj_X_temp = Vr_adj_X_mod_1st + linear_cnt*Vr_step_X;
        			Vr_adj_Y_temp = Vr_adj_Y_mod_1st + linear_cnt*Vr_step_Y;
					Vr_adj_Vpp_temp = sqrt(pow(Vr_adj_X_temp,2)+pow(Vr_adj_Y_temp,2));
					Vr_adj_phase_temp = atan2(Vr_adj_Y_temp,Vr_adj_X_temp)* RAD2DEG;
					if(Vr_adj_phase_temp<0)
						Vr_adj_phase_temp = Vr_adj_phase_temp + 360;

					//Set the Vr amplitude
					high_freq_Cal_Vr_gain = (u32) (Vr_adj_Vpp_temp * 13107);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), high_freq_Cal_Vr_gain);










					DDS2_amplitude_factor = (u32)((Vr_adj_Vpp_temp/5.22)*16383);
										DDS2_ASF[2] = DDS2_amplitude_factor;
										DDS2_ASF[1] = DDS2_amplitude_factor>>8;
										DDS2_phase_factor=(u32)(Vr_adj_phase_temp*45.5083);
										DDS2_POW0[2] =DDS2_phase_factor;
										DDS2_POW0[1] =DDS2_phase_factor>>8;
										/***** DDS configuration *****/
										DDS2_Write();
										usleep(100);




















					/*	initialize the DDS	 */
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), (u32)(0 * FPGA_PHASE_COE));	//Tony add for Vx
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(Vr_adj_phase_temp * FPGA_PHASE_COE));	//Tony add for Vr
					usleep(10);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
					usleep(10000);
					Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));

					//usleep(1000);
					//keep the state in the sample state
					high_freq_Cal_single_state = high_freq_Cal_single_state;
				}

				break;
		default:
			high_freq_Cal_single_state = 0;
			break;
	}
    Balance_dft_delay_flag = 0;
}

/*Cal_initialize function*/
void Cal_initialize(void)
{
    /*--------  init for Auto Rr  --------*/
    Auto_Rr_state = 0;
    
    /*--------  init for balance algorithm  --------*/
    high_freq_Cal_single_state = 0;
    sweep_freq_times = 0;
    Sweep_mode_Vx_Vr_freq_display = 0.0;
    Sweep_mode_Vx_amp_display = 0.0;

    /*--------  init for cal  --------*/
	Cal_single_state = 0;
    Cal_Rr_single_state = 0;
    //moving_aver_cnt_X = 0;
    //moving_aver_cnt_Y = 0;
    aver_A_Single_cnt = 0;
    aver_B_Single_cnt = 0;
    aver_C_Single_cnt = 0;
    aver_D_Single_cnt = 0;
    moving_aver_compl_cnt_X = 0;
    moving_aver_compl_cnt_Y = 0;

    //for sweep
    freq_sweep_pointer_temp = freq_sweep_array;
    amp_sweep_pointer_temp = amplitude_sweep_array;

    //for Data_interact
    Data_interact_ON_flag = 0;
}

/*Repeat_balance_cal_initialize function*/
void Repeat_balance_cal_initialize(void)
{
    /*--------  init for Auto Rr  --------*/
    Auto_Rr_state = 0;
    
    /*--------  init for balance algorithm  --------*/
    high_freq_Cal_single_state = 0;
    sweep_freq_times = 0;
    Sweep_mode_Vx_Vr_freq_display = 0.0;
    Sweep_mode_Vx_amp_display = 0.0;

    /*--------  init for cal  --------*/
	Cal_single_state = 0;
    Cal_Rr_single_state = 0;
    //moving_aver_cnt_X = 0;
    //moving_aver_cnt_Y = 0;
    aver_A_Single_cnt = 0;
    aver_B_Single_cnt = 0;
    aver_C_Single_cnt = 0;
    aver_D_Single_cnt = 0;
    moving_aver_compl_cnt_X = 0;
    moving_aver_compl_cnt_Y = 0;
    /*--------  init for balance many times, but now is not used  --------*/
    repeat_balance_aver_cnt_X = 0;
    repeat_balance_aver_cnt_Y = 0;
    repeat_aver_compl_cnt_X = 0;
    repeat_aver_compl_cnt_Y = 0;

    //for sweep
    freq_sweep_pointer_temp = freq_sweep_array;
    amp_sweep_pointer_temp = amplitude_sweep_array;
}

void determine_work_mode(){
    //measure mode
    switch (Measure_Mode) {
        case SINGLE_MEASURE_MODE:

            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);
            usleep(1000);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
            usleep(10000);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));

            if(Enable_Auto_Rr_flag){
                Set_flag_and_Prepare_to_enter_Auto_Rr();
            }
            else
            {
                switch (Measure_Term) {
                    case DUT_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                        break;
                    case Rr_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                        break;
                    case STRAY_C_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Measure_stray_C();
                        break;
                    default:
                        break;
                }
            }
            break;
        case REPEAT_MEASURE_MODE:

            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);
//            usleep(2);
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
            usleep(10000);
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));


            if(Enable_Auto_Rr_flag){
                Set_flag_and_Prepare_to_enter_Auto_Rr();
            }
            else
            {
                switch (Measure_Term) {
                    case DUT_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                        break;
                    case Rr_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                        break;
                    case STRAY_C_MEASRUE_TERM:
                        Set_flag_and_Prepare_to_enter_Measure_stray_C();
                        break;
                    default:
                        break;
                }
            }
            break;
        case FREQ_SWEEP_MEASURE_MODE:
            if (*freq_sweep_pointer_temp != 0.0) {
                Vx_Vr_freq = *freq_sweep_pointer_temp;

                /*--------  record Stray_Z in current freq  --------*/
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
                        Vref_phase_inc = Vx_Vr_phase_inc;									
                        Vmod_phase_inc = 0;
                        point_of_period = 1000000.0 / Vx_Vr_freq * cycles;
                        DFT_delay_us = 1000000.0 / Vx_Vr_freq * cycles * 3;
                        break;

                        /*			 mixed Frequency		*/
                    case 1:
                        Vx_Vr_phase_inc = (Vx_Vr_freq * FPGA_FREQ_COE);
                        Vref_phase_inc = 30924;									// fixed the freq of Vref to 1.8KHz;
                        Vmod_phase_inc = Vx_Vr_phase_inc + 30924;				// fixed the freq of Vmod, freq_Vmod = freq_Vx + 1.8KHz
                        point_of_period = 1000000.0 / 1800.0 * cycles;
                        DFT_delay_us = 1000000.0 / 1800.0 * cycles * 3;
                        break;

                    default:
                        Vx_Vr_phase_inc = (Vx_Vr_freq * FPGA_FREQ_COE);
                        Vref_phase_inc = 30924;									// fixed the freq of Vref to 1.8KHz;
                        Vmod_phase_inc = Vx_Vr_phase_inc + 30924;				// fixed the freq of Vmod, freq_Vmod = freq_Vx + 1.8KHz
                        point_of_period = (u32)(1000000.0 / 1800.0 * cycles);
                        DFT_delay_us = 1000000.0 / Vx_Vr_freq * cycles * 3;
                        break;
                }





                //add by jst
    			DDS1_DDS2_freq_tunning_word = (Vx_Vr_freq/AD9552_Freq_out*(AD9508_DIVIDER+1))*pow(2,32);
    			DDS1_FTW0[4] = DDS1_DDS2_freq_tunning_word;
    			DDS1_FTW0[3] = DDS1_DDS2_freq_tunning_word>>8;
    			DDS1_FTW0[2] = DDS1_DDS2_freq_tunning_word>>16;
    			DDS1_FTW0[1] = DDS1_DDS2_freq_tunning_word>>24;


    			DDS2_FTW0[4] = DDS1_FTW0[4];
    			DDS2_FTW0[3] = DDS1_FTW0[3];
    			DDS2_FTW0[2] = DDS1_FTW0[2];
    			DDS2_FTW0[1] = DDS1_FTW0[1];

    			DDS1_POW0[1]=0x00;
    			DDS1_POW0[2]=0x00;
    			DDS2_POW0[1]=0x00;
    			DDS2_POW0[2]=0x00;

    			DDS1_Write();
    			usleep(10);
    			DDS2_Write();
    			usleep(10);


    			DDS3_POW0[1]=0x00;
    			DDS3_POW0[2]=0x00;

    			DDS3_freq_tunning_word = (Vmod_phase_inc/17.179869184/AD9552_Freq_out*(AD9508_DIVIDER+1))*pow(2,32);
    			DDS3_FTW0[4] = DDS3_freq_tunning_word;
    			DDS3_FTW0[3] = DDS3_freq_tunning_word>>8;
    			DDS3_FTW0[2] = DDS3_freq_tunning_word>>16;
    			DDS3_FTW0[1] = DDS3_freq_tunning_word>>24;
    			DDS3_Write();
    			usleep(10);




                /*--------  output the freq  --------*/
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG15_OFFSET), Vx_Vr_phase_inc);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG20_OFFSET), Vmod_phase_inc);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG23_OFFSET), Vref_phase_inc);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG27_OFFSET), point_of_period);

                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);
                usleep(10);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
                usleep(10000);
    			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));


                if(Enable_Auto_Rr_flag){
                    Set_flag_and_Prepare_to_enter_Auto_Rr();
                }
                else
                {
                    switch (Measure_Term) {
                        case DUT_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                            break;
                        case Rr_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                            break;
                        case STRAY_C_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Measure_stray_C();
                            break;
                        default:
                            break;
                    }
                }
            }
            else
            {
                Master_switch_flag = 0;
                Workspace = NULL_WORKSPACE;
                Measure_Mode = NONE_MEASURE_MODE;
                Balance_dft_delay_flag = 0;
                for (freq_sweep_pointer_temp = freq_sweep_array ; *freq_sweep_pointer_temp != 0.0 ; freq_sweep_pointer_temp++ ) {
                    *freq_sweep_pointer_temp = 0.0;
                }
                freq_sweep_pointer_temp = freq_sweep_array;
            }
            break;
        case AMP_SWEEP_MEASURE_MODE:
            if (*amp_sweep_pointer_temp != 0.0) {
                Vx_gain = (u32)((*amp_sweep_pointer_temp) * FPGA_GAIN_COE);

                /*--------  output the Vx_gain  --------*/
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG18_OFFSET), Vx_gain);

                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), Vx_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), Vr_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG21_OFFSET), Vmod_phase_mod);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG24_OFFSET), Vref_phase_mod);
//                usleep(2);
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(0));
                usleep(10000);
    			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));



                if(Enable_Auto_Rr_flag){
                    Set_flag_and_Prepare_to_enter_Auto_Rr();
                }
                else
                {
                    switch (Measure_Term) {
                        case DUT_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                            break;
                        case Rr_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                            break;
                        case STRAY_C_MEASRUE_TERM:
                            Set_flag_and_Prepare_to_enter_Measure_stray_C();
                            break;
                        default:
                            break;
                    }
                }
            }
            else
            {
                Master_switch_flag = 0;
                Workspace = NULL_WORKSPACE;
                Measure_Mode = NONE_MEASURE_MODE;
                Balance_dft_delay_flag = 0;
                for (amp_sweep_pointer_temp = amplitude_sweep_array ; *amp_sweep_pointer_temp != 0.0 ; amp_sweep_pointer_temp++ ) {
                    *amp_sweep_pointer_temp = 0.0;
                }
                amp_sweep_pointer_temp = amplitude_sweep_array;
            }           
            break;
        default:
            
            break;
    }
}

void Set_flag_and_Prepare_to_enter_Auto_Rr()
{
    Workspace = AUTO_Rr;
    //Vr=0
    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG19_OFFSET), 0);
    //set Rr=5k
    IA_CTRL_Analog = IA_CTRL_Analog & 0x00000ee3;
    IA_CTRL_Analog += 4;
    Rr = 5000.0;
    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
    /*--------  measure Vx  --------*/
    IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
    IA_CTRL_Analog += 0;
    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
    Auto_Rr_dft_delay_flag = 0;
}

void Set_flag_and_Prepare_to_enter_Balance_Algorithm()
{
    Workspace = BALANCED_ALGORITHM;
    Balance_dft_delay_flag = 0;
}

void Set_flag_and_Prepare_to_enter_Measure_stray_C()
{
    Workspace = MEASURE_STRAY_C;
    /*--------  measure Vx  --------*/
    IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
    IA_CTRL_Analog += 0;
    Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
    Measure_stray_Cb_delay_flag = 0;
}
