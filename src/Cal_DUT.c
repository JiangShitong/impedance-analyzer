/*
 * Cal_DUT.c
 *
 *  Created on: 2019年1月25日
 *      Author: Administrator
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "xbasic_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "ARM_interface.h"
#include "UART.h"
#include <sleep.h>

#define MOVING_NUM 1000
#define SINGLE_CAL_SWEEP_MODE_AVER_NUM 1//aver Dut_XY or Rr_XY in sweep mode or single mode
#define REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT 5//5 points generate one aver point. refer 整机调试----魏国权,多次平衡的示意图
volatile u8 AVER_ABCD_SINGLE_POINT = 10;//average ABCD_singl points

volatile double Vx_X_single = 0.0;
volatile double Vx_Y_single = 0.0;
volatile double Vr_X_single = 0.0;
volatile double Vr_Y_single = 0.0;
volatile double Lp_X_single = 0.0;
volatile double Lp_Y_single = 0.0;
volatile double Fit_AD_Vx_X_single = 0.0;
volatile double Fit_AD_Vx_Y_single = 0.0;
volatile double Fit_AD_Vr_X_single = 0.0;
volatile double Fit_AD_Vr_Y_single = 0.0;
volatile double Fit_AD_Lp_X_single = 0.0;
volatile double Fit_AD_Lp_Y_single = 0.0;

volatile double A_single = 0.0;
volatile double B_single = 0.0;
volatile double C_single = 0.0;
volatile double D_single = 0.0;
volatile double Average_A_single = 0.0;
volatile double Average_B_single = 0.0;
volatile double Average_C_single = 0.0;
volatile double Average_D_single = 0.0;
volatile u8 aver_A_Single_cnt = 0;
volatile u8 aver_B_Single_cnt = 0;
volatile u8 aver_C_Single_cnt = 0;
volatile u8 aver_D_Single_cnt = 0;
volatile double aver_sum_A_Single = 0.0;
volatile double aver_sum_B_Single = 0.0;
volatile double aver_sum_C_Single = 0.0;
volatile double aver_sum_D_Single = 0.0;
volatile double A_Single_array[200] = {0.0};
volatile double B_Single_array[200] = {0.0};
volatile double C_Single_array[200] = {0.0};
volatile double D_Single_array[200] = {0.0};

volatile double Coef_X = 0.0;
volatile double Coef_Y = 0.0;

//volatile double DUT_single_X = 0.0;
//volatile double DUT_single_Y = 0.0;

volatile double Vx_X_rep = 0.0;
volatile double Vx_Y_rep = 0.0;
volatile double Vr_X_rep = 0.0;
volatile double Vr_Y_rep = 0.0;
volatile double Lp_X_rep = 0.0;
volatile double Lp_Y_rep = 0.0;

volatile double A_rep = 0.0;
volatile double B_rep= 0.0;
volatile double C_rep= 0.0;
volatile double D_rep = 0.0;

volatile double numer_X_rep = 0.0;
volatile double numer_Y_rep = 0.0;
volatile double denom_rep = 0.0;

//volatile double DUT_rep_X = 0.0;
//volatile double DUT_rep_Y = 0.0;

volatile double DUT_X = 0.0;
volatile double DUT_Y = 0.0;
volatile long double Rr_X = 0.0;
volatile long double Rr_Y = 0.0;
volatile double DUT_display_fitting_1 = 0.0;
volatile double DUT_display_fitting_2 = 0.0;
volatile double DUT_display_1 = 0.0;
volatile double DUT_display_2 = 0.0;
volatile u8 Cal_single_state = 0;
volatile u8 Cal_Rr_single_state = 0;
volatile u8 Cal_rep_state = 0;
volatile u32 Cal_Vr_gain = 0;
volatile u32 Cal_Vr_phase = 0;

volatile u8 moving_aver_compl_cnt_X = 0;
volatile u8 repeat_aver_compl_cnt_X = 0;
volatile u8 moving_aver_compl_cnt_Y = 0;
volatile u8 repeat_aver_compl_cnt_Y = 0;
volatile u8 moving_aver_cnt_X =  0;
volatile u8 repeat_balance_aver_cnt_X =  0;
volatile double moving_aver_temp_X[MOVING_NUM] = {0.0};
volatile double repeat_balance_temp_X[REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT] = {0.0};//array length is 8
volatile double moving_aver_sum_X = 0.0;
volatile double repeat_aver_sum_X = 0.0;
volatile double moving_aver_result_X = 0.0;
volatile double repeat_aver_result_X = 0.0;
volatile u8 moving_aver_cnt_Y =  0;
volatile u8 repeat_balance_aver_cnt_Y =  0;
volatile double moving_aver_temp_Y[MOVING_NUM] = {0.0};
volatile double repeat_balance_temp_Y[REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT] = {0.0};
volatile double moving_aver_sum_Y = 0.0;
volatile double repeat_aver_sum_Y = 0.0;
volatile double moving_aver_result_Y = 0.0;
volatile double repeat_aver_result_Y = 0.0;
volatile u8 repeat_balance_points = 2;//the min is 2

volatile u8 multi_aver_number = 0;
volatile double multi_aver_result_X = 0.0;
volatile double multi_aver_sum_X = 0.0;
volatile double multi_aver_result_Y = 0.0;
volatile double multi_aver_sum_Y = 0.0;
volatile u16 Cal_cnt = 0;
volatile double Cal_Vx_X_array[1000] = {0.0};
volatile double Cal_Vx_Y_array[1000] = {0.0};
volatile double Cal_Vr_X_array[1000] = {0.0};
volatile double Cal_Vr_Y_array[1000] = {0.0};
volatile double Cal_Lp_X_array[1000] = {0.0};
volatile double Cal_Lp_Y_array[1000] = {0.0};
volatile double Dut_array[1000] = {0.0};

extern double DFT_A_X_display;
extern double DFT_A_Y_display;
extern double PSD_B_X_display;
extern double PSD_B_Y_display;
extern u8 Master_switch_flag;
extern u32 IA_CTRL_Analog;
extern Xfloat32 Vx_Vr_freq;
extern u8 sequence_1;
extern u8 sequence_2;
extern u16 Rr;
extern u32 Vr_gain;
extern u32 Vr_phase_mod;
extern double RAD2DEG;
//extern u32 IA_CTRL_Analog;
extern u8 freq_mixed_flag;

extern double PSD_A_X_display;
extern double PSD_A_Y_display;

extern double Cal_PSD_A_X_display;
extern double Cal_PSD_A_Y_display;
extern double Cal_PSD_B_X_display;
extern double Cal_PSD_B_Y_display;

extern u8 Balance_dft_delay_flag;
extern u8 Master_switch_flag;
extern Xfloat32 *freq_sweep_pointer_temp;
extern Xfloat32 *amp_sweep_pointer_temp;
extern u8 Workspace;
extern u8 Cal_dft_delay_flag;
extern u8 Cal_Rr_dft_delay_flag;
volatile u8 Data_interact_ON_flag = 0;//it is the flag that emsures successful interaction with the Labivew.

volatile double Vx_Vpp, Vr_Vpp, Lp_Vpp;
volatile double Vx_Vrms, Vr_Vrms, Lp_Vrms;
volatile double Vx_Xita, Vr_Xita, Lp_Xita;
extern double Cal_PSD_A_Vpp;
extern double Cal_PSD_A_Vrms;
extern double Cal_PSD_B_Vrms;

extern double Stray_Z;

extern u32 Data_interact_ON_cnt;//for send FLAG time out
extern u8 Measure_Mode;

extern Xfloat32 Sweep_mode_Vx_Vr_freq_display;
extern Xfloat32 Sweep_mode_Vx_amp_display;
extern u32 Vx_gain;
extern u32 sweep_freq_times;





extern double debug_flag;





void Cal_single(void);
void Cal_rep(void);
//void delay_us(int timeus);
double ParameterFittingAttributes(double X, double Y, Xfloat32 freq, u8 sequence, u16 Rr);
double ParameterAttributes(double X, double Y, Xfloat32 freq, u8 sequence, u16 Rr);
void Moving_Aver_X(double data);
void Moving_Aver_Y(double data);
void Multi_Aver_Y(double data, u8 aver_number);
void Multi_Aver_Y(double data, u8 aver_number);
extern void Send_Data2PC_str(char *str);
void Aver_A_Single(double data);
double Round_retained_three_decimal(double num_of_double);
void Repeat_balance_no_moving_Aver_X(double data);
void Repeat_balance_no_moving_Aver_Y(double data);
extern void complex_product(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_divide(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_plus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_minus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
void Cal_Rr_moving_aver_initialize(void);
void Cal_moving_aver_initialize(void);

/*Cal Dut funtion*/
void Cal_single(void)
{
    long double Rr_Y_0, Rr_Y_1, Rr_Y_2, Rr_Y_3, Rr_Y_4, Rr_Y_5, Rr_Y_6, Rr_Y_7, Rr_Y_8, Rr_Y_9, Rr_Y_10, Rr_Y_11, Rr_Y_12, Rr_Y_13, Rr_Y_14;
    long double Rr_X_0, Rr_X_1, Rr_X_2, Rr_X_3, Rr_X_4, Rr_X_5, Rr_X_6, Rr_X_7, Rr_X_8, Rr_X_9, Rr_X_10, Rr_X_11, Rr_X_12, Rr_X_13;
    long double Fit_AD_Vrms, Fit_AD_Xita;
    long double Fit_AD_RTerm_0, Fit_AD_RTerm_1, Fit_AD_RTerm_2, Fit_AD_RTerm_3, Fit_AD_RTerm_4, Fit_AD_RTerm_5, Fit_AD_RTerm_6, Fit_AD_RTerm_7, Fit_AD_RTerm_8, Fit_AD_RTerm_9, Fit_AD_RTerm_10, Fit_AD_RTerm_11, Fit_AD_RTerm;
    long double Fit_AD_XitaTerm_0, Fit_AD_XitaTerm_1, Fit_AD_XitaTerm_2, Fit_AD_XitaTerm_3, Fit_AD_XitaTerm_4, Fit_AD_XitaTerm_5, Fit_AD_XitaTerm_6, Fit_AD_XitaTerm_7, Fit_AD_XitaTerm_8, Fit_AD_XitaTerm_9, Fit_AD_XitaTerm_10, Fit_AD_XitaTerm_11, Fit_AD_XitaTerm;
    long double Dut_Z_module;
	switch(Cal_single_state)
	{
		/*	Read the Vx		*/
		case 0:
			//the DFT result
			// Vx_X_single = DFT_A_X_display;
			// Vx_Y_single = -1.0 * DFT_A_Y_display;

			//the PSD result
            Vx_Vrms = Cal_PSD_A_Vrms;
            Vx_Vpp = Cal_PSD_A_Vpp;
            if(freq_mixed_flag)
            {
			    Vx_X_single = Cal_PSD_A_X_display;
			    Vx_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Vx_Xita = atan2(Vx_Y_single, Vx_X_single) * RAD2DEG;

                /*High freq Vx AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 2.43647933643590e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 1.11703223671299e-07 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 4.56202313081729e-13 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 8.10510839227128e-19 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 7.01855387750807e-25 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 3.19577011724785e-31 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 7.30626417383309e-38 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 6.56840377409380e-45 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_Vrms = Vx_Vrms * Fit_AD_RTerm;

                Fit_AD_Xita = 0.0;

                Fit_AD_Vx_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Vx_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }
            else
            {
                Vx_X_single = Cal_PSD_A_X_display;
                Vx_Y_single = Cal_PSD_A_Y_display;
                Vx_Xita = atan2(Vx_Y_single, Vx_X_single) * RAD2DEG;

                /*Low freq Vx AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 1.23473420630106e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 4.90680229353423e-05 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 1.23108714587466e-07 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 1.38167152750442e-10 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 9.92576135595846e-14 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 4.65976588412469e-17 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 1.38833432637770e-20 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 2.60285375030146e-24 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_RTerm_8 = 0.0 - 3.02707464374032e-28 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_RTerm_9 = 0.0 + 2.09555536311036e-32 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_RTerm_10 = 0.0 - 7.79432626585634e-37 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_RTerm_11 = 0.0 + 1.17302511634848e-41 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_RTerm = Fit_AD_RTerm_0 + Fit_AD_RTerm_1 + Fit_AD_RTerm_2 + Fit_AD_RTerm_3 + Fit_AD_RTerm_4 + Fit_AD_RTerm_5 + Fit_AD_RTerm_6 + Fit_AD_RTerm_7 + Fit_AD_RTerm_8 + Fit_AD_RTerm_9 + Fit_AD_RTerm_10 + Fit_AD_RTerm_11;
                Fit_AD_Vrms = Vx_Vrms * Fit_AD_RTerm;

                Fit_AD_Xita = 0.0;

                Fit_AD_Vx_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Vx_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }

            /*--------  record data for debug  --------*/
            if (Cal_cnt == 1000)
            {
                Cal_cnt = 0;
            }
            Cal_Vx_X_array[Cal_cnt] = Vx_X_single;
            Cal_Vx_Y_array[Cal_cnt] = Vx_Y_single;

			/*	Measure Vr		*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;//1(bias_on)_11(no meaning)_0000(Vx)_111(500R)11(none)
			IA_CTRL_Analog += 32;                        //IA_CTRL_Analog = e3f:1(bias_on)_11(no meaning)_0001(Vr)_111(500R)11(none)
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

			Cal_single_state ++;
			break;

		/*	Read the Vr		*/
		case 1:
			//the DFT result
			// Vr_X_single = DFT_A_X_display;
			// Vr_Y_single = -1.0 * DFT_A_Y_display;

			//the PSD result
            Vr_Vrms = Cal_PSD_A_Vrms;
            Vr_Vpp = Cal_PSD_A_Vpp;
            if(freq_mixed_flag)
            {
			    Vr_X_single = Cal_PSD_A_X_display;
			    Vr_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Vr_Xita = atan2(Vr_Y_single, Vr_X_single) * RAD2DEG;
                /*High freq Vr AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 2.43323655630827e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 2.21713115658559e-07 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 1.49401972954543e-12 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 4.78955490760829e-18 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 8.93568813792144e-24 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 1.07337857856307e-29 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 8.68938917120826e-36 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 4.79671116794601e-42 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_RTerm_8 = 0.0 - 1.77743765474277e-48 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_RTerm_9 = 0.0 + 4.21704694978406e-55 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_RTerm_10 = 0.0 - 5.77060276229245e-62 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_RTerm_11 = 0.0 + 3.45427350134705e-69 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_RTerm = Fit_AD_RTerm_0 + Fit_AD_RTerm_1 + Fit_AD_RTerm_2 + Fit_AD_RTerm_3 + Fit_AD_RTerm_4 + Fit_AD_RTerm_5 + Fit_AD_RTerm_6 + Fit_AD_RTerm_7 + Fit_AD_RTerm_8 + Fit_AD_RTerm_9 + Fit_AD_RTerm_10 + Fit_AD_RTerm_11;
                Fit_AD_Vrms = Vr_Vrms * Fit_AD_RTerm;

                Fit_AD_XitaTerm_0 = 0.0 + 6.04215535805588e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_XitaTerm_1 = 0.0 + 3.96662759397377e-06 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_XitaTerm_2 = 0.0 - 4.43485711007332e-11 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_XitaTerm_3 = 0.0 + 2.09905401732967e-16 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_XitaTerm_4 = 0.0 - 5.16535426485498e-22 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_XitaTerm_5 = 0.0 + 7.54942889417478e-28 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_XitaTerm_6 = 0.0 - 6.99641647683924e-34 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_XitaTerm_7 = 0.0 + 4.22298898318464e-40 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_XitaTerm_8 = 0.0 - 1.65575097707616e-46 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_XitaTerm_9 = 0.0 + 4.06572021331287e-53 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_XitaTerm_10 = 0.0 - 5.67806283712728e-60 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_XitaTerm_11 = 0.0 + 3.44035589280027e-67 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_XitaTerm = Fit_AD_XitaTerm_0 + Fit_AD_XitaTerm_1 + Fit_AD_XitaTerm_2 + Fit_AD_XitaTerm_3 + Fit_AD_XitaTerm_4 + Fit_AD_XitaTerm_5 + Fit_AD_XitaTerm_6 + Fit_AD_XitaTerm_7 + Fit_AD_XitaTerm_8 + Fit_AD_XitaTerm_9 + Fit_AD_XitaTerm_10 + Fit_AD_XitaTerm_11;
                Fit_AD_Xita = Vr_Xita + Fit_AD_XitaTerm;

                Fit_AD_Vr_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Vr_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }
            else
            {
                Vr_X_single = Cal_PSD_A_X_display;
                Vr_Y_single = Cal_PSD_A_Y_display;
                Vr_Xita = atan2(Vr_Y_single, Vr_X_single) * RAD2DEG;
                /*Low freq Vr AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 1.24206168859584e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 1.12369176304305e-05 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 5.68173394306487e-08 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 7.55846134977826e-11 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 6.58029849229964e-14 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 3.61933787935613e-17 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 1.20369107210225e-20 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 2.44252102664515e-24 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_RTerm_8 = 0.0 - 3.02617113601629e-28 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_RTerm_9 = 0.0 + 2.21989510836107e-32 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_RTerm_10 = 0.0 - 8.78144032664688e-37 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_RTerm_11 = 0.0 + 1.42687172009080e-41 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_RTerm = Fit_AD_RTerm_0 + Fit_AD_RTerm_1 + Fit_AD_RTerm_2 + Fit_AD_RTerm_3 + Fit_AD_RTerm_4 + Fit_AD_RTerm_5 + Fit_AD_RTerm_6 + Fit_AD_RTerm_7 + Fit_AD_RTerm_8 + Fit_AD_RTerm_9 + Fit_AD_RTerm_10 + Fit_AD_RTerm_11;
                Fit_AD_Vrms = Vr_Vrms * Fit_AD_RTerm;

                Fit_AD_XitaTerm_0 = 0.0 - 1.57353175550840e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_XitaTerm_1 = 0.0 + 5.28369522457618e-02 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_XitaTerm_2 = 0.0 - 1.37027996410389e-05 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_XitaTerm_3 = 0.0 + 1.49504187770912e-08 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_XitaTerm_4 = 0.0 - 9.25451407278674e-12 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_XitaTerm_5 = 0.0 + 3.74503086139706e-15 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_XitaTerm_6 = 0.0 - 1.03699825007750e-18 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_XitaTerm_7 = 0.0 + 1.97001606494140e-22 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_XitaTerm_8 = 0.0 - 2.49948414328851e-26 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_XitaTerm_9 = 0.0 + 2.00582831618654e-30 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_XitaTerm_10 = 0.0 - 9.14740468347308e-35 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_XitaTerm_11 = 0.0 + 1.79847521333521e-39 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_XitaTerm = Fit_AD_XitaTerm_0 + Fit_AD_XitaTerm_1 + Fit_AD_XitaTerm_2 + Fit_AD_XitaTerm_3 + Fit_AD_XitaTerm_4 + Fit_AD_XitaTerm_5 + Fit_AD_XitaTerm_6 + Fit_AD_XitaTerm_7 + Fit_AD_XitaTerm_8 + Fit_AD_XitaTerm_9 + Fit_AD_XitaTerm_10 + Fit_AD_XitaTerm_11;
                Fit_AD_Xita = Vr_Xita + Fit_AD_XitaTerm;

                Fit_AD_Vr_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Vr_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }

            /*--------  record data for debug  --------*/
            Cal_Vr_X_array[Cal_cnt] = Vr_X_single;
            Cal_Vr_Y_array[Cal_cnt] = Vr_Y_single;

			/*	Measure Lp		*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 64;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);    //IA_CTRL_Analog = e5f:1(bias_on)_11(no meaning)_0010(Lp)_111(500R)11(none)

			Cal_single_state ++;
			break;

		/*	Read the Lp		*/
		case 2:
			//the DFT result
			// Lp_X_single = DFT_A_X_display;
			// Lp_Y_single = -1.0 * DFT_A_Y_display;
			//the PSD result
            Lp_Vrms = Cal_PSD_A_Vrms;
            Lp_Vpp = Cal_PSD_A_Vpp;
            if(freq_mixed_flag)
            {
			    Lp_X_single = -1.0 * Cal_PSD_A_X_display;
			    Lp_Y_single = Cal_PSD_A_Y_display;
                Lp_Xita = atan2(Lp_Y_single, Lp_X_single) * RAD2DEG;
                /*High freq Lp AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 2.43626275589583e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 2.19768900097873e-07 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 1.49453262039676e-12 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 4.80991768820170e-18 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 9.02973215235531e-24 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 1.09345728153218e-29 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 8.92459534968032e-36 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 4.95937632300159e-42 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_RTerm_8 = 0.0 - 1.84552042923278e-48 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_RTerm_9 = 0.0 + 4.38609709735096e-55 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_RTerm_10 = 0.0 - 5.99897884722949e-62 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_RTerm_11 = 0.0 + 3.58296454378132e-69 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_RTerm = Fit_AD_RTerm_0 + Fit_AD_RTerm_1 + Fit_AD_RTerm_2 + Fit_AD_RTerm_3 + Fit_AD_RTerm_4 + Fit_AD_RTerm_5 + Fit_AD_RTerm_6 + Fit_AD_RTerm_7 + Fit_AD_RTerm_8 + Fit_AD_RTerm_9 + Fit_AD_RTerm_10 + Fit_AD_RTerm_11;
                Fit_AD_Vrms = Lp_Vrms * Fit_AD_RTerm;

                Fit_AD_XitaTerm_0 = 0.0 + 6.05136582269766e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_XitaTerm_1 = 0.0 + 4.22035429186147e-06 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_XitaTerm_2 = 0.0 - 4.23794419261548e-11 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_XitaTerm_3 = 0.0 + 1.99358957786280e-16 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_XitaTerm_4 = 0.0 - 4.87080816529660e-22 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_XitaTerm_5 = 0.0 + 7.06877202803322e-28 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_XitaTerm_6 = 0.0 - 6.51102071827208e-34 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_XitaTerm_7 = 0.0 + 3.91083111685784e-40 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_XitaTerm_8 = 0.0 - 1.52774391215021e-46 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_XitaTerm_9 = 0.0 + 3.74176421627125e-53 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_XitaTerm_10 = 0.0 - 5.21706038187571e-60 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_XitaTerm_11 = 0.0 + 3.15822686415824e-67 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_XitaTerm = Fit_AD_XitaTerm_0 + Fit_AD_XitaTerm_1 + Fit_AD_XitaTerm_2 + Fit_AD_XitaTerm_3 + Fit_AD_XitaTerm_4 + Fit_AD_XitaTerm_5 + Fit_AD_XitaTerm_6 + Fit_AD_XitaTerm_7 + Fit_AD_XitaTerm_8 + Fit_AD_XitaTerm_9 + Fit_AD_XitaTerm_10 + Fit_AD_XitaTerm_11;
                Fit_AD_Xita = Lp_Xita + Fit_AD_XitaTerm;

                Fit_AD_Lp_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Lp_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }
            else
            {
                Lp_X_single = -1.0 * Cal_PSD_A_X_display;
                Lp_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Lp_Xita = atan2(Lp_Y_single, Lp_X_single) * RAD2DEG;
                /*Low freq Lp AD fitting*/
                Fit_AD_RTerm_0 = 0.0 + 1.24322804882483e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_RTerm_1 = 0.0 + 8.03983277572535e-06 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_RTerm_2 = 0.0 - 4.93237072664331e-08 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_RTerm_3 = 0.0 + 6.70592079641702e-11 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_RTerm_4 = 0.0 - 6.03065579530691e-14 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_RTerm_5 = 0.0 + 3.40188898881187e-17 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_RTerm_6 = 0.0 - 1.14838890152506e-20 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_RTerm_7 = 0.0 + 2.35033976960584e-24 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_RTerm_8 = 0.0 - 2.92610211372907e-28 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_RTerm_9 = 0.0 + 2.15189318571470e-32 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_RTerm_10 = 0.0 - 8.51964127014357e-37 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_RTerm_11 = 0.0 + 1.38347742173232e-41 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_RTerm = Fit_AD_RTerm_0 + Fit_AD_RTerm_1 + Fit_AD_RTerm_2 + Fit_AD_RTerm_3 + Fit_AD_RTerm_4 + Fit_AD_RTerm_5 + Fit_AD_RTerm_6 + Fit_AD_RTerm_7 + Fit_AD_RTerm_8 + Fit_AD_RTerm_9 + Fit_AD_RTerm_10 + Fit_AD_RTerm_11;
                Fit_AD_Vrms = Lp_Vrms * Fit_AD_RTerm;

                Fit_AD_XitaTerm_0 = 0.0 - 1.57366454395589e+00 * pow(Vx_Vr_freq, 0.0);
                Fit_AD_XitaTerm_1 = 0.0 + 5.28030539904527e-02 * pow(Vx_Vr_freq, 1.0);
                Fit_AD_XitaTerm_2 = 0.0 - 1.36055215207126e-05 * pow(Vx_Vr_freq, 2.0);
                Fit_AD_XitaTerm_3 = 0.0 + 1.48406837673459e-08 * pow(Vx_Vr_freq, 3.0);
                Fit_AD_XitaTerm_4 = 0.0 - 9.18953213983888e-12 * pow(Vx_Vr_freq, 4.0);
                Fit_AD_XitaTerm_5 = 0.0 + 3.72179972896447e-15 * pow(Vx_Vr_freq, 5.0);
                Fit_AD_XitaTerm_6 = 0.0 - 1.03167683601314e-18 * pow(Vx_Vr_freq, 6.0);
                Fit_AD_XitaTerm_7 = 0.0 + 1.96201234811539e-22 * pow(Vx_Vr_freq, 7.0);
                Fit_AD_XitaTerm_8 = 0.0 - 2.49157650647559e-26 * pow(Vx_Vr_freq, 8.0);
                Fit_AD_XitaTerm_9 = 0.0 + 2.00083389440913e-30 * pow(Vx_Vr_freq, 9.0);
                Fit_AD_XitaTerm_10 = 0.0 - 9.12886089013701e-35 * pow(Vx_Vr_freq, 10.0);
                Fit_AD_XitaTerm_11 = 0.0 + 1.79535681985688e-39 * pow(Vx_Vr_freq, 11.0);
                Fit_AD_XitaTerm = Fit_AD_XitaTerm_0 + Fit_AD_XitaTerm_1 + Fit_AD_XitaTerm_2 + Fit_AD_XitaTerm_3 + Fit_AD_XitaTerm_4 + Fit_AD_XitaTerm_5 + Fit_AD_XitaTerm_6 + Fit_AD_XitaTerm_7 + Fit_AD_XitaTerm_8 + Fit_AD_XitaTerm_9 + Fit_AD_XitaTerm_10 + Fit_AD_XitaTerm_11;
                Fit_AD_Xita = Lp_Xita + Fit_AD_XitaTerm;

                Fit_AD_Lp_X_single = Fit_AD_Vrms * cos(Fit_AD_Xita);
                Fit_AD_Lp_Y_single = Fit_AD_Vrms * sin(Fit_AD_Xita);
            }

            /*--------  record data for debug  --------*/
            Cal_Lp_X_array[Cal_cnt] = Lp_X_single;
            Cal_Lp_Y_array[Cal_cnt] = Lp_Y_single;
            Cal_cnt++;

		/*	Calculate the temp		*/
			A_single = Vx_X_single - Lp_X_single;//the numerator real part
			B_single = Vx_Y_single - Lp_Y_single;//the numerator imaginary part
			C_single = Lp_X_single - Vr_X_single;//the denominator real part
			//C_single = Lp_X_single - Vr_X_single - Lp_Y_single / Stray_Z;//the denominator real part
			D_single = Lp_Y_single - Vr_Y_single;//the denominator imaginary part
			//D_single = Lp_Y_single - Vr_Y_single + Lp_X_single / Stray_Z;//the denominator imaginary part


            //moving average ABCD_single
            if(AVER_ABCD_SINGLE_POINT >= 2)
            {
                Aver_A_Single(A_single);
                Aver_B_Single(B_single);
                Aver_C_Single(C_single);
                Aver_D_Single(D_single);
            }
            else
            {
                Average_A_single = A_single;
                Average_B_single = B_single;
                Average_C_single = C_single;
                Average_D_single = D_single;
                aver_A_Single_cnt = 1;
            }


		/*	Calculate the 		*/
            if(aver_A_Single_cnt >= AVER_ABCD_SINGLE_POINT)
            //aver_Single_cnt reach AVER_ABCD_SINGLE_POINT, so enter generateing one Dut_X,Dut_Y
            {
                complex_divide(Average_A_single, Average_B_single, Average_C_single, Average_D_single, &Coef_X, &Coef_Y);//(Vx-Lp)/(Lp-Vr) by jst


                if(Rr == 50000)
                {
                	//add by jst
                	Rr_X_0 = 0.0 + 5.40391233279853e+04 * pow(Vx_Vr_freq, 0.0);
                	Rr_X_1 = 0.0 - 1.18080318146497e-02 * pow(Vx_Vr_freq, 1.0);
                	Rr_X_2 = 0.0 + 1.16760748748844e-08 * pow(Vx_Vr_freq, 2.0);
                	Rr_X_3 = 0.0 - 6.16988013539424e-15 * pow(Vx_Vr_freq, 3.0);
                	Rr_X_4 = 0.0 + 1.89099807475222e-21 * pow(Vx_Vr_freq, 4.0);
                	Rr_X_5 = 0.0 - 3.72088143465139e-28 * pow(Vx_Vr_freq, 5.0);
                	Rr_X_6 = 0.0 + 4.93954532151942e-35 * pow(Vx_Vr_freq, 6.0);
                	Rr_X_7 = 0.0 - 4.54152284833005e-42 * pow(Vx_Vr_freq, 7.0);
                	Rr_X_8 = 0.0 + 2.92255798530419e-49 * pow(Vx_Vr_freq, 8.0);
                	Rr_X_9 = 0.0 - 1.31111720217680e-56 * pow(Vx_Vr_freq, 9.0);
                	Rr_X_10 = 0.0 + 4.01334847437829e-64 * pow(Vx_Vr_freq, 10.0);
                	Rr_X_11 = 0.0 - 7.98359681280282e-72 * pow(Vx_Vr_freq, 11.0);
                	Rr_X_12 = 0.0 + 9.29572318769715e-80 * pow(Vx_Vr_freq, 12.0);
                	Rr_X_13 = 0.0 - 4.80458839841276e-88 * pow(Vx_Vr_freq, 13.0);
                	Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11 + Rr_X_12 + Rr_X_13;
                	Rr_Y_0 = 0.0 + 9.99086568837324e+02 * pow(Vx_Vr_freq, 0.0);
                	Rr_Y_1 = 0.0 - 9.07548508194030e-03 * pow(Vx_Vr_freq, 1.0);
                	Rr_Y_2 = 0.0 + 5.95873563581415e-09 * pow(Vx_Vr_freq, 2.0);
                	Rr_Y_3 = 0.0 - 3.04330585766502e-15 * pow(Vx_Vr_freq, 3.0);
                	Rr_Y_4 = 0.0 + 9.21486744159348e-22 * pow(Vx_Vr_freq, 4.0);
                	Rr_Y_5 = 0.0 - 1.77973285117191e-28 * pow(Vx_Vr_freq, 5.0);
                	Rr_Y_6 = 0.0 + 2.31428272542319e-35 * pow(Vx_Vr_freq, 6.0);
                	Rr_Y_7 = 0.0 - 2.08551581465191e-42 * pow(Vx_Vr_freq, 7.0);
                	Rr_Y_8 = 0.0 + 1.31799785358823e-49 * pow(Vx_Vr_freq, 8.0);
                	Rr_Y_9 = 0.0 - 5.82344926647013e-57 * pow(Vx_Vr_freq, 9.0);
                	Rr_Y_10 = 0.0 + 1.76176672926646e-64 * pow(Vx_Vr_freq, 10.0);
                	Rr_Y_11 = 0.0 - 3.47708223950328e-72 * pow(Vx_Vr_freq, 11.0);
                	Rr_Y_12 = 0.0 + 4.03273226736800e-80 * pow(Vx_Vr_freq, 12.0);
                	Rr_Y_13 = 0.0 - 2.08430770103775e-88 * pow(Vx_Vr_freq, 13.0);
                	Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13;


                    //for WK measure 50kohm fitting use 4 significant digits
                    // Rr_X_11 = 0.0 - 4.01011515517084e-70 * pow(Vx_Vr_freq, 11.0);
                    // Rr_X_10 = 0.0 + 3.04295578066315e-62 * pow(Vx_Vr_freq, 10.0);
                    // Rr_X_9 =  0.0 - 9.73747391659035e-55 * pow(Vx_Vr_freq, 9.0);
                    // Rr_X_8 =  0.0 + 1.73998697317456e-47 * pow(Vx_Vr_freq, 8.0);
                    // Rr_X_7 =  0.0 - 1.92095229223685e-40 * pow(Vx_Vr_freq, 7.0);
                    // Rr_X_6 =  0.0 + 1.3591800642501e-33 * pow(Vx_Vr_freq, 6.0);
                    // Rr_X_5 =  0.0 - 6.13736201251982e-27 * pow(Vx_Vr_freq, 5.0);
                    // Rr_X_4 =  0.0 + 1.68293866946969e-20 * pow(Vx_Vr_freq, 4.0);
                    // Rr_X_3 =  0.0 - 2.44961404090715e-14 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 + 1.3491611641856e-08 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 - 0.0116044846936079 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 51360.609925704;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11;
//
                    // Rr_Y_14 = 0.0 - 3.07850288431434e-90 * pow(Vx_Vr_freq, 14.0);
                    // Rr_Y_13 = 0.0 + 7.66795595275457e-83 * pow(Vx_Vr_freq, 13.0);
                    // Rr_Y_12 = 0.0 + 3.07179255490052e-75 * pow(Vx_Vr_freq, 12.0);
                    // Rr_Y_11 = 0.0 - 1.86155095129469e-67 * pow(Vx_Vr_freq, 11.0);
                    // Rr_Y_10 = 0.0 + 4.29417164134688e-60 * pow(Vx_Vr_freq, 10.0);
                    // Rr_Y_9 =  0.0 - 5.83339669095528e-53 * pow(Vx_Vr_freq, 9.0);
                    // Rr_Y_8 =  0.0 + 5.20538300270394e-46 * pow(Vx_Vr_freq, 8.0);
                    // Rr_Y_7 =  0.0 - 3.17257260932501e-39 * pow(Vx_Vr_freq, 7.0);
                    // Rr_Y_6 =  0.0 + 1.32939718958762e-32 * pow(Vx_Vr_freq, 6.0);
                    // Rr_Y_5 =  0.0 - 3.76157763640417e-26 * pow(Vx_Vr_freq, 5.0);
                    // Rr_Y_4 =  0.0 + 6.87613445678858e-20 * pow(Vx_Vr_freq, 4.0);
                    // Rr_Y_3 =  0.0 - 7.59524278884676e-14 * pow(Vx_Vr_freq, 3.0);
                    // Rr_Y_2 =  0.0 + 5.22923824388732e-08 * pow(Vx_Vr_freq, 2.0);
                    // Rr_Y_1 =  0.0 - 0.0374635219681058 * pow(Vx_Vr_freq, 1.0);
                    // Rr_Y_0 =  0.0 + 745.683514927947;
                    // Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13 + Rr_Y_14;

                	/*guoquan
                	Rr_X_0 = 0.0 + 5.05876732740738e+04 * pow(Vx_Vr_freq, 0.0);
                	Rr_X_1 = 0.0 - 5.65452096824499e-03 * pow(Vx_Vr_freq, 1.0);
                	Rr_X_2 = 0.0 - 1.07812261245119e-08 * pow(Vx_Vr_freq, 2.0);
                	Rr_X_3 = 0.0 + 2.04949534480421e-14 * pow(Vx_Vr_freq, 3.0);
                	Rr_X_4 = 0.0 - 2.83850719040582e-20 * pow(Vx_Vr_freq, 4.0);
                	Rr_X_5 = 0.0 + 2.16227351407436e-26 * pow(Vx_Vr_freq, 5.0);
                	Rr_X_6 = 0.0 - 9.78549884489318e-33 * pow(Vx_Vr_freq, 6.0);
                	Rr_X_7 = 0.0 + 2.84645268572087e-39 * pow(Vx_Vr_freq, 7.0);
                	Rr_X_8 = 0.0 - 5.55228883588027e-46 * pow(Vx_Vr_freq, 8.0);
                	Rr_X_9 = 0.0 + 7.36556430020332e-53 * pow(Vx_Vr_freq, 9.0);
                	Rr_X_10 = 0.0 - 6.57251176309870e-60 * pow(Vx_Vr_freq, 10.0);
                	Rr_X_11 = 0.0 + 3.78108846833455e-67 * pow(Vx_Vr_freq, 11.0);
                	Rr_X_12 = 0.0 - 1.26797276499566e-74 * pow(Vx_Vr_freq, 12.0);
                	Rr_X_13 = 0.0 + 1.88435200959434e-82 * pow(Vx_Vr_freq, 13.0);
                	Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11 + Rr_X_12 + Rr_X_13;

                	Rr_Y_0 = 0.0 + 6.15993213543245e+02 * pow(Vx_Vr_freq, 0.0);
                	Rr_Y_1 = 0.0 - 3.64234370152810e-02 * pow(Vx_Vr_freq, 1.0);
                	Rr_Y_2 = 0.0 + 5.03207489019866e-08 * pow(Vx_Vr_freq, 2.0);
                	Rr_Y_3 = 0.0 - 7.20766993525629e-14 * pow(Vx_Vr_freq, 3.0);
                	Rr_Y_4 = 0.0 + 6.44361914160270e-20 * pow(Vx_Vr_freq, 4.0);
                	Rr_Y_5 = 0.0 - 3.48884153810528e-26 * pow(Vx_Vr_freq, 5.0);
                	Rr_Y_6 = 0.0 + 1.22559145919244e-32 * pow(Vx_Vr_freq, 6.0);
                	Rr_Y_7 = 0.0 - 2.92863978975274e-39 * pow(Vx_Vr_freq, 7.0);
                	Rr_Y_8 = 0.0 + 4.87065371034363e-46 * pow(Vx_Vr_freq, 8.0);
                	Rr_Y_9 = 0.0 - 5.65274702699887e-53 * pow(Vx_Vr_freq, 9.0);
                	Rr_Y_10 = 0.0 + 4.49613252396221e-60 * pow(Vx_Vr_freq, 10.0);
                	Rr_Y_11 = 0.0 - 2.33822134855425e-67 * pow(Vx_Vr_freq, 11.0);
                	Rr_Y_12 = 0.0 + 7.16578175710274e-75 * pow(Vx_Vr_freq, 12.0);
                	Rr_Y_13 = 0.0 - 9.81596547749268e-83 * pow(Vx_Vr_freq, 13.0);
                	Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13;*/
                }
                else if(Rr == 5000)
                {
                	//add by jst
                	Rr_X_0 = 0.0 + 5.42866739290766e+03 * pow(Vx_Vr_freq, 0.0);
                	Rr_X_1 = 0.0 - 1.21408685208735e-03 * pow(Vx_Vr_freq, 1.0);
                	Rr_X_2 = 0.0 + 1.19784916646457e-09 * pow(Vx_Vr_freq, 2.0);
                	Rr_X_3 = 0.0 - 5.92142139846014e-16 * pow(Vx_Vr_freq, 3.0);
                	Rr_X_4 = 0.0 + 1.71051248336449e-22 * pow(Vx_Vr_freq, 4.0);
                	Rr_X_5 = 0.0 - 3.14937815191988e-29 * pow(Vx_Vr_freq, 5.0);
                	Rr_X_6 = 0.0 + 3.88678991688222e-36 * pow(Vx_Vr_freq, 6.0);
                	Rr_X_7 = 0.0 - 3.30837089918410e-43 * pow(Vx_Vr_freq, 7.0);
                	Rr_X_8 = 0.0 + 1.96614475806864e-50 * pow(Vx_Vr_freq, 8.0);
                	Rr_X_9 = 0.0 - 8.13504222562074e-58 * pow(Vx_Vr_freq, 9.0);
                	Rr_X_10 = 0.0 + 2.29528496033887e-65 * pow(Vx_Vr_freq, 10.0);
                	Rr_X_11 = 0.0 - 4.20808816902756e-73 * pow(Vx_Vr_freq, 11.0);
                	Rr_X_12 = 0.0 + 4.51635777611769e-81 * pow(Vx_Vr_freq, 12.0);
                	Rr_X_13 = 0.0 - 2.15230130975267e-89 * pow(Vx_Vr_freq, 13.0);
                	Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11 + Rr_X_12 + Rr_X_13;
                	Rr_Y_0 = 0.0 + 1.23765620755486e+02 * pow(Vx_Vr_freq, 0.0);
                	Rr_Y_1 = 0.0 - 7.11112253181200e-04 * pow(Vx_Vr_freq, 1.0);
                	Rr_Y_2 = 0.0 + 8.19275637509442e-10 * pow(Vx_Vr_freq, 2.0);
                	Rr_Y_3 = 0.0 - 4.65570090033371e-16 * pow(Vx_Vr_freq, 3.0);
                	Rr_Y_4 = 0.0 + 1.52085616378700e-22 * pow(Vx_Vr_freq, 4.0);
                	Rr_Y_5 = 0.0 - 3.13260463374404e-29 * pow(Vx_Vr_freq, 5.0);
                	Rr_Y_6 = 0.0 + 4.28553083458993e-36 * pow(Vx_Vr_freq, 6.0);
                	Rr_Y_7 = 0.0 - 4.01150501678610e-43 * pow(Vx_Vr_freq, 7.0);
                	Rr_Y_8 = 0.0 + 2.60452080572387e-50 * pow(Vx_Vr_freq, 8.0);
                	Rr_Y_9 = 0.0 - 1.17117365356092e-57 * pow(Vx_Vr_freq, 9.0);
                	Rr_Y_10 = 0.0 + 3.57692328571771e-65 * pow(Vx_Vr_freq, 10.0);
                	Rr_Y_11 = 0.0 - 7.07758983631192e-73 * pow(Vx_Vr_freq, 11.0);
                	Rr_Y_12 = 0.0 + 8.18102028515711e-81 * pow(Vx_Vr_freq, 12.0);
                	Rr_Y_13 = 0.0 - 4.19312301622269e-89 * pow(Vx_Vr_freq, 13.0);
                	Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13;
                    //for WK measure 5kohm fitting use 4 significant digits
                    // Rr_X_11 = 0.0 + 5.19307028424598e-72 * pow(Vx_Vr_freq, 11.0);
                    // Rr_X_10 = 0.0 - 3.34507290129267e-64 * pow(Vx_Vr_freq, 10.0);
                    // Rr_X_9 =  0.0 + 9.40762681576109e-57 * pow(Vx_Vr_freq, 9.0);
                    // Rr_X_8 =  0.0 - 1.51377813698844e-49 * pow(Vx_Vr_freq, 8.0);
                    // Rr_X_7 =  0.0 + 1.53220022267439e-42 * pow(Vx_Vr_freq, 7.0);
                    // Rr_X_6 =  0.0 - 1.00694992420794e-35 * pow(Vx_Vr_freq, 6.0);
                    // Rr_X_5 =  0.0 + 4.25150740318013e-29 * pow(Vx_Vr_freq, 5.0);
                    // Rr_X_4 =  0.0 - 1.08554788067959e-22 * pow(Vx_Vr_freq, 4.0);
                    // Rr_X_3 =  0.0 + 1.47167517211393e-16 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 - 1.19061041130606e-10 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 + 0.00011120184715125 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 4987.40131797573;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11;
//
                    // Rr_Y_14 = 0.0 - 1.02217019054408e-91 * pow(Vx_Vr_freq, 14.0);
                    // Rr_Y_13 = 0.0 + 8.21834231013603e-84 * pow(Vx_Vr_freq, 13.0);
                    // Rr_Y_12 = 0.0 - 2.96981991226112e-76 * pow(Vx_Vr_freq, 12.0);
                    // Rr_Y_11 = 0.0 + 6.3713962431043e-69 * pow(Vx_Vr_freq, 11.0);
                    // Rr_Y_10 = 0.0 - 9.02753344159673e-62 * pow(Vx_Vr_freq, 10.0);
                    // Rr_Y_9 =  0.0 + 8.88731422941494e-55 * pow(Vx_Vr_freq, 9.0);
                    // Rr_Y_8 =  0.0 - 6.22285459718514e-48 * pow(Vx_Vr_freq, 8.0);
                    // Rr_Y_7 =  0.0 + 3.11516078273101e-41 * pow(Vx_Vr_freq, 7.0);
                    // Rr_Y_6 =  0.0 - 1.1024044056908e-34 * pow(Vx_Vr_freq, 6.0);
                    // Rr_Y_5 =  0.0 + 2.67310613082666e-28 * pow(Vx_Vr_freq, 5.0);
                    // Rr_Y_4 =  0.0 - 4.19055365812331e-22 * pow(Vx_Vr_freq, 4.0);
                    // Rr_Y_3 =  0.0 + 3.96283221956793e-16 * pow(Vx_Vr_freq, 3.0);
                    // Rr_Y_2 =  0.0 - 2.45205284566873e-10 * pow(Vx_Vr_freq, 2.0);
                    // Rr_Y_1 =  0.0 - 8.6302020736449e-05 * pow(Vx_Vr_freq, 1.0);
                    // Rr_Y_0 =  0.0 + 19.4760071479758;
                    // Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13 + Rr_Y_14;




                	//guoquan
                 /*   Rr_X_12 = 0.0 - 2.89721528404051e-78 * pow(Vx_Vr_freq, 12.0);
                    Rr_X_11 = 0.0 + 1.82305400844116e-70 * pow(Vx_Vr_freq, 11.0);
                    Rr_X_10 = 0.0 - 5.08748262266269e-63 * pow(Vx_Vr_freq, 10.0);
                    Rr_X_9 =  0.0 + 8.30106385750579e-56 * pow(Vx_Vr_freq, 9.0);
                    Rr_X_8 =  0.0 - 8.78685453511258e-49 * pow(Vx_Vr_freq, 8.0);
                    Rr_X_7 =  0.0 + 6.32175914606449e-42 * pow(Vx_Vr_freq, 7.0);
                    Rr_X_6 =  0.0 - 3.13996597136824e-35 * pow(Vx_Vr_freq, 6.0);
                    Rr_X_5 =  0.0 + 1.06481951090832e-28 * pow(Vx_Vr_freq, 5.0);
                    Rr_X_4 =  0.0 - 2.35366051953163e-22 * pow(Vx_Vr_freq, 4.0);
                    Rr_X_3 =  0.0 + 3.09195979650694e-16 * pow(Vx_Vr_freq, 3.0);
                    Rr_X_2 =  0.0 - 2.51703905121726e-10 * pow(Vx_Vr_freq, 2.0);
                    Rr_X_1 =  0.0 + 0.000183363090605981 * pow(Vx_Vr_freq, 1.0);
                    Rr_X_0 =  0.0 + 5031.18567627718;
                    Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11 + Rr_X_12;

                    Rr_Y_14 = 0.0 - 1.67020961867396e-90 * pow(Vx_Vr_freq, 14.0);
                    Rr_Y_13 = 0.0 + 1.22347447304776e-82 * pow(Vx_Vr_freq, 13.0);
                    Rr_Y_12 = 0.0 - 4.02831097543652e-75 * pow(Vx_Vr_freq, 12.0);
                    Rr_Y_11 = 0.0 + 7.87461415642625e-68 * pow(Vx_Vr_freq, 11.0);
                    Rr_Y_10 = 0.0 - 1.01693625606021e-60 * pow(Vx_Vr_freq, 10.0);
                    Rr_Y_9 =  0.0 + 9.13399219470807e-54 * pow(Vx_Vr_freq, 9.0);
                    Rr_Y_8 =  0.0 - 5.85012565831212e-47 * pow(Vx_Vr_freq, 8.0);
                    Rr_Y_7 =  0.0 + 2.69453973442522e-40 * pow(Vx_Vr_freq, 7.0);
                    Rr_Y_6 =  0.0 - 8.88431092380012e-34 * pow(Vx_Vr_freq, 6.0);
                    Rr_Y_5 =  0.0 + 2.06008017282101e-27 * pow(Vx_Vr_freq, 5.0);
                    Rr_Y_4 =  0.0 - 3.24902404486276e-21 * pow(Vx_Vr_freq, 4.0);
                    Rr_Y_3 =  0.0 + 3.31257753324908e-15 * pow(Vx_Vr_freq, 3.0);
                    Rr_Y_2 =  0.0 - 2.05503577713174e-09 * pow(Vx_Vr_freq, 2.0);
                    Rr_Y_1 =  0.0 + 0.000489164180821706 * pow(Vx_Vr_freq, 1.0);
                    Rr_Y_0 =  0.0 - 37.8138003992439;
                    Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13 + Rr_Y_14; */
                }
                else if(Rr == 500)
                {

                	Rr_X_0 = 0.0 + 4.99881096239113e+02 * pow(Vx_Vr_freq, 0.0);
                	Rr_X_1 = 0.0 + 1.84541031877369e-06 * pow(Vx_Vr_freq, 1.0);
                	Rr_X_2 = 0.0 - 2.98704169575119e-12 * pow(Vx_Vr_freq, 2.0);
                	Rr_X_3 = 0.0 + 1.96806622473387e-18 * pow(Vx_Vr_freq, 3.0);
                	Rr_X_4 = 0.0 - 7.38937950673987e-25 * pow(Vx_Vr_freq, 4.0);
                	Rr_X_5 = 0.0 + 1.68951692609562e-31 * pow(Vx_Vr_freq, 5.0);
                	Rr_X_6 = 0.0 - 2.50751802827245e-38 * pow(Vx_Vr_freq, 6.0);
                	Rr_X_7 = 0.0 + 2.50959835128747e-45 * pow(Vx_Vr_freq, 7.0);
                	Rr_X_8 = 0.0 - 1.72548355408334e-52 * pow(Vx_Vr_freq, 8.0);
                	Rr_X_9 = 0.0 + 8.16161694218969e-60 * pow(Vx_Vr_freq, 9.0);
                	Rr_X_10 = 0.0 - 2.60893935026950e-67 * pow(Vx_Vr_freq, 10.0);
                	Rr_X_11 = 0.0 + 5.38164383857742e-75 * pow(Vx_Vr_freq, 11.0);
                	Rr_X_12 = 0.0 - 6.46351293298069e-83 * pow(Vx_Vr_freq, 12.0);
                	Rr_X_13 = 0.0 + 3.43218773413426e-91 * pow(Vx_Vr_freq, 13.0);
                	Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7 + Rr_X_8 + Rr_X_9 + Rr_X_10 + Rr_X_11 + Rr_X_12 + Rr_X_13;
                    //blue line 7 order fitting for 3ms PSD
                    // Rr_X_7 =  0.0 + 3.92136980019609e-47 * pow(Vx_Vr_freq, 7.0);
                    // Rr_X_6 =  0.0 - 1.36797095117579e-39 * pow(Vx_Vr_freq, 6.0);
                    // Rr_X_5 =  0.0 + 1.87617580630971e-32 * pow(Vx_Vr_freq, 5.0);
                    // Rr_X_4 =  0.0 - 1.27514044052634e-25 * pow(Vx_Vr_freq, 4.0);
                    // Rr_X_3 =  0.0 + 4.49551409157501e-19 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 - 8.86863404096659e-13 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 + 1.82817752088484e-06 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 502.780050175903;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6 + Rr_X_7;

                    // blue line 3 order fitting for 3ms PSD
                    // Rr_X_3 =  0.0 + 6.69979905384031e-21 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 - 1.65965500244407e-13 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 + 1.4111426121373e-06 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 502.809404964866;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3;

                    // origin line 6 order fitting for 3ms PSD
                    //Rr_X_6 =  0.0 + 1.24375457727081e-40 * pow(Vx_Vr_freq, 6.0);
                    //Rr_X_5 =  0.0 - 4.14994936912423e-33 * pow(Vx_Vr_freq, 5.0);
                    //Rr_X_4 =  0.0 + 5.39065991545212e-26 * pow(Vx_Vr_freq, 4.0);
                    //Rr_X_3 =  0.0 - 3.37004174330232e-19 * pow(Vx_Vr_freq, 3.0);
                    //Rr_X_2 =  0.0 + 9.49723128015711e-13 * pow(Vx_Vr_freq, 2.0);
                    //Rr_X_1 =  0.0 - 3.39437768613886e-07 * pow(Vx_Vr_freq, 1.0);
                    //Rr_X_0 =  0.0 + 503.774122969756;
                    //Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6;





/*                    //dont consider Lp stray C by guoquan
                    Rr_X_0 = 0.0 + 5.00455926419583e+02 * pow(Vx_Vr_freq, 0.0);
                    Rr_X_1 = 0.0 - 4.85536160358400e-07 * pow(Vx_Vr_freq, 1.0);
                    Rr_X_2 = 0.0 + 1.12663292349709e-13 * pow(Vx_Vr_freq, 2.0);
                    Rr_X_3 = 0.0 - 1.71061338916269e-20 * pow(Vx_Vr_freq, 3.0);
                    Rr_X_4 = 0.0 + 1.26643240524944e-27 * pow(Vx_Vr_freq, 4.0);
                    Rr_X_5 = 0.0 - 3.73737867608929e-35 * pow(Vx_Vr_freq, 5.0);
                    Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5;*/







                    //behind 8.4MHz,(origin line+bule line)/2 5 order fitting for 3ms PSD
                    // Rr_X_5 =  0.0 - 2.60369842707287e-34 * pow(Vx_Vr_freq, 5.0);
                    // Rr_X_4 =  0.0 + 7.15949424298601e-27 * pow(Vx_Vr_freq, 4.0);
                    // Rr_X_3 =  0.0 - 6.57489104833157e-20 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 + 1.70719392190134e-13 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 + 6.64530993346783e-07 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 503.352523559382;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5;

                    //Rr_Y_11 = 0.0 + 3.12685363579513e-73 * pow(Vx_Vr_freq, 11.0);
                    //Rr_Y_10 = 0.0 - 1.78706318002853e-65 * pow(Vx_Vr_freq, 10.0);
                    //Rr_Y_9 =  0.0 + 4.41531801003942e-58 * pow(Vx_Vr_freq, 9.0);
                    //Rr_Y_8 =  0.0 - 6.16528615310305e-51 * pow(Vx_Vr_freq, 8.0);
                    //Rr_Y_7 =  0.0 + 5.33521984361464e-44 * pow(Vx_Vr_freq, 7.0);
                    //Rr_Y_6 =  0.0 - 2.94816533496273e-37 * pow(Vx_Vr_freq, 6.0);
                    //Rr_Y_5 =  0.0 + 1.03127116285931e-30 * pow(Vx_Vr_freq, 5.0);
                    //Rr_Y_4 =  0.0 - 2.17660435084998e-24 * pow(Vx_Vr_freq, 4.0);
                    //Rr_Y_3 =  0.0 + 2.49947777543383e-18 * pow(Vx_Vr_freq, 3.0);
                    //Rr_Y_2 =  0.0 - 1.39146146389593e-12 * pow(Vx_Vr_freq, 2.0);
                    //Rr_Y_1 =  0.0 + 3.06085729503647e-06 * pow(Vx_Vr_freq, 1.0);
                    //Rr_Y_0 =  0.0 + 2.61508398107633;
                    //Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11;





                	Rr_Y_0 = 0.0 + 1.04173750169943e+01 * pow(Vx_Vr_freq, 0.0);
                	Rr_Y_1 = 0.0 - 5.65114394179119e-05 * pow(Vx_Vr_freq, 1.0);
                	Rr_Y_2 = 0.0 + 6.70994923295424e-11 * pow(Vx_Vr_freq, 2.0);
                	Rr_Y_3 = 0.0 - 3.68199058871216e-17 * pow(Vx_Vr_freq, 3.0);
                	Rr_Y_4 = 0.0 + 1.16669992022986e-23 * pow(Vx_Vr_freq, 4.0);
                	Rr_Y_5 = 0.0 - 2.34094726605905e-30 * pow(Vx_Vr_freq, 5.0);
                	Rr_Y_6 = 0.0 + 3.12947012018155e-37 * pow(Vx_Vr_freq, 6.0);
                	Rr_Y_7 = 0.0 - 2.86903932621199e-44 * pow(Vx_Vr_freq, 7.0);
                	Rr_Y_8 = 0.0 + 1.82754076749706e-51 * pow(Vx_Vr_freq, 8.0);
                	Rr_Y_9 = 0.0 - 8.07403939880938e-59 * pow(Vx_Vr_freq, 9.0);
                	Rr_Y_10 = 0.0 + 2.42587349487579e-66 * pow(Vx_Vr_freq, 10.0);
                	Rr_Y_11 = 0.0 - 4.72786486910754e-74 * pow(Vx_Vr_freq, 11.0);
                	Rr_Y_12 = 0.0 + 5.38928701862931e-82 * pow(Vx_Vr_freq, 12.0);
                	Rr_Y_13 = 0.0 - 2.72726132459052e-90 * pow(Vx_Vr_freq, 13.0);
                	Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6 + Rr_Y_7 + Rr_Y_8 + Rr_Y_9 + Rr_Y_10 + Rr_Y_11 + Rr_Y_12 + Rr_Y_13;








/*                    //guoquan
                    Rr_Y_0 = 0.0 - 1.09167527593605e-01 * pow(Vx_Vr_freq, 0.0);
                    Rr_Y_1 = 0.0 + 1.95312684771997e-06 * pow(Vx_Vr_freq, 1.0);
                    Rr_Y_2 = 0.0 + 2.11839042254208e-13 * pow(Vx_Vr_freq, 2.0);
                    Rr_Y_3 = 0.0 - 4.19652992977504e-20 * pow(Vx_Vr_freq, 3.0);
                    Rr_Y_4 = 0.0 + 3.82440015342276e-27 * pow(Vx_Vr_freq, 4.0);
                    Rr_Y_5 = 0.0 - 1.31765015687256e-34 * pow(Vx_Vr_freq, 5.0);
                    Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5;*/












                    /*Rr_X and Rr_Y for 10ms PSD, It is wrong! */
                    // Rr_X_6 =  0.0 + 1.1896247117485e-43 * pow(Vx_Vr_freq, 6.0);
                    // Rr_X_5 =  0.0 - 2.63901666429014e-34 * pow(Vx_Vr_freq, 5.0);
                    // Rr_X_4 =  0.0 + 7.19878370387263e-27 * pow(Vx_Vr_freq, 4.0);
                    // Rr_X_3 =  0.0 - 6.59515445018966e-20 * pow(Vx_Vr_freq, 3.0);
                    // Rr_X_2 =  0.0 + 1.71212028783363e-13 * pow(Vx_Vr_freq, 2.0);
                    // Rr_X_1 =  0.0 + 6.63970858913792e-07 * pow(Vx_Vr_freq, 1.0);
                    // Rr_X_0 =  0.0 + 503.352891129732;
                    // Rr_X = Rr_X_0 + Rr_X_1 + Rr_X_2 + Rr_X_3 + Rr_X_4 + Rr_X_5 + Rr_X_6;
//
                    // Rr_Y_6 =  0.0 + 2.37601423105338e-41 * pow(Vx_Vr_freq, 6.0);
                    // Rr_Y_5 =  0.0 - 5.69177036464261e-34 * pow(Vx_Vr_freq, 5.0);
                    // Rr_Y_4 =  0.0 + 3.0558752132268e-27 * pow(Vx_Vr_freq, 4.0);
                    // Rr_Y_3 =  0.0 + 2.43352943779296e-20 * pow(Vx_Vr_freq, 3.0);
                    // Rr_Y_2 =  0.0 - 3.05468316609412e-13 * pow(Vx_Vr_freq, 2.0);
                    // Rr_Y_1 =  0.0 + 3.17735175886331e-06 * pow(Vx_Vr_freq, 1.0);
                    // Rr_Y_0 =  0.0 + 2.52571985651337;
                    // Rr_Y = Rr_Y_0 + Rr_Y_1 + Rr_Y_2 + Rr_Y_3 + Rr_Y_4 + Rr_Y_5 + Rr_Y_6;
                }
                else
                {
                }

                complex_product(Coef_X, Coef_Y, Rr_X, Rr_Y, &DUT_X, &DUT_Y);//(Vx-Lp)/(Lp-Vr)*Rr=DUT      by jst

                //moving aver DUT_X, DUT_Y
                if(Measure_Mode != REPEAT_MEASURE_MODE)
                {
                    if(SINGLE_CAL_SWEEP_MODE_AVER_NUM > 1)
                    {
                        Moving_Aver_X(DUT_X);
                        Moving_Aver_Y(DUT_Y);
                    }
                    else
                    {
                        moving_aver_result_X = DUT_X;
                        moving_aver_result_Y = DUT_Y;
                        moving_aver_compl_cnt_X++;//It indicates that output is valid.
                        moving_aver_compl_cnt_Y++;
                    }
                }
                else if(Measure_Mode == REPEAT_MEASURE_MODE)
                {
                    // Repeat_balance_Aver_X(DUT_X);
                    // Repeat_balance_Aver_Y(DUT_Y);
                    // Repeat_balance_no_moving_Aver_X(DUT_X);
                    // Repeat_balance_no_moving_Aver_Y(DUT_Y);
                    moving_aver_result_X = DUT_X;
                    moving_aver_result_Y = DUT_Y;
                    moving_aver_compl_cnt_X++;
                    moving_aver_compl_cnt_Y++;
                }

    			DUT_display_1 = ParameterAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_1, Rr);
    			DUT_display_2 = ParameterAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_2, Rr);
                Dut_array[Cal_cnt] = DUT_display_1;
    			DUT_display_fitting_1 = ParameterFittingAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_1, Rr);
    			DUT_display_fitting_2 = ParameterFittingAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_2, Rr);
                Dut_Z_module = pow(moving_aver_result_X, 2) + pow(moving_aver_result_Y, 2);
            }

            /*--------  end of cal  --------*/

            if (moving_aver_compl_cnt_X >= 1 || moving_aver_compl_cnt_Y >= 1) {
                switch (Measure_Mode) {
                    case SINGLE_MEASURE_MODE:
                        Workspace = NULL_WORKSPACE;
                        Measure_Mode = NONE_MEASURE_MODE;
                        Master_switch_flag = 0;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case REPEAT_MEASURE_MODE:
                        Workspace = CAL_DUT;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case FREQ_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        freq_sweep_pointer_temp++;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case AMP_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        amp_sweep_pointer_temp++;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    default:

                        break;
                }
            }



            /*--------  measure Vx, for next cal_dut  --------*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 0;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

			Cal_single_state = 0;

			break;

		default:
			Cal_single_state = 0;
			break;
	}
    Cal_dft_delay_flag = 0;
}


/*Cal Rr function*/
void Cal_Rr_single(void)
{
    long double DUT_Y_0, DUT_Y_1, DUT_Y_2, DUT_Y_3, DUT_Y_4, DUT_Y_5, DUT_Y_6, DUT_Y_7, DUT_Y_8, DUT_Y_9, DUT_Y_10, DUT_Y_11, DUT_Y_12, DUT_Y_13, DUT_Y_14;
    long double DUT_X_0, DUT_X_1, DUT_X_2, DUT_X_3, DUT_X_4, DUT_X_5, DUT_X_6, DUT_X_7, DUT_X_8, DUT_X_9, DUT_X_10, DUT_X_11, DUT_X_12, DUT_X_13;
	switch(Cal_Rr_single_state)
	{
		/*	Read the Vx		*/
		case 0:
			//the DFT result
			// Vx_X_single = DFT_A_X_display;
			// Vx_Y_single = -1.0 * DFT_A_Y_display;

			//the PSD result
            Vx_Vrms = Cal_PSD_A_Vrms;
            if(freq_mixed_flag)
            {
                Vx_X_single = Cal_PSD_A_X_display;
                Vx_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Vx_Xita = atan2(Vx_Y_single, Vx_X_single) * RAD2DEG;
            }
            else
            {
                Vx_X_single = Cal_PSD_A_X_display;
                Vx_Y_single = Cal_PSD_A_Y_display;
                Vx_Xita = atan2(Vx_Y_single, Vx_X_single) * RAD2DEG;
            }
            Vx_Vpp = Cal_PSD_A_Vpp;

            /*--------  record data for debug  --------*/
            if (Cal_cnt == 1000)
            {
                Cal_cnt = 0;
            }
            Cal_Vx_X_array[Cal_cnt] = Vx_X_single;
            Cal_Vx_Y_array[Cal_cnt] = Vx_Y_single;

			/*	Measure Vr		*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 32;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
			usleep(10);
			Cal_Rr_single_state ++;
			break;

		/*	Read the Vr		*/
		case 1:
			//the DFT result
			// Vr_X_single = DFT_A_X_display;
			// Vr_Y_single = -1.0 * DFT_A_Y_display;

			//the PSD result
            Vr_Vrms = Cal_PSD_A_Vrms;
            if(freq_mixed_flag)
            {
			    Vr_X_single = Cal_PSD_A_X_display;
			    Vr_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Vr_Xita = atan2(Vr_Y_single, Vr_X_single) * RAD2DEG;
            }
            else
            {
                Vr_X_single = Cal_PSD_A_X_display;
                Vr_Y_single = Cal_PSD_A_Y_display;
                Vr_Xita = atan2(Vr_Y_single, Vr_X_single) * RAD2DEG;
            }
            Vr_Vpp = Cal_PSD_A_Vpp;

            /*--------  record data for debug  --------*/
            Cal_Vr_X_array[Cal_cnt] = Vr_X_single;
            Cal_Vr_Y_array[Cal_cnt] = Vr_Y_single;

			/*	Measure Lp		*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 64;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

			Cal_Rr_single_state ++;
			break;

		/*	Read the Lp		*/
		case 2:
			//the DFT result
			// Lp_X_single = DFT_A_X_display;
			// Lp_Y_single = -1.0 * DFT_A_Y_display;
			//the DFT result
            Lp_Vrms = Cal_PSD_A_Vrms;
            if(freq_mixed_flag)
            {
			    Lp_X_single = -1.0 * Cal_PSD_A_X_display;
			    Lp_Y_single = Cal_PSD_A_Y_display;
                Lp_Xita = atan2(Lp_Y_single, Lp_X_single) * RAD2DEG;
            }
            else
            {
                Lp_X_single = -1.0 * Cal_PSD_A_X_display;
                Lp_Y_single = -1.0 * Cal_PSD_A_Y_display;
                Lp_Xita = atan2(Lp_Y_single, Lp_X_single) * RAD2DEG;
            }
            Lp_Vpp = Cal_PSD_A_Vpp;

            /*--------  record data for debug  --------*/
            Cal_Lp_X_array[Cal_cnt] = Lp_X_single;
            Cal_Lp_Y_array[Cal_cnt] = Lp_Y_single;
            Cal_cnt++;

		/*	Calculate the temp		*/
			A_single = Vx_X_single - Lp_X_single;//the numerator real part
			B_single = Vx_Y_single - Lp_Y_single;//the numerator imaginary part
			C_single = Lp_X_single - Vr_X_single;//the denominator real part
			//C_single = Lp_X_single - Vr_X_single - Lp_Y_single / Stray_Z;//the denominator real part
			D_single = Lp_Y_single - Vr_Y_single;//the denominator imaginary part
			//D_single = Lp_Y_single - Vr_Y_single + Lp_X_single / Stray_Z;//the denominator imaginary part


            //moving average ABCD_single
            if(AVER_ABCD_SINGLE_POINT >= 2)
            {
                Aver_A_Single(A_single);
                Aver_B_Single(B_single);
                Aver_C_Single(C_single);
                Aver_D_Single(D_single);
            }
            else
            {
                Average_A_single = A_single;
                Average_B_single = B_single;
                Average_C_single = C_single;
                Average_D_single = D_single;
                aver_A_Single_cnt = 1;
            }


		/*	Calculate the 		*/
            if(aver_A_Single_cnt >= AVER_ABCD_SINGLE_POINT)
            {
                complex_divide(Average_A_single, Average_B_single, Average_C_single, Average_D_single, &Coef_X, &Coef_Y);

                if(Rr == 50000)
                {
                	//add by jst
                	DUT_X_0 = 0.0 + 5.39909941740532e+04 * pow(Vx_Vr_freq, 0.0);
                	DUT_X_1 = 0.0 - 1.11044714209002e-02 * pow(Vx_Vr_freq, 1.0);
                	DUT_X_2 = 0.0 + 1.04792550891259e-08 * pow(Vx_Vr_freq, 2.0);
                	DUT_X_3 = 0.0 - 5.13294694379057e-15 * pow(Vx_Vr_freq, 3.0);
                	DUT_X_4 = 0.0 + 1.44640073583244e-21 * pow(Vx_Vr_freq, 4.0);
                	DUT_X_5 = 0.0 - 2.59794118410843e-28 * pow(Vx_Vr_freq, 5.0);
                	DUT_X_6 = 0.0 + 3.13564696840804e-35 * pow(Vx_Vr_freq, 6.0);
                	DUT_X_7 = 0.0 - 2.61854188027623e-42 * pow(Vx_Vr_freq, 7.0);
                	DUT_X_8 = 0.0 + 1.53233607536911e-49 * pow(Vx_Vr_freq, 8.0);
                	DUT_X_9 = 0.0 - 6.26813174332982e-57 * pow(Vx_Vr_freq, 9.0);
                	DUT_X_10 = 0.0 + 1.75595372317657e-64 * pow(Vx_Vr_freq, 10.0);
                	DUT_X_11 = 0.0 - 3.21056671187633e-72 * pow(Vx_Vr_freq, 11.0);
                	DUT_X_12 = 0.0 + 3.45185325647584e-80 * pow(Vx_Vr_freq, 12.0);
                	DUT_X_13 = 0.0 - 1.65529508388223e-88 * pow(Vx_Vr_freq, 13.0);
                	DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6 + DUT_X_7 + DUT_X_8 + DUT_X_9 + DUT_X_10 + DUT_X_11 + DUT_X_12 + DUT_X_13;
                	DUT_Y_0 = 0.0 + 9.76475393733475e+00 * pow(Vx_Vr_freq, 0.0);
                	DUT_Y_1 = 0.0 - 3.37786898238349e-03 * pow(Vx_Vr_freq, 1.0);
                	DUT_Y_2 = 0.0 + 5.13217631604819e-10 * pow(Vx_Vr_freq, 2.0);
                	DUT_Y_3 = 0.0 - 3.18903288206759e-16 * pow(Vx_Vr_freq, 3.0);
                	DUT_Y_4 = 0.0 + 1.19805979930115e-22 * pow(Vx_Vr_freq, 4.0);
                	DUT_Y_5 = 0.0 - 2.67985023073049e-29 * pow(Vx_Vr_freq, 5.0);
                	DUT_Y_6 = 0.0 + 3.89507538142339e-36 * pow(Vx_Vr_freq, 6.0);
                	DUT_Y_7 = 0.0 - 3.82220300511270e-43 * pow(Vx_Vr_freq, 7.0);
                	DUT_Y_8 = 0.0 + 2.57282470952288e-50 * pow(Vx_Vr_freq, 8.0);
                	DUT_Y_9 = 0.0 - 1.18787430476439e-57 * pow(Vx_Vr_freq, 9.0);
                	DUT_Y_10 = 0.0 + 3.69409150668979e-65 * pow(Vx_Vr_freq, 10.0);
                	DUT_Y_11 = 0.0 - 7.39079823210019e-73 * pow(Vx_Vr_freq, 11.0);
                	DUT_Y_12 = 0.0 + 8.58837816877762e-81 * pow(Vx_Vr_freq, 12.0);
                	DUT_Y_13 = 0.0 - 4.40447366475996e-89 * pow(Vx_Vr_freq, 13.0);
                	DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13;

                /*
                	//guoquan
                    //50k Dut fitting
                	DUT_X_0 = 0.0 + 5.00265667586367e+04 * pow(Vx_Vr_freq, 0.0);
                	DUT_X_1 = 0.0 - 4.04456807184635e-04 * pow(Vx_Vr_freq, 1.0);
                	DUT_X_2 = 0.0 + 2.79099132476274e-10 * pow(Vx_Vr_freq, 2.0);
                	DUT_X_3 = 0.0 - 3.00514441731571e-16 * pow(Vx_Vr_freq, 3.0);
                	DUT_X_4 = 0.0 - 3.19701102659181e-22 * pow(Vx_Vr_freq, 4.0);
                	DUT_X_5 = 0.0 + 5.20256610202493e-28 * pow(Vx_Vr_freq, 5.0);
                	DUT_X_6 = 0.0 - 3.25540214432014e-34 * pow(Vx_Vr_freq, 6.0);
                	DUT_X_7 = 0.0 + 1.18774186185755e-40 * pow(Vx_Vr_freq, 7.0);
                	DUT_X_8 = 0.0 - 2.78755985097511e-47 * pow(Vx_Vr_freq, 8.0);
                	DUT_X_9 = 0.0 + 4.33813602272569e-54 * pow(Vx_Vr_freq, 9.0);
                	DUT_X_10 = 0.0 - 4.45708857352760e-61 * pow(Vx_Vr_freq, 10.0);
                	DUT_X_11 = 0.0 + 2.90783725833290e-68 * pow(Vx_Vr_freq, 11.0);
                	DUT_X_12 = 0.0 - 1.09174874908450e-75 * pow(Vx_Vr_freq, 12.0);
                	DUT_X_13 = 0.0 + 1.79650303928047e-83 * pow(Vx_Vr_freq, 13.0);
                	DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6 + DUT_X_7 + DUT_X_8 + DUT_X_9 + DUT_X_10 + DUT_X_11 + DUT_X_12 + DUT_X_13;

                	DUT_Y_0 = 0.0 + 6.91379821581598e+01 * pow(Vx_Vr_freq, 0.0);
                	DUT_Y_1 = 0.0 - 5.30818947095639e-03 * pow(Vx_Vr_freq, 1.0);
                	DUT_Y_2 = 0.0 + 4.61619561481445e-09 * pow(Vx_Vr_freq, 2.0);
                	DUT_Y_3 = 0.0 - 8.72547967125020e-15 * pow(Vx_Vr_freq, 3.0);
                	DUT_Y_4 = 0.0 + 9.36775149002918e-21 * pow(Vx_Vr_freq, 4.0);
                	DUT_Y_5 = 0.0 - 6.13151968465802e-27 * pow(Vx_Vr_freq, 5.0);
                	DUT_Y_6 = 0.0 + 2.59698736960693e-33 * pow(Vx_Vr_freq, 6.0);
                	DUT_Y_7 = 0.0 - 7.37543207925585e-40 * pow(Vx_Vr_freq, 7.0);
                	DUT_Y_8 = 0.0 + 1.43048475131722e-46 * pow(Vx_Vr_freq, 8.0);
                	DUT_Y_9 = 0.0 - 1.89876200355556e-53 * pow(Vx_Vr_freq, 9.0);
                	DUT_Y_10 = 0.0 + 1.69596290705045e-60 * pow(Vx_Vr_freq, 10.0);
                	DUT_Y_11 = 0.0 - 9.74195433578301e-68 * pow(Vx_Vr_freq, 11.0);
                	DUT_Y_12 = 0.0 + 3.24956577107926e-75 * pow(Vx_Vr_freq, 12.0);
                	DUT_Y_13 = 0.0 - 4.78255810176246e-83 * pow(Vx_Vr_freq, 13.0);
                	DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13;
*/
                    //100k Dut fitting
                    //DUT_X_11 = 0.0 + 3.72980069853547e-69 * pow(Vx_Vr_freq, 11.0);
                    //DUT_X_10 = 0.0 - 2.26698874842195e-61 * pow(Vx_Vr_freq, 10.0);
                    //DUT_X_9 =  0.0 + 6.05400573888566e-54 * pow(Vx_Vr_freq, 9.0);
                    //DUT_X_8 =  0.0 - 9.33650539890037e-47 * pow(Vx_Vr_freq, 8.0);
                    //DUT_X_7 =  0.0 + 9.18934174201664e-40 * pow(Vx_Vr_freq, 7.0);
                    //DUT_X_6 =  0.0 - 6.0168058668055e-33 * pow(Vx_Vr_freq, 6.0);
                    //DUT_X_5 =  0.0 + 2.6451223011178e-26 * pow(Vx_Vr_freq, 5.0);
                    //DUT_X_4 =  0.0 - 7.65640490486556e-20 * pow(Vx_Vr_freq, 4.0);
                    //DUT_X_3 =  0.0 + 1.36633258903851e-13 * pow(Vx_Vr_freq, 3.0);
                    //DUT_X_2 =  0.0 - 1.21565316738982e-07 * pow(Vx_Vr_freq, 2.0);
                    //DUT_X_1 =  0.0 - 0.00819338893767954 * pow(Vx_Vr_freq, 1.0);
                    //DUT_X_0 =  0.0 + 98769.1978363877;
                    //DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6 + DUT_X_7 + DUT_X_8 + DUT_X_9 + DUT_X_10 + DUT_X_11;

                    //DUT_Y_13 = 0.0 + 3.31721385827992e-82 * pow(Vx_Vr_freq, 13.0);
                    //DUT_Y_12 = 0.0 - 2.30457890042724e-74 * pow(Vx_Vr_freq, 12.0);
                    //DUT_Y_11 = 0.0 + 7.14467845112203e-67 * pow(Vx_Vr_freq, 11.0);
                    //DUT_Y_10 = 0.0 - 1.30314206418319e-59 * pow(Vx_Vr_freq, 10.0);
                    //DUT_Y_9 =  0.0 + 1.55169996137494e-52 * pow(Vx_Vr_freq, 9.0);
                    //DUT_Y_8 =  0.0 - 1.26467916982319e-45 * pow(Vx_Vr_freq, 8.0);
                    //DUT_Y_7 =  0.0 + 7.18559301692646e-39 * pow(Vx_Vr_freq, 7.0);
                    //DUT_Y_6 =  0.0 - 2.83710333078322e-32 * pow(Vx_Vr_freq, 6.0);
                    //DUT_Y_5 =  0.0 + 7.57068685935685e-26 * pow(Vx_Vr_freq, 5.0);
                    //DUT_Y_4 =  0.0 - 1.267336189497e-19 * pow(Vx_Vr_freq, 4.0);
                    //DUT_Y_3 =  0.0 + 1.04857370723009e-13 * pow(Vx_Vr_freq, 3.0);
                    //DUT_Y_2 =  0.0 + 1.73927073415128e-08 * pow(Vx_Vr_freq, 2.0);
                    //DUT_Y_1 =  0.0 - 0.0934656336088117 * pow(Vx_Vr_freq, 1.0);
                    //DUT_Y_0 =  0.0 - 2202.15876231753;
                    //DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13;
                }
                else if(Rr == 5000)
                {

                	//add by jst
                	DUT_X_0 = 0.0 + 5.43109967190636e+03 * pow(Vx_Vr_freq, 0.0);
                	DUT_X_1 = 0.0 - 1.21142116270429e-03 * pow(Vx_Vr_freq, 1.0);
                	DUT_X_2 = 0.0 + 1.19711580729088e-09 * pow(Vx_Vr_freq, 2.0);
                	DUT_X_3 = 0.0 - 5.91898288471185e-16 * pow(Vx_Vr_freq, 3.0);
                	DUT_X_4 = 0.0 + 1.71325724057895e-22 * pow(Vx_Vr_freq, 4.0);
                	DUT_X_5 = 0.0 - 3.16641190630898e-29 * pow(Vx_Vr_freq, 5.0);
                	DUT_X_6 = 0.0 + 3.93185278818060e-36 * pow(Vx_Vr_freq, 6.0);
                	DUT_X_7 = 0.0 - 3.37657511068535e-43 * pow(Vx_Vr_freq, 7.0);
                	DUT_X_8 = 0.0 + 2.03065011431395e-50 * pow(Vx_Vr_freq, 8.0);
                	DUT_X_9 = 0.0 - 8.52900503803202e-58 * pow(Vx_Vr_freq, 9.0);
                	DUT_X_10 = 0.0 + 2.45065517174440e-65 * pow(Vx_Vr_freq, 10.0);
                	DUT_X_11 = 0.0 - 4.59018109881717e-73 * pow(Vx_Vr_freq, 11.0);
                	DUT_X_12 = 0.0 + 5.04911370380602e-81 * pow(Vx_Vr_freq, 12.0);
                	DUT_X_13 = 0.0 - 2.47386226930114e-89 * pow(Vx_Vr_freq, 13.0);
                	DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6 + DUT_X_7 + DUT_X_8 + DUT_X_9 + DUT_X_10 + DUT_X_11 + DUT_X_12 + DUT_X_13;
                	DUT_Y_0 = 0.0 + 3.97376740882951e+00 * pow(Vx_Vr_freq, 0.0);
                	DUT_Y_1 = 0.0 - 4.98047615210832e-05 * pow(Vx_Vr_freq, 1.0);
                	DUT_Y_2 = 0.0 + 1.41676393570666e-11 * pow(Vx_Vr_freq, 2.0);
                	DUT_Y_3 = 0.0 - 6.85808571820587e-18 * pow(Vx_Vr_freq, 3.0);
                	DUT_Y_4 = 0.0 + 1.90336043526631e-24 * pow(Vx_Vr_freq, 4.0);
                	DUT_Y_5 = 0.0 - 3.29507215210125e-31 * pow(Vx_Vr_freq, 5.0);
                	DUT_Y_6 = 0.0 + 3.74598264847156e-38 * pow(Vx_Vr_freq, 6.0);
                	DUT_Y_7 = 0.0 - 2.87871048908494e-45 * pow(Vx_Vr_freq, 7.0);
                	DUT_Y_8 = 0.0 + 1.51222997046991e-52 * pow(Vx_Vr_freq, 8.0);
                	DUT_Y_9 = 0.0 - 5.39382160985777e-60 * pow(Vx_Vr_freq, 9.0);
                	DUT_Y_10 = 0.0 + 1.26832377840352e-67 * pow(Vx_Vr_freq, 10.0);
                	DUT_Y_11 = 0.0 - 1.84039156605724e-75 * pow(Vx_Vr_freq, 11.0);
                	DUT_Y_12 = 0.0 + 1.42787687057170e-83 * pow(Vx_Vr_freq, 12.0);
                	DUT_Y_13 = 0.0 - 4.03300247453363e-92 * pow(Vx_Vr_freq, 13.0);
                	DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13;
                   /* guoquan
                    DUT_X_6 =  0.0 + 3.45975352835315e-40 * pow(Vx_Vr_freq, 6.0);
                    DUT_X_5 =  0.0 - 1.2436050005166e-32 * pow(Vx_Vr_freq, 5.0);
                    DUT_X_4 =  0.0 + 1.63010809137229e-25 * pow(Vx_Vr_freq, 4.0);
                    DUT_X_3 =  0.0 - 9.69151428856779e-19 * pow(Vx_Vr_freq, 3.0);
                    DUT_X_2 =  0.0 + 2.31795939470718e-12 * pow(Vx_Vr_freq, 2.0);
                    DUT_X_1 =  0.0 - 3.30056599694528e-06 * pow(Vx_Vr_freq, 1.0);
                    DUT_X_0 =  0.0 + 4999.91276575165;
                    DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6;

                    DUT_Y_7 =  0.0 - 4.94222601556139e-47 * pow(Vx_Vr_freq, 7.0);
                    DUT_Y_6 =  0.0 + 1.03385471065022e-39 * pow(Vx_Vr_freq, 6.0);
                    DUT_Y_5 =  0.0 - 3.3967045560174e-33 * pow(Vx_Vr_freq, 5.0);
                    DUT_Y_4 =  0.0 - 5.83269752279114e-26 * pow(Vx_Vr_freq, 4.0);
                    DUT_Y_3 =  0.0 + 4.92702049068869e-19 * pow(Vx_Vr_freq, 3.0);
                    DUT_Y_2 =  0.0 - 1.10652484510215e-12 * pow(Vx_Vr_freq, 2.0);
                    DUT_Y_1 =  0.0 - 3.69436044175677e-05 * pow(Vx_Vr_freq, 1.0);
                    DUT_Y_0 =  0.0 - 0.839890688269933;
                    DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7;*/
                }
                else if(Rr == 500)
                {

                	DUT_X_0 = 0.0 + 5.00238590573887e+02 * pow(Vx_Vr_freq, 0.0);
                	DUT_X_1 = 0.0 + 1.77541555910214e-07 * pow(Vx_Vr_freq, 1.0);
                	DUT_X_2 = 0.0 - 1.64455422747989e-13 * pow(Vx_Vr_freq, 2.0);
                	DUT_X_3 = 0.0 + 8.67802357520034e-20 * pow(Vx_Vr_freq, 3.0);
                	DUT_X_4 = 0.0 - 2.81463099640612e-26 * pow(Vx_Vr_freq, 4.0);
                	DUT_X_5 = 0.0 + 5.87359087298763e-33 * pow(Vx_Vr_freq, 5.0);
                	DUT_X_6 = 0.0 - 8.15677280873252e-40 * pow(Vx_Vr_freq, 6.0);
                	DUT_X_7 = 0.0 + 7.71637936242214e-47 * pow(Vx_Vr_freq, 7.0);
                	DUT_X_8 = 0.0 - 5.03263449941234e-54 * pow(Vx_Vr_freq, 8.0);
                	DUT_X_9 = 0.0 + 2.25988698028035e-61 * pow(Vx_Vr_freq, 9.0);
                	DUT_X_10 = 0.0 - 6.85705119310618e-69 * pow(Vx_Vr_freq, 10.0);
                	DUT_X_11 = 0.0 + 1.34222326827625e-76 * pow(Vx_Vr_freq, 11.0);
                	DUT_X_12 = 0.0 - 1.52957702205408e-84 * pow(Vx_Vr_freq, 12.0);
                	DUT_X_13 = 0.0 + 7.70823309850887e-93 * pow(Vx_Vr_freq, 13.0);
                	DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6 + DUT_X_7 + DUT_X_8 + DUT_X_9 + DUT_X_10 + DUT_X_11 + DUT_X_12 + DUT_X_13;

                	DUT_Y_0 = 0.0 - 3.87244437233735e-03 * pow(Vx_Vr_freq, 0.0);
                	DUT_Y_1 = 0.0 + 1.08251552369315e-08 * pow(Vx_Vr_freq, 1.0);
                	DUT_Y_2 = 0.0 - 3.55196944433017e-13 * pow(Vx_Vr_freq, 2.0);
                	DUT_Y_3 = 0.0 + 2.82944467441633e-19 * pow(Vx_Vr_freq, 3.0);
                	DUT_Y_4 = 0.0 - 1.13813590559222e-25 * pow(Vx_Vr_freq, 4.0);
                	DUT_Y_5 = 0.0 + 2.69304166355963e-32 * pow(Vx_Vr_freq, 5.0);
                	DUT_Y_6 = 0.0 - 4.06074934564166e-39 * pow(Vx_Vr_freq, 6.0);
                	DUT_Y_7 = 0.0 + 4.07751402733719e-46 * pow(Vx_Vr_freq, 7.0);
                	DUT_Y_8 = 0.0 - 2.78615846380447e-53 * pow(Vx_Vr_freq, 8.0);
                	DUT_Y_9 = 0.0 + 1.30004475835449e-60 * pow(Vx_Vr_freq, 9.0);
                	DUT_Y_10 = 0.0 - 4.07600450263945e-68 * pow(Vx_Vr_freq, 10.0);
                	DUT_Y_11 = 0.0 + 8.21047046238307e-76 * pow(Vx_Vr_freq, 11.0);
                	DUT_Y_12 = 0.0 - 9.59814635346239e-84 * pow(Vx_Vr_freq, 12.0);
                	DUT_Y_13 = 0.0 + 4.94923926009671e-92 * pow(Vx_Vr_freq, 13.0);
                	DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13;

                	// DUT_X = 500.0;guoquan
/*                    DUT_X_6 =  0.0 + 2.48530364734885e-42 * pow(Vx_Vr_freq, 6.0);
                    DUT_X_5 =  0.0 - 7.17761726612902e-35 * pow(Vx_Vr_freq, 5.0);
                    DUT_X_4 =  0.0 + 6.60539819397118e-28 * pow(Vx_Vr_freq, 4.0);
                    DUT_X_3 =  0.0 - 1.32055046928212e-21 * pow(Vx_Vr_freq, 3.0);
                    DUT_X_2 =  0.0 - 1.1610557797994e-14 * pow(Vx_Vr_freq, 2.0);
                    DUT_X_1 =  0.0 + 6.8015102214461e-08 * pow(Vx_Vr_freq, 1.0);
                    DUT_X_0 =  0.0 + 499.836236067459;
                    DUT_X = DUT_X_0 + DUT_X_1 + DUT_X_2 + DUT_X_3 + DUT_X_4 + DUT_X_5 + DUT_X_6;*/

                /*    DUT_Y_14 = 0.0 + 2.13872170710239e-93 * pow(Vx_Vr_freq, 14.0);
                    DUT_Y_13 = 0.0 - 1.60817961734309e-85 * pow(Vx_Vr_freq, 13.0);
                    DUT_Y_12 = 0.0 + 5.41803923517545e-78 * pow(Vx_Vr_freq, 12.0);
                    DUT_Y_11 = 0.0 - 1.07859310010219e-70 * pow(Vx_Vr_freq, 11.0);
                    DUT_Y_10 = 0.0 + 1.40851467344859e-63 * pow(Vx_Vr_freq, 10.0);
                    DUT_Y_9 =  0.0 - 1.26609144184504e-56 * pow(Vx_Vr_freq, 9.0);
                    DUT_Y_8 =  0.0 + 7.99375057170511e-50 * pow(Vx_Vr_freq, 8.0);
                    DUT_Y_7 =  0.0 - 3.550783281422e-43 * pow(Vx_Vr_freq, 7.0);
                    DUT_Y_6 =  0.0 + 1.09337107592042e-36 * pow(Vx_Vr_freq, 6.0);
                    DUT_Y_5 =  0.0 - 2.25573572097038e-30 * pow(Vx_Vr_freq, 5.0);
                    DUT_Y_4 =  0.0 + 2.92231805421132e-24 * pow(Vx_Vr_freq, 4.0);
                    DUT_Y_3 =  0.0 - 2.07182406391706e-18 * pow(Vx_Vr_freq, 3.0);
                    DUT_Y_2 =  0.0 + 4.88896434833865e-13 * pow(Vx_Vr_freq, 2.0);
                    DUT_Y_1 =  0.0 + 2.00046700963735e-08 * pow(Vx_Vr_freq, 1.0);
                    DUT_Y_0 =  0.0 - 0.0814295229216255;
                    DUT_Y = DUT_Y_0 + DUT_Y_1 + DUT_Y_2 + DUT_Y_3 + DUT_Y_4 + DUT_Y_5 + DUT_Y_6 + DUT_Y_7 + DUT_Y_8 + DUT_Y_9 + DUT_Y_10 + DUT_Y_11 + DUT_Y_12 + DUT_Y_13 + DUT_Y_14;*/


                }
                else
                {
                }

                complex_divide(DUT_X, DUT_Y, Coef_X, Coef_Y, &Rr_X, &Rr_Y);

                if(Measure_Mode != REPEAT_MEASURE_MODE)
                {
                    if(SINGLE_CAL_SWEEP_MODE_AVER_NUM > 1)
                    {
                        Moving_Aver_X(Rr_X);
                        Moving_Aver_Y(Rr_Y);
                    }
                    else
                    {
                        moving_aver_result_X = Rr_X;
                        moving_aver_result_Y = Rr_Y;
                        moving_aver_compl_cnt_X++;
                        moving_aver_compl_cnt_Y++;
                    }
                }
                else if(Measure_Mode == REPEAT_MEASURE_MODE)
                {
                    // Repeat_balance_Aver_X(DUT_X);
                    // Repeat_balance_Aver_Y(DUT_Y);
                    // Repeat_balance_no_moving_Aver_X(DUT_X);
                    // Repeat_balance_no_moving_Aver_Y(DUT_Y);
                    moving_aver_result_X = Rr_X;
                    moving_aver_result_Y = Rr_Y;
                    moving_aver_compl_cnt_X++;
                    moving_aver_compl_cnt_Y++;
                }

    			DUT_display_1 = ParameterAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_1, Rr);
    			DUT_display_2 = ParameterAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_2, Rr);
    			DUT_display_fitting_1 = ParameterFittingAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_1, Rr);
    			DUT_display_fitting_2 = ParameterFittingAttributes(moving_aver_result_X, moving_aver_result_Y, Vx_Vr_freq, sequence_2, Rr);
                Dut_array[Cal_cnt] = DUT_display_fitting_1;
            }

            /*--------  end of cal  --------*/
            if (moving_aver_compl_cnt_X == 1 || moving_aver_compl_cnt_Y == 1) {
                switch (Measure_Mode) {
                    case SINGLE_MEASURE_MODE:
                        Workspace = NULL_WORKSPACE;
                        Measure_Mode = NONE_MEASURE_MODE;
                        Master_switch_flag = 0;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case REPEAT_MEASURE_MODE:
                        Workspace = CAL_Rr;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case FREQ_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        freq_sweep_pointer_temp++;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    case AMP_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        aver_A_Single_cnt = 0;
                        aver_B_Single_cnt = 0;
                        aver_C_Single_cnt = 0;
                        aver_D_Single_cnt = 0;
                        moving_aver_compl_cnt_X = 0;
                        moving_aver_compl_cnt_Y = 0;
                        amp_sweep_pointer_temp++;
                        Cal_moving_aver_initialize();
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out
                        Data_interact_ON_flag = 1;
                        break;
                    default:

                        break;
                }
            }


            /*--------  measure Vx, for next cal_Rr  --------*/
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
			IA_CTRL_Analog += 0;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

			Cal_Rr_single_state = 0;

			break;

		default:
			Cal_Rr_single_state = 0;
			break;
	}
    Cal_Rr_dft_delay_flag = 0;
}

/*moving average variable initialize when Cal Dut*/
void Cal_moving_aver_initialize(void)
{
	Cal_single_state = 0;
	moving_aver_cnt_X =  0;
	moving_aver_cnt_Y =  0;
}

/*moving average variable initialize when Cal Rr*/
void Cal_Rr_moving_aver_initialize(void)
{
	Cal_Rr_single_state = 0;
	moving_aver_cnt_X =  0;
	moving_aver_cnt_Y =  0;
}

/*Calculate the fitting required electrical parameters according to X and Y of Dut*/
double ParameterFittingAttributes(double X, double Y, Xfloat32 freq, u8 sequence, u16 Rr)
{
	double parameter = 0.0;
    double Fitting_X, Fitting_Y;
    double Fitting_X_0,Fitting_X_1,Fitting_X_2,Fitting_X_3,Fitting_X_4,Fitting_X_5,Fitting_X_6,Fitting_X_7,Fitting_X_8,Fitting_X_9,Fitting_X_10,Fitting_X_11,Fitting_X_12;
    double Open_fitting_R, Open_fitting_X;
    double Open_fitting_temp_X_1, Open_fitting_temp_Y_1;
    double Open_fitting_temp_X_2, Open_fitting_temp_Y_2;


    /*--------  Open fitting  --------*/
    // if (freq >= 200000.0)
    // {
    //     Open_fitting_R = 0.0 - 1.2909E-29 * pow(freq, 5.0) + 2.0473E-22 * pow(freq, 4.0) + 9.5505E-15 * pow(freq, 3.0) - 2.3678E-7 * pow(freq, 2.0) + 1.278 * pow(freq, 1.0) + 4.534E6;
    //     Open_fitting_X = 0.0 - 1.537E-40 * pow(freq, 7.0) + 9.4537E-33 * pow(freq, 6.0) - 2.2816E-25 * pow(freq, 5.0) + 2.7735E-18 * pow(freq, 4.0) - 1.8122E-11 * pow(freq, 3.0) + 6.258E-5 * pow(freq, 2.0) - 103.5321 * pow(freq, 1.0) + 6.6858E7;
    //     complex_product(X, Y, Open_fitting_R, Open_fitting_X, &Open_fitting_temp_X_1, &Open_fitting_temp_Y_1);
    //     complex_minus(Open_fitting_R, Open_fitting_X, X, Y, &Open_fitting_temp_X_2, &Open_fitting_temp_Y_2);
    //     complex_divide(Open_fitting_temp_X_1, Open_fitting_temp_Y_1, Open_fitting_temp_X_2, Open_fitting_temp_Y_2, &X, &Y);
    // }

    /*--------  5k Dut load fitting  --------*/
    //Fitting_X_11 = 0.0 - 1.97828783412539e-76 * pow(Vx_Vr_freq, 11.0);
    //Fitting_X_10 = 0.0 + 1.54764517311425e-68 * pow(Vx_Vr_freq, 10.0);
    //Fitting_X_9 =  0.0 - 5.03394283843427e-61 * pow(Vx_Vr_freq, 9.0);
    //Fitting_X_8 =  0.0 + 8.99148703373499e-54 * pow(Vx_Vr_freq, 8.0);
    //Fitting_X_7 =  0.0 - 9.70953509575626e-47 * pow(Vx_Vr_freq, 7.0);
    //Fitting_X_6 =  0.0 + 6.4965851691806e-40 * pow(Vx_Vr_freq, 6.0);
    //Fitting_X_5 =  0.0 - 2.58719199195609e-33 * pow(Vx_Vr_freq, 5.0);
    //Fitting_X_4 =  0.0 + 5.02712028318104e-27 * pow(Vx_Vr_freq, 4.0);
    //Fitting_X_3 =  0.0 + 3.4703642404163e-22 * pow(Vx_Vr_freq, 3.0);
    //Fitting_X_2 =  0.0 - 1.11119234836697e-14 * pow(Vx_Vr_freq, 2.0);
    //Fitting_X_1 =  0.0 + 1.00536971120626e-09 * pow(Vx_Vr_freq, 1.0);
    //Fitting_X_0 =  0.0 + 1.00140645909667;
    //Fitting_X = Fitting_X_0 + Fitting_X_1 + Fitting_X_2 + Fitting_X_3 + Fitting_X_4 + Fitting_X_5 + Fitting_X_6 + Fitting_X_7 + Fitting_X_8 + Fitting_X_9 + Fitting_X_10 + Fitting_X_11;


    /*--------  50k Dut load fitting  --------*/
    //Fitting_X_11 = 0.0 - 9.28652120409415e-74 * pow(Vx_Vr_freq, 11.0);
    //Fitting_X_10 = 0.0 + 5.08471998925707e-66 * pow(Vx_Vr_freq, 10.0);
    //Fitting_X_9 =  0.0 - 1.21106074854653e-58 * pow(Vx_Vr_freq, 9.0);
    //Fitting_X_8 =  0.0 + 1.64542428509824e-51 * pow(Vx_Vr_freq, 8.0);
    //Fitting_X_7 =  0.0 - 1.40478645393709e-44 * pow(Vx_Vr_freq, 7.0);
    //Fitting_X_6 =  0.0 + 7.81741234588287e-38 * pow(Vx_Vr_freq, 6.0);
    //Fitting_X_5 =  0.0 - 2.83841680401888e-31 * pow(Vx_Vr_freq, 5.0);
    //Fitting_X_4 =  0.0 + 6.49218596537637e-25 * pow(Vx_Vr_freq, 4.0);
    //Fitting_X_3 =  0.0 - 8.55556168151232e-19 * pow(Vx_Vr_freq, 3.0);
    //Fitting_X_2 =  0.0 + 6.14790506619532e-13 * pow(Vx_Vr_freq, 2.0);
    //Fitting_X_1 =  0.0 - 1.49613783423671e-07 * pow(Vx_Vr_freq, 1.0);
    //Fitting_X_0 =  0.0 + 1.02333828558968;
    //Fitting_X = Fitting_X_0 + Fitting_X_1 + Fitting_X_2 + Fitting_X_3 + Fitting_X_4 + Fitting_X_5 + Fitting_X_6 + Fitting_X_7 + Fitting_X_8 + Fitting_X_9 + Fitting_X_10 + Fitting_X_11;

    //X = X / Fitting_X;
    //Y = Y / Fitting_Y;


	switch(sequence)
	{
		/*	0		.Inductance	*/
		case 0:
//			parameter = Y * (double)Rr / (2.0 * 3.1415926f * freq);
			parameter = Y / (2.0 * 3.1415926f * freq);			//	L
			break;
		/*	1		.Capacitance	*/
		case 1:
			parameter = 0.0 - 1.0 /  (2.0 * 3.1415926f * freq * Y);			//	C
			break;
		/*	2		.Resistance	*/
		case 2:
//			parameter = 0.0 - X * (double)Rr;
			parameter = X;			//	R
			break;
		/*	3		.Impedance	*/
		case 3:
			parameter = sqrt(pow(X, 2) + pow(Y, 2));			//	Z
			break;
		/*	4		.Admittance	*/
		case 4:
			parameter = 1.0 / (sqrt(pow(X, 2) + pow(Y, 2)));				//	Y
			break;
		/*	5		.Reactance	*/
		case 5:
//			parameter = Y * (double)Rr;
			parameter = Y;				//	X
			break;
		/*	6		.Conductance	*/
		case 6:
			parameter = X / ((pow(X, 2) + pow(Y, 2)));				//	G
			break;
		/*	7		.Susceptance	*/
		case 7:
//			parameter = 0.0 - Y / ((pow(X, 2) + pow(Y, 2)) * (double)Rr);				//	B
			parameter = 0.0 - Y / ((pow(X, 2) + pow(Y, 2)));
			break;
		/*	8		.Q-factors	*/
		case 8:
//			parameter = X / Y;
			parameter = X / Y;				//	Q
			break;
		/*	9		.D-factors	*/
		case 9:
//			parameter = Y / X;				//	D
			parameter = Y / X;				//	D
			break;
		/*	10	.Angle	*/
		case 10:
//			parameter = atan2(Y, X) * RAD2DEG;				//	D
			parameter = atan2(Y, X) * RAD2DEG;				//	D
			break;
		default:
			parameter = sqrt(pow(X, 2) + pow(Y, 2));			//	Z
			break;
	}

	//parameter = parameter * (double)Rr;
	return(parameter);
}

/*Calculate the required electrical parameters according to X and Y of Dut*/
double ParameterAttributes(double X, double Y, Xfloat32 freq, u8 sequence, u16 Rr)
{
	double parameter = 0.0;

	switch(sequence)
	{
		/*	0		.Inductance	*/
		case 0:
//			parameter = Y * (double)Rr / (2.0 * 3.1415926f * freq);
			parameter = Y / (2.0 * 3.1415926f * freq);			//	L
			break;
		/*	1		.Capacitance	*/
		case 1:
			parameter = 0.0 - 1.0 /  (2.0 * 3.1415926f * freq * Y);			//	C
			break;
		/*	2		.Resistance	*/
		case 2:
//			parameter = 0.0 - X * (double)Rr;
			parameter = X;			//	R
			break;
		/*	3		.Impedance	*/
		case 3:
			parameter = sqrt(pow(X, 2) + pow(Y, 2));			//	Z
			break;
		/*	4		.Admittance	*/
		case 4:
			parameter = 1.0 / (sqrt(pow(X, 2) + pow(Y, 2)));				//	Y
			break;
		/*	5		.Reactance	*/
		case 5:
//			parameter = Y * (double)Rr;
			parameter = Y;				//	X
			break;
		/*	6		.Conductance	*/
		case 6:
			parameter = X / ((pow(X, 2) + pow(Y, 2)));				//	G
			break;
		/*	7		.Susceptance	*/
		case 7:
//			parameter = 0.0 - Y / ((pow(X, 2) + pow(Y, 2)) * (double)Rr);				//	B
			parameter = 0.0 - Y / ((pow(X, 2) + pow(Y, 2)));
			break;
		/*	8		.Q-factors	*/
		case 8:
//			parameter = X / Y;
			parameter = X / Y;				//	Q
			break;
		/*	9		.D-factors	*/
		case 9:
//			parameter = Y / X;				//	D
			parameter = Y / X;				//	D
			break;
		/*	10	.Angle	*/
		case 10:
//			parameter = atan2(Y, X) * RAD2DEG;				//	D
			parameter = atan2(Y, X) * RAD2DEG;				//	D
			break;
		default:
			parameter = sqrt(pow(X, 2) + pow(Y, 2));			//	Z
			break;
	}

	//parameter = parameter * (double)Rr;
	return(parameter);
}

/*average X*/
void Moving_Aver_X(double data)
{
	u8 i;
	if(moving_aver_cnt_X == 0)
	{
		moving_aver_temp_X[moving_aver_cnt_X] = data;
		moving_aver_sum_X = 0.0;
		moving_aver_result_X = 0.0;
		moving_aver_cnt_X++;
		moving_aver_sum_X += data;
		moving_aver_compl_cnt_X = 0;
	}
	else if(moving_aver_cnt_X != SINGLE_CAL_SWEEP_MODE_AVER_NUM - 1)
	{
		moving_aver_temp_X[moving_aver_cnt_X] = data;
		moving_aver_sum_X += data;
		moving_aver_cnt_X++;
	}
	else
	{
        moving_aver_temp_X[moving_aver_cnt_X] = data;
        moving_aver_sum_X = moving_aver_sum_X + data;
		moving_aver_result_X = moving_aver_sum_X / SINGLE_CAL_SWEEP_MODE_AVER_NUM;
		moving_aver_compl_cnt_X++;
	}
}

/*average Y*/
void Moving_Aver_Y(double data)
{
	u8 i;
	if(moving_aver_cnt_Y == 0)
	{
		moving_aver_temp_Y[moving_aver_cnt_Y] = data;
		moving_aver_sum_Y = 0.0;
		moving_aver_result_Y = 0.0;
		moving_aver_cnt_Y++;
		moving_aver_sum_Y += data;
		moving_aver_compl_cnt_Y = 0;
	}
	else if(moving_aver_cnt_Y != SINGLE_CAL_SWEEP_MODE_AVER_NUM - 1)
	{
		moving_aver_temp_Y[moving_aver_cnt_Y] = data;
		moving_aver_sum_Y += data;
		moving_aver_cnt_Y++;
	}
	else
	{
        moving_aver_temp_Y[moving_aver_cnt_Y] = data;
        moving_aver_sum_Y = moving_aver_sum_Y + data;
		moving_aver_result_Y = moving_aver_sum_Y / SINGLE_CAL_SWEEP_MODE_AVER_NUM;
		moving_aver_compl_cnt_Y++;
	}
}

/*moving average X*/
void Repeat_balance_Moving_Aver_X(double data)
{
	u8 i;
	if(moving_aver_cnt_X == 0)
	{
		moving_aver_temp_X[moving_aver_cnt_X] = data;
		moving_aver_sum_X = 0.0;
		moving_aver_result_X = 0.0;
		moving_aver_cnt_X++;
		moving_aver_sum_X += data;
		moving_aver_compl_cnt_X = 0;
	}
	else if(moving_aver_cnt_X < repeat_balance_points - 1)
	{
		moving_aver_temp_X[moving_aver_cnt_X] = data;
		moving_aver_sum_X += data;
		moving_aver_cnt_X++;
	}
    else if (moving_aver_cnt_X == repeat_balance_points - 1)
    {
		moving_aver_temp_X[moving_aver_cnt_X] = data;
		moving_aver_sum_X += data;
		moving_aver_cnt_X++;
        moving_aver_result_X = moving_aver_sum_X / repeat_balance_points;
        moving_aver_compl_cnt_X++;
    }
	else
	{
    	moving_aver_sum_X = moving_aver_sum_X - moving_aver_temp_X[0]  + data;
		moving_aver_result_X = moving_aver_sum_X / repeat_balance_points;
		for(i=0; i<(repeat_balance_points - 1);i++)
		{
			moving_aver_temp_X[i] = moving_aver_temp_X[i+1];
		}
		moving_aver_temp_X[repeat_balance_points - 1] = data;
		moving_aver_compl_cnt_X++;//moving_aver_compl_cnt_X = 1,is arithmetic average; while moving_aver_compl_cnt_X > 1, is moving average.
	}
}


/*average X*/
void Repeat_balance_Aver_X(double data)
{
	u8 i;
	if(repeat_balance_aver_cnt_X == 0)
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X = 0.0;
		repeat_aver_result_X = 0.0;
		repeat_balance_aver_cnt_X++;
		repeat_aver_sum_X += data;
	}
	else if(repeat_balance_aver_cnt_X != REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT - 1)
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X += data;
		repeat_balance_aver_cnt_X++;
	}
	else
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X += data;
		repeat_aver_result_X = repeat_aver_sum_X / REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT;
		repeat_aver_compl_cnt_X++;
        if (repeat_aver_compl_cnt_X == repeat_balance_points) {
            Workspace = DETERMINE_WORK_MODE;
            repeat_aver_compl_cnt_X = 0;
        }
        Repeat_balance_Moving_Aver_X(repeat_aver_result_X);
        repeat_balance_aver_cnt_X = 0;
	}
}


/*moving average Y*/
void Repeat_balance_Moving_Aver_Y(double data)
{
	u8 i;
	if(moving_aver_cnt_Y == 0)
	{
		moving_aver_temp_Y[moving_aver_cnt_Y] = data;
		moving_aver_sum_Y = 0.0;
		moving_aver_result_Y = 0.0;
		moving_aver_cnt_Y++;
		moving_aver_sum_Y += data;
		moving_aver_compl_cnt_Y = 0;
	}
	else if(moving_aver_cnt_Y < repeat_balance_points - 1)
	{
		moving_aver_temp_Y[moving_aver_cnt_Y] = data;
		moving_aver_sum_Y += data;
		moving_aver_cnt_Y++;
	}
    else if (moving_aver_cnt_Y == repeat_balance_points - 1)
    {
		moving_aver_temp_Y[moving_aver_cnt_Y] = data;
		moving_aver_sum_Y += data;
		moving_aver_cnt_Y++;
        moving_aver_result_Y = moving_aver_sum_Y / repeat_balance_points;
        moving_aver_compl_cnt_Y++;
    }
	else
	{
    	moving_aver_sum_Y = moving_aver_sum_Y - moving_aver_temp_Y[0]  + data;
		moving_aver_result_Y = moving_aver_sum_Y / repeat_balance_points;
		for(i=0; i<(repeat_balance_points - 1);i++)
		{
			moving_aver_temp_Y[i] = moving_aver_temp_Y[i+1];
		}
		moving_aver_temp_Y[repeat_balance_points - 1] = data;
		moving_aver_compl_cnt_Y++;//moving_aver_compl_cnt_X = 1,is arithmetic average; while moving_aver_compl_cnt_X > 1, is moving average.
	}
}


/*average Y*/
void Repeat_balance_Aver_Y(double data)
{
	u8 i;
	if(repeat_balance_aver_cnt_Y == 0)
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y = 0.0;
		repeat_aver_result_Y = 0.0;
		repeat_balance_aver_cnt_Y++;
		repeat_aver_sum_Y += data;
	}
	else if(repeat_balance_aver_cnt_Y != REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT - 1)
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y += data;
		repeat_balance_aver_cnt_Y++;
	}
	else
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y += data;
		repeat_aver_result_Y = repeat_aver_sum_Y / REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT;
		repeat_aver_compl_cnt_Y++;//moving_aver_compl_cnt_X = 1,is arithmetic average; while moving_aver_compl_cnt_X > 1, is moving average.
        if (repeat_aver_compl_cnt_Y == repeat_balance_points) {
            Workspace = DETERMINE_WORK_MODE;
            repeat_aver_compl_cnt_Y = 0;
        }
        Repeat_balance_Moving_Aver_Y(repeat_aver_result_Y);
        repeat_balance_aver_cnt_Y = 0;
	}
}


/*moving average A_Single*/
void Aver_A_Single(double data)
{
    /* moving average */
	u8 i;
	if(aver_A_Single_cnt == 0)
	{
        A_Single_array[aver_A_Single_cnt] = data;
		aver_sum_A_Single = 0.0;
		Average_A_single = 0.0;
		aver_A_Single_cnt++;
		aver_sum_A_Single += data;
	}
	else if(aver_A_Single_cnt < AVER_ABCD_SINGLE_POINT - 1)
	{
        A_Single_array[aver_A_Single_cnt] = data;
		aver_sum_A_Single += data;
		aver_A_Single_cnt++;
	}
	else if(aver_A_Single_cnt == AVER_ABCD_SINGLE_POINT - 1)
	{
        A_Single_array[aver_A_Single_cnt] = data;
		aver_sum_A_Single += data;
        aver_A_Single_cnt++;
		Average_A_single = aver_sum_A_Single / AVER_ABCD_SINGLE_POINT;
        // Average_A_single = Round_retained_three_decimal(Average_A_single);
	}
    else
    {
        aver_sum_A_Single = aver_sum_A_Single - A_Single_array[0] + data;
        Average_A_single = aver_sum_A_Single / AVER_ABCD_SINGLE_POINT;
        // Average_A_single = Round_retained_three_decimal(Average_A_single);
		for(i=0; i<(AVER_ABCD_SINGLE_POINT - 1);i++)
		{
			A_Single_array[i] = A_Single_array[i+1];
		}
        A_Single_array[AVER_ABCD_SINGLE_POINT - 1] = data;
    }
}

/*moving average B_Single*/
void Aver_B_Single(double data)
{
	u8 i;
	if(aver_B_Single_cnt == 0)
	{
        B_Single_array[aver_B_Single_cnt] = data;
		aver_sum_B_Single = 0.0;
		Average_B_single = 0.0;
		aver_B_Single_cnt++;
		aver_sum_B_Single += data;
	}
	else if(aver_B_Single_cnt < AVER_ABCD_SINGLE_POINT - 1)
	{
        B_Single_array[aver_B_Single_cnt] = data;
		aver_sum_B_Single += data;
		aver_B_Single_cnt++;
	}
	else if(aver_B_Single_cnt == AVER_ABCD_SINGLE_POINT - 1)
	{
        B_Single_array[aver_B_Single_cnt] = data;
		aver_sum_B_Single += data;
        aver_B_Single_cnt++;
		Average_B_single = aver_sum_B_Single / AVER_ABCD_SINGLE_POINT;
        // Average_B_single = Round_retained_three_decimal(Average_B_single);
	}
    else
    {
        aver_sum_B_Single = aver_sum_B_Single - B_Single_array[0] + data;
        Average_B_single = aver_sum_B_Single / AVER_ABCD_SINGLE_POINT;
        // Average_B_single = Round_retained_three_decimal(Average_B_single);
		for(i=0; i<(AVER_ABCD_SINGLE_POINT - 1);i++)
		{
			B_Single_array[i] = B_Single_array[i+1];
		}
        B_Single_array[AVER_ABCD_SINGLE_POINT - 1] = data;
    }
}

/*moving average C_Single*/
void Aver_C_Single(double data)
{
	u8 i;
	if(aver_C_Single_cnt == 0)
	{
        C_Single_array[aver_C_Single_cnt] = data;
		aver_sum_C_Single = 0.0;
		Average_C_single = 0.0;
		aver_C_Single_cnt++;
		aver_sum_C_Single += data;
	}
	else if(aver_C_Single_cnt < AVER_ABCD_SINGLE_POINT - 1)
	{
        C_Single_array[aver_C_Single_cnt] = data;
		aver_sum_C_Single += data;
		aver_C_Single_cnt++;
	}
	else if(aver_C_Single_cnt == AVER_ABCD_SINGLE_POINT - 1)
	{
        C_Single_array[aver_C_Single_cnt] = data;
		aver_sum_C_Single += data;
        aver_C_Single_cnt++;
		Average_C_single = aver_sum_C_Single / AVER_ABCD_SINGLE_POINT;
        // Average_C_single = Round_retained_three_decimal(Average_C_single);
	}
    else
    {
        aver_sum_C_Single = aver_sum_C_Single - C_Single_array[0] + data;
        Average_C_single = aver_sum_C_Single / AVER_ABCD_SINGLE_POINT;
        // Average_C_single = Round_retained_three_decimal(Average_C_single);
		for(i=0; i<(AVER_ABCD_SINGLE_POINT - 1);i++)
		{
			C_Single_array[i] = C_Single_array[i+1];
		}
        C_Single_array[AVER_ABCD_SINGLE_POINT - 1] = data;
    }
}

/*moving average D_Single*/
void Aver_D_Single(double data)
{
	u8 i;
	if(aver_D_Single_cnt == 0)
	{
        D_Single_array[aver_D_Single_cnt] = data;
		aver_sum_D_Single = 0.0;
		Average_D_single = 0.0;
		aver_D_Single_cnt++;
		aver_sum_D_Single += data;
	}
	else if(aver_D_Single_cnt < AVER_ABCD_SINGLE_POINT - 1)
	{
        D_Single_array[aver_D_Single_cnt] = data;
		aver_sum_D_Single += data;
		aver_D_Single_cnt++;
	}
	else if(aver_D_Single_cnt == AVER_ABCD_SINGLE_POINT - 1)
	{
        D_Single_array[aver_D_Single_cnt] = data;
		aver_sum_D_Single += data;
        aver_D_Single_cnt++;
		Average_D_single = aver_sum_D_Single / AVER_ABCD_SINGLE_POINT;
        // Average_D_single = Round_retained_three_decimal(Average_D_single);
	}
    else
    {
        aver_sum_D_Single = aver_sum_D_Single - D_Single_array[0] + data;
        Average_D_single = aver_sum_D_Single / AVER_ABCD_SINGLE_POINT;
        // Average_D_single = Round_retained_three_decimal(Average_D_single);
		for(i=0; i<(AVER_ABCD_SINGLE_POINT - 1);i++)
		{
			D_Single_array[i] = D_Single_array[i+1];
		}
        D_Single_array[AVER_ABCD_SINGLE_POINT - 1] = data;
    }
}

/*round retained three decimal*/
double Round_retained_three_decimal(double num_of_double)
{
    long temp_of_long;
    double temp_of_double;
    if(num_of_double > 0)
    {
        temp_of_long = (long)((num_of_double + 0.00005) * 10000);
    }
    else
    {
        temp_of_long = (long)((num_of_double - 0.00005) * 10000);
    }

    temp_of_double = (double)(temp_of_long / 10000.0);
    return temp_of_double;
}

/*repeat balance average X*/
void Repeat_balance_no_moving_Aver_X(double data)
{
	u8 i;
	if(repeat_balance_aver_cnt_X == 0)
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X = 0.0;
		moving_aver_result_X = 0.0;
		repeat_balance_aver_cnt_X++;
		repeat_aver_sum_X += data;
	}
	else if(repeat_balance_aver_cnt_X != REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT - 1)
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X += data;
		repeat_balance_aver_cnt_X++;
	}
	else
	{
		repeat_balance_temp_X[repeat_balance_aver_cnt_X] = data;
		repeat_aver_sum_X += data;
		moving_aver_result_X = repeat_aver_sum_X / REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT;
		moving_aver_compl_cnt_X++;
        repeat_balance_aver_cnt_X = 0;
	}
}

/*repeat balance average Y*/
void Repeat_balance_no_moving_Aver_Y(double data)
{
	u8 i;
	if(repeat_balance_aver_cnt_Y == 0)
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y = 0.0;
		moving_aver_result_Y = 0.0;
		repeat_balance_aver_cnt_Y++;
		repeat_aver_sum_Y += data;
	}
	else if(repeat_balance_aver_cnt_Y != REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT - 1)
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y += data;
		repeat_balance_aver_cnt_Y++;
	}
	else
	{
		repeat_balance_temp_Y[repeat_balance_aver_cnt_Y] = data;
		repeat_aver_sum_Y += data;
		moving_aver_result_Y = repeat_aver_sum_Y / REPEAT_BALANCE_AVER_NUM_GENERATE_ONE_POINT;
		moving_aver_compl_cnt_Y++;//moving_aver_compl_cnt_X = 1,is arithmetic average; while moving_aver_compl_cnt_X > 1, is moving average.
        repeat_balance_aver_cnt_Y = 0;
	}
}
