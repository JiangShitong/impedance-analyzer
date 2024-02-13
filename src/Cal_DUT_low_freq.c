/*
 * Cal_DUT.c
 *
 *  Created on: 2019Äê1ÔÂ15ÈÕ
 *      Author: Zachary
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "xbasic_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "ARM_interface.h"
#include "UART.h"

#define   FPGA_GAIN_COE   13107  //16bits DAC  FPGA_GAIN_COE=(2^16-1)/5	zdw

volatile double low_freq_Vx_X_single = 0.0;
volatile double low_freq_Vx_Y_single = 0.0;
volatile double low_freq_Vr_X_single = 0.0;
volatile double low_freq_Vr_Y_single = 0.0;
volatile double low_freq_Lp_X_single = 0.0;
volatile double low_freq_Lp_Y_single = 0.0;

volatile double low_freq_A_single = 0.0;
volatile double low_freq_B_single = 0.0;
volatile double low_freq_C_single = 0.0;
volatile double low_freq_D_single = 0.0;

volatile double low_freq_numer_X_single = 0.0;
volatile double low_freq_numer_Y_single = 0.0;
volatile double low_freq_denom_single = 0.0;

//volatile double DUT_single_X = 0.0;
//volatile double DUT_single_Y = 0.0;

volatile double low_freq_Vx_X_rep = 0.0;
volatile double low_freq_Vx_Y_rep = 0.0;
volatile double low_freq_Vr_X_rep = 0.0;
volatile double low_freq_Vr_Y_rep = 0.0;
volatile double low_freq_Lp_X_rep = 0.0;
volatile double low_freq_Lp_Y_rep = 0.0;

volatile double low_freq_A_rep = 0.0;
volatile double low_freq_B_rep= 0.0;
volatile double low_freq_C_rep= 0.0;
volatile double low_freq_D_rep = 0.0;

volatile double low_freq_numer_X_rep = 0.0;
volatile double low_freq_numer_Y_rep = 0.0;
volatile double low_freq_denom_rep = 0.0;

//volatile double DUT_rep_X = 0.0;
//volatile double DUT_rep_Y = 0.0;

volatile double low_freq_DUT_X = 0.0;
volatile double low_freq_DUT_Y = 0.0;
volatile double DUT_low_freq_display_1 = 0.0;
volatile double DUT_low_freq_display_2 = 0.0;
volatile u8 low_freq_Cal_rep_state = 0;
volatile u32 low_freq_Cal_Vx_gain = 0;

volatile double low_freq_DUT_X_aver = 0.0;
volatile double low_freq_DUT_Y_aver = 0.0;
volatile u8 low_freq_cnt = 0;
volatile double low_freq_sum_X = 0.0;
volatile double low_freq_sum_Y = 0.0;

volatile u8 low_freq_moving_aver_cnt_X =  0;
volatile double low_freq_moving_aver_data_X_1st = 0.0;
volatile double low_freq_moving_aver_sum_X = 0.0;
volatile double low_freq_moving_aver_result_X = 0.0;
volatile u8 low_freq_moving_aver_cnt_Y =  0;
volatile double low_freq_moving_aver_data_Y_1st = 0.0;
volatile double low_freq_moving_aver_sum_Y = 0.0;
volatile double low_freq_moving_aver_result_Y = 0.0;

extern double DFT_A_X_display;
extern double DFT_A_Y_display;

extern u8 Master_switch_flag;
extern u32 IA_CTRL_Analog;
extern Xfloat32 Vx_Vr_freq;
extern u8 sequence_1;
extern u8 sequence_2;
extern u16 Rr;
extern double RAD2DEG;

//for sweep
extern u8 Balance_dft_delay_flag;
extern u8 Cal_dft_delay_flag;
extern u8 Workspace;
extern u8 Measure_Term;
extern u8 Cal_dft_delay_flag;
extern u8 Cal_Rr_dft_delay_flag;

volatile double debug_flag = 0;

void Cal_single_low_freq(void);


void Cal_DUT_low_freq(void)
{
	if(Master_switch_flag)
	{
		Cal_single_low_freq();
	}
}

/*Execute this function at low frequencies*/
void Cal_single_low_freq(void)
{

    switch (Measure_Term) {
        case DUT_MEASRUE_TERM:
            /*--------  mesure Vx, for cal_dut  --------*/
            IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;//1(bias_on)_11(no meaning)_0000(Vx)_111(500R)11(none)
            IA_CTRL_Analog += 0;
            Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

            Workspace = CAL_DUT;
            Master_switch_flag = 1;
            Cal_dft_delay_flag = 0;
            break;
        case Rr_MEASRUE_TERM:
            /*--------  mesure Vx, for cal_Rr  --------*/
            IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;//1(bias_on)_11(no meaning)_0000(Vx)_111(500R)11(none)
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
