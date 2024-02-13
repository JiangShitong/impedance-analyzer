/*
 * FPGA.c
 *
 *  Created on: 2018Äê12ÔÂ23ÈÕ
 *      Author: Zachary
 */


#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <ctype.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xparameters_ps.h"
#include "ARM_interface.h"
#include "xbasic_types.h"
#include <sleep.h>

#define INTERCEPT 0.003//fitting AD amplitude

const double FPGA_COE = 4.768371582e-10;
const double FPGA_PHASE_COE = 5.493247883e-3;
const double DEG2RAD = (3.1415926f / 180);
const double RAD2DEG = (180 / 3.1415926f);

volatile unsigned long long A_X_h, B_X_h, A_Y_h, B_Y_h, data_l;
volatile unsigned long long DFT_A_X_h, DFT_B_X_h, DFT_A_Y_h, DFT_B_Y_h, DFT_A_X_l, DFT_B_X_l, DFT_A_Y_l, DFT_B_Y_l;
volatile unsigned char PSD_aver_flag = FALSE;



volatile double FPGA_A_X = 0.0;
volatile double FPGA_A_Y = 0.0;
volatile double FPGA_B_X = 0.0;
volatile double FPGA_B_Y = 0.0;
volatile double FPGA_Vr_phase = 0.0;

volatile double DFT_A_X = 0.0;
volatile double DFT_A_Y = 0.0;
volatile double DFT_B_X = 0.0;
volatile double DFT_B_Y = 0.0;

volatile double PSD_A_R = 0.0;
volatile double PSD_B_R = 0.0;
volatile double PSD_A_Vpp = 0.0;
volatile double PSD_B_Vpp = 0.0;
volatile double Cal_PSD_A_Vpp = 0.0;
volatile double Cal_PSD_B_Vpp = 0.0;
volatile double Cal_PSD_A_Vrms = 0.0;
volatile double Cal_PSD_B_Vrms = 0.0;
volatile double	PSD_A_Xita = 0.0;
volatile double	PSD_B_Xita = 0.0;

volatile double PSD_A_X_display = 0.0;
volatile double PSD_A_Y_display = 0.0;
volatile double PSD_B_X_display = 0.0;
volatile double PSD_B_Y_display = 0.0;

volatile double Cal_PSD_A_X_display = 0.0;
volatile double Cal_PSD_A_Y_display = 0.0;
volatile double Cal_PSD_B_X_display = 0.0;
volatile double Cal_PSD_B_Y_display = 0.0;

volatile double DFT_A_X_display = 0.0;
volatile double DFT_A_Y_display = 0.0;
volatile double DFT_B_X_display = 0.0;
volatile double DFT_B_Y_display = 0.0;

volatile double DFT_A_R = 0.0;
volatile double DFT_B_R = 0.0;
volatile double DFT_A_Vpp = 0.0;
volatile double DFT_B_Vpp = 0.0;
volatile double	DFT_A_Xita = 0.0;
volatile double	DFT_B_Xita = 0.0;

volatile unsigned long long PSD_aver_cnt = 0;

extern unsigned char freq_mode_flag;
extern u8 freq_mixed_flag;
extern u32 point_of_period;
extern int DFT_delay_us;
extern u32 IA_CTRL_Analog;

extern u8 Balance_dft_delay_flag;

void Read_data(void);
double Data_PSD_FPGA2ARM(unsigned long long data_h, unsigned long long data_l, u8 data_l_shift);
double Data_DFT_FPGA2ARM(unsigned long long data_h, unsigned long long data_l);
void Data_combine(void);
void Cal_DUT_low_freq(void);
void Cal_DUT_high_freq(void);
void delay_us(u32 timeus);
double Round_retained_five_decimal(double num_of_double);

extern double debug_flag;

/*Balanced_algorithm function*/
void Balanced_algorithm(void)
{
	//usleep(3800);				//Set the delay  300us for PSD
	// usleep(14800);				//Set the delay  1ms for PSD
	//delay_us(1000000);				//Set the delay  3ms for PSD
	//delay_us(DFT_delay_us);
/*	IA_CTRL_Analog = IA_CTRL_Analog & 0x00000efc;
	IA_CTRL_Analog += 1;
	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);*/
	//Cal_DUT_high_freq();

    if(Balance_dft_delay_flag)
    {

	    Read_data();
	    Data_combine();
	    if(freq_mixed_flag)
	    {
            //Vr is from FPGA
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000efc;
			IA_CTRL_Analog += 1;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
	    	Cal_DUT_high_freq();
	    }
	    else
	    {
            //Vr is from small board
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000efc;
			IA_CTRL_Analog += 0;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);

	    	Cal_DUT_low_freq();
	    }
    }
}

/*read the signal*/
void Read_data(void)
{
	A_X_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG0_OFFSET));
	A_Y_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG1_OFFSET));
	B_X_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG2_OFFSET));
	B_Y_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG3_OFFSET));
	data_l = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG4_OFFSET));
	PSD_aver_flag = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG5_OFFSET));

	DFT_A_X_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG6_OFFSET));
	DFT_A_Y_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG7_OFFSET));
	DFT_B_X_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG8_OFFSET));
	DFT_B_Y_h = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG9_OFFSET));
	DFT_A_X_l = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG10_OFFSET));
	DFT_A_Y_l = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG11_OFFSET));
	DFT_B_X_l = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG12_OFFSET));
	DFT_B_Y_l = Xil_In32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG13_OFFSET));
}

/*Data_PSD_FPGA2ARM function*/
double Data_PSD_FPGA2ARM(unsigned long long data_h, unsigned long long data_l, u8 data_l_shift)
{
	unsigned long long data = 0;
	unsigned char data_sign_flag = FALSE;
	double FPGA_data = 0.0;
	data = (data_l >> data_l_shift) & 0x000f;
	data |= data_h << (4);
	data_sign_flag = (data_h >> 31) & (0x1);
	if(data_sign_flag)
	{
		data = ~(data);
		data += 1;
		data &= 0x00fffffffff;
		FPGA_data = -1 * (signed long long)data;
	}
	else
	{
		FPGA_data = (signed long long)data;
	}
	return (FPGA_data);
}

/*Data_DFT_FPGA2ARM function*/
double Data_DFT_FPGA2ARM(unsigned long long data_h, unsigned long long data_l)
{
	unsigned long long data = 0;
	unsigned char data_sign_flag = FALSE;
	double FPGA_data = 0.0;
	data = (data_h << 32) | (data_l & 0x00ffff);
	data_sign_flag = (data_h >> 31) & (0x1);
	if(data_sign_flag)
	{
		data = ~(data);
		data += 1;
		data &= 0xffffffffffffffff;
		FPGA_data = -1 * (signed long long)data;
	}
	else
	{
		FPGA_data = (signed long long)data;
	}
	return (FPGA_data);
}

/*Data_combine function*/
void Data_combine(void)
{
	FPGA_A_X = Data_PSD_FPGA2ARM(A_X_h, data_l, 0);
	FPGA_A_Y = Data_PSD_FPGA2ARM(A_Y_h, data_l, 4);
	FPGA_B_X = Data_PSD_FPGA2ARM(B_X_h, data_l, 8);
	FPGA_B_Y = Data_PSD_FPGA2ARM(B_Y_h, data_l, 12);

	DFT_A_X = Data_DFT_FPGA2ARM(DFT_A_X_h, DFT_A_X_l);
	DFT_A_Y = Data_DFT_FPGA2ARM(DFT_A_Y_h, DFT_A_Y_l);
	DFT_B_X = Data_DFT_FPGA2ARM(DFT_B_X_h, DFT_B_X_l);
	DFT_B_Y = Data_DFT_FPGA2ARM(DFT_B_Y_h, DFT_B_Y_l);


//******************************the X, Y Vrms data to display on LabVIEW***********************************************//
	PSD_A_X_display = (FPGA_A_X * FPGA_COE) / 1.414213562;//the 1.414213562 is sqrt(2),so it is Vrms
	PSD_A_Y_display = (FPGA_A_Y * FPGA_COE) / 1.414213562;
	PSD_B_X_display = (FPGA_B_X * FPGA_COE) / 1.414213562;
	PSD_B_Y_display = (FPGA_B_Y * FPGA_COE) / 1.414213562;

    /*--------  Vpp for A,B path  --------*/
	Cal_PSD_A_X_display = FPGA_A_X * FPGA_COE * 2.0;
	Cal_PSD_A_Y_display = FPGA_A_Y * FPGA_COE * 2.0;
	Cal_PSD_B_X_display = FPGA_B_X * FPGA_COE * 2.0;
	Cal_PSD_B_Y_display = FPGA_B_Y * FPGA_COE * 2.0;
    /*--------  Round ratained A_X,A_Y,B_X,B_Y  --------*/
    Cal_PSD_A_X_display = Round_retained_five_decimal(Cal_PSD_A_X_display);
    Cal_PSD_A_Y_display = Round_retained_five_decimal(Cal_PSD_A_Y_display);
    Cal_PSD_B_X_display = Round_retained_five_decimal(Cal_PSD_B_X_display);
    Cal_PSD_B_Y_display = Round_retained_five_decimal(Cal_PSD_B_Y_display);
    Cal_PSD_A_Vpp = sqrt(pow(Cal_PSD_A_X_display,2.0) + pow(Cal_PSD_A_Y_display,2.0));
    Cal_PSD_B_Vpp = sqrt(pow(Cal_PSD_B_X_display,2.0) + pow(Cal_PSD_B_Y_display,2.0));
    //A,B path Vpp
    Cal_PSD_A_Vpp = sqrt(pow(Cal_PSD_A_X_display,2.0) + pow(Cal_PSD_A_Y_display,2.0));
    Cal_PSD_B_Vpp = sqrt(pow(Cal_PSD_B_X_display,2.0) + pow(Cal_PSD_B_Y_display,2.0));
    //A,B path Vrms
    Cal_PSD_A_Vrms = sqrt(pow(PSD_A_X_display,2.0) + pow(PSD_A_Y_display,2.0));
    Cal_PSD_B_Vrms = sqrt(pow(PSD_B_X_display,2.0) + pow(PSD_B_Y_display,2.0));


	DFT_A_X_display =  (DFT_A_X * FPGA_COE) / 1.414213562 / (double)point_of_period;
	DFT_A_Y_display =  (DFT_A_Y * FPGA_COE) / 1.414213562 / (double)point_of_period;
	DFT_B_X_display =  (DFT_B_X * FPGA_COE) / 1.414213562 / (double)point_of_period;
	DFT_B_Y_display =  (DFT_B_Y * FPGA_COE) / 1.414213562 / (double)point_of_period;

	DFT_A_R = FPGA_COE * sqrt(pow(DFT_A_X, 2) + pow(DFT_A_Y, 2)) / 1.414213562 / (double)point_of_period;
	DFT_B_R = FPGA_COE * sqrt(pow(DFT_B_X, 2) + pow(DFT_B_Y, 2)) / 1.414213562 / (double)point_of_period;
	DFT_A_Vpp = DFT_A_R * 1.414213562 * 2;
	DFT_B_Vpp = DFT_B_R * 1.414213562 * 2;
	DFT_A_Xita = atan2(DFT_A_Y, DFT_A_X) * RAD2DEG;
	DFT_B_Xita = atan2(DFT_B_Y, DFT_B_X) * RAD2DEG;
}

/*Round_retained_five_decimal function*/
double Round_retained_five_decimal(double num_of_double)
{
    long temp_of_long;
    double temp_of_double;
    if(num_of_double > 0)
    {
        temp_of_long = (long)((num_of_double + 0.000005) * 100000);
    }
    else
    {
        temp_of_long = (long)((num_of_double - 0.000005) * 100000);
    }

    temp_of_double = (double)(temp_of_long / 100000.0);
    return temp_of_double;
}
