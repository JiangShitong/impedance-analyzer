#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "xbasic_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "ARM_interface.h"
#include "UART.h"
#include <sleep.h>

volatile u8 Auto_Rr_state = 0;
extern u8 Auto_Rr_dft_delay_flag;
extern u8 freq_mixed_flag;
extern double Vx_X_single;
extern double Vx_Y_single;
extern double Lp_X_single;
extern double Lp_Y_single;
extern double Vr_X_single;
extern double Vr_Y_single;
extern double Cal_PSD_A_X_display;
extern double Cal_PSD_A_Y_display;
extern double Cal_PSD_B_X_display;
extern double Cal_PSD_B_Y_display;
extern u32 IA_CTRL_Analog;
extern u16 Rr;

extern u8 Workspace;
volatile double Ratio_Rr_parallel_stray_C_Dut_X, Ratio_Rr_parallel_stray_C_Dut_Y;
volatile double Absolute_value_Ratio_Rr_parallel_stray_C_Dut;
extern double Stray_Z;

extern void Set_flag_and_Prepare_to_enter_Balance_Algorithm();
extern void Set_flag_and_Prepare_to_enter_Measure_stray_C();
extern u8 Measure_Term;

void Auto_Rr(void)
{
    double Vx, Vx_decrease;
    double E, F;
    double Ratio_Rr_Dut_X, Ratio_Rr_Dut_Y;
    double Absolute_value_Ratio_Rr_Dut;
    double G, I;
    // double C = 0.000000000220;

    switch ( Auto_Rr_state )
    {
    case 0:
        /*--------  record Vx  --------*/
        if(freq_mixed_flag)
        {
		    Vx_X_single = Cal_PSD_A_X_display;
		    Vx_Y_single = -1.0 * Cal_PSD_A_Y_display;
        }
        else
        {
            Vx_X_single = Cal_PSD_A_X_display;
            Vx_Y_single = Cal_PSD_A_Y_display;
        }
		/*	Measure Lp		*/
		IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
		IA_CTRL_Analog += 64;
		Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        Auto_Rr_state++;
        break;

    case 1:
        /*--------  record Lp  --------*/
        if(freq_mixed_flag)
        {
		    Lp_X_single = -1.0 * Cal_PSD_A_X_display;
		    Lp_Y_single = Cal_PSD_A_Y_display;
        }
        else
        {
            Lp_X_single = -1.0 * Cal_PSD_A_X_display;
            Lp_Y_single = -1.0 * Cal_PSD_A_Y_display;
        }
		/*	Measure Vr		*/
		IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
		IA_CTRL_Analog += 32;
		Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        Auto_Rr_state++;
        break;
    
    case 2:
        /*--------  record Vr,but it is not helpful for auto Rr  --------*/
        if(freq_mixed_flag)
        {
		    Vr_X_single = Cal_PSD_A_X_display;
		    Vr_Y_single = -1.0 * Cal_PSD_A_Y_display;
        }
        else
        {
            Vr_X_single = Cal_PSD_A_X_display;
            Vr_Y_single = Cal_PSD_A_Y_display;
        }
        

        E = Vx_X_single - Lp_X_single;
        F = Vx_Y_single - Lp_Y_single;
        Vx = sqrt(pow(Vx_X_single, 2) + pow(Vx_Y_single, 2));
        G = sqrt(9.0/(pow(Vx, 2)));
        // I = 3.1415926 * 2.0 * Vx_Vr_freq * C * 5000.0;//set Rr 5k when auto Rr.
        I = 1.0 / Stray_Z * Rr;

        Ratio_Rr_parallel_stray_C_Dut_X = (Lp_X_single * E + Lp_Y_single * F)/(pow(E, 2) + pow(F, 2));
        Ratio_Rr_parallel_stray_C_Dut_Y = (Lp_Y_single * E - Lp_X_single * F)/(pow(E, 2) + pow(F, 2));
        Ratio_Rr_Dut_X = Ratio_Rr_parallel_stray_C_Dut_X * 1.0 - Ratio_Rr_parallel_stray_C_Dut_Y * I;
        Ratio_Rr_Dut_Y = Ratio_Rr_parallel_stray_C_Dut_X * I + Ratio_Rr_parallel_stray_C_Dut_Y * 1.0;

        Absolute_value_Ratio_Rr_parallel_stray_C_Dut = sqrt(pow(Ratio_Rr_parallel_stray_C_Dut_X, 2) + pow(Ratio_Rr_parallel_stray_C_Dut_Y, 2));
        Absolute_value_Ratio_Rr_Dut = sqrt(pow(Ratio_Rr_Dut_X, 2) + pow(Ratio_Rr_Dut_Y, 2));

        if ((Absolute_value_Ratio_Rr_Dut > 1.0) && (Absolute_value_Ratio_Rr_Dut <= 10.0))
        {
            // printf("500r\n");
            //Vr is 500r
            Rr = 500;
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000ee3;
			IA_CTRL_Analog += 0;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        }
        else if((Absolute_value_Ratio_Rr_Dut <= 1.0) && (Absolute_value_Ratio_Rr_Dut > 0.1))
        {
            // printf("5k\n");
            //Vr is 5k
            Rr = 5000;
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000ee3;
			IA_CTRL_Analog += 4;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        }
        else if((Absolute_value_Ratio_Rr_Dut > 0.0) && (Absolute_value_Ratio_Rr_Dut <= 0.1))
        {
            // printf("50k\n");
            //Vr is 50k
            Rr = 50000;
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000ee3;
			IA_CTRL_Analog += 8;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        }
        else if(Absolute_value_Ratio_Rr_Dut > 10.0)
        {
            if (sqrt(Absolute_value_Ratio_Rr_Dut / 10.0) > G)
            {
                Vx_decrease = sqrt(90.0/Absolute_value_Ratio_Rr_Dut);
                // printf("decrease Vx to %f\n", Vx_decrease);
            }
            else
            {
                // printf("500r\n");
            //Vr is 500r
            Rr = 500;
			IA_CTRL_Analog = IA_CTRL_Analog & 0x00000ee3;
			IA_CTRL_Analog += 0;
			Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
            }
        }

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
                Set_flag_and_Prepare_to_enter_Balance_Algorithm();
                break;
        }

        Auto_Rr_state = 0;
        break;

    default :
        Auto_Rr_state = 0;
        break;
    }
    Auto_Rr_dft_delay_flag = 0;
}
