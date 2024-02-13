#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "xbasic_types.h"
#include "xparameters.h"
#include "xil_printf.h"
#include "ARM_interface.h"
#include "UART.h"
#include <sleep.h>

volatile u8 Measure_Stray_C_state = 0;
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
extern Xfloat32 Vx_Vr_freq;
volatile Xfloat32 Standard_Dut = 100.0;
volatile u8 Z_stray_C_array_pointer = 0;//pointer of array Z stray C.
extern u8 Measure_stray_Cb_delay_flag;
extern Xfloat32 *Measure_stray_Cb_freq_pointer;//pointer to freq_array of Measure_stray_Cb.
volatile Xfloat32 Z_Cb_module;
extern Xfloat32 *freq_sweep_pointer_temp;
extern Xfloat32 *amp_sweep_pointer_temp;
extern u8 Data_interact_ON_flag;
extern u32 Data_interact_ON_cnt;//for send FLAG time out
extern u8 Master_switch_flag;
extern u8 Measure_Mode;
extern double Vx_Vpp, Vr_Vpp, Lp_Vpp;
extern double Vx_Vrms, Vr_Vrms, Lp_Vrms;

extern u32 sweep_freq_times;
extern Xfloat32 Sweep_mode_Vx_Vr_freq_display;
extern Xfloat32 Sweep_mode_Vx_amp_display;
extern u32 Vx_gain;

extern void complex_product(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_divide(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_plus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);
extern void complex_minus(double A_X, double A_Y, double B_X, double B_Y, double *C_X, double *C_Y);

/*Measure stray Lp_C*/
void Measure_Stray_C(void)
{
    Xfloat32 A_X, A_Y, B_X, B_Y;
    double Cb_X, Cb_Y;
    switch ( Measure_Stray_C_state )
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
        Vx_Vpp = sqrt(pow(Vx_X_single, 2) + pow(Vx_Y_single, 2));
		/*	Measure Lp		*/
		IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
		IA_CTRL_Analog += 64;
		Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        Measure_Stray_C_state++;
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
        Lp_Vpp = sqrt(pow(Lp_X_single, 2) + pow(Lp_Y_single, 2));
		/*	Measure Vr		*/
		IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
		IA_CTRL_Analog += 32;
		Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
        Measure_Stray_C_state++;
        break;
    
    case 2:
        /*--------  record Vr --------*/
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
        Vr_Vpp = sqrt(pow(Vr_X_single, 2) + pow(Vr_Y_single, 2));
        A_X = Vx_X_single * Rr + Vr_X_single * Standard_Dut - Lp_X_single * Rr - Lp_X_single * Standard_Dut;
        A_Y = Vx_Y_single * Rr + Vr_Y_single * Standard_Dut - Lp_Y_single * Rr - Lp_Y_single * Standard_Dut;
        B_X = -1.0 * 2.0 * 3.1415926 * Vx_Vr_freq * Standard_Dut * Rr * Lp_Y_single;
        B_Y = 2.0 * 3.1415926 * Vx_Vr_freq * Standard_Dut * Rr * Lp_X_single;
        complex_divide(A_X, A_Y, B_X, B_Y, &Cb_X, &Cb_Y);
        Z_Cb_module = 1.0/(2.0 * 3.1415926 * Vx_Vr_freq * Cb_X);//The Z_Cb_module actually the impedance module of Cb.


                switch (Measure_Mode) {
                    case SINGLE_MEASURE_MODE:
                        Workspace = NULL_WORKSPACE;
                        Measure_Mode = NONE_MEASURE_MODE;
                        Master_switch_flag = 0;
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out 
                        Data_interact_ON_flag = 1;
                        break;
                    case REPEAT_MEASURE_MODE:
                        Workspace = MEASURE_STRAY_C;
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out 
                        Data_interact_ON_flag = 1;
                        break;
                    case FREQ_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        freq_sweep_pointer_temp++;
                        Sweep_mode_Vx_Vr_freq_display = Vx_Vr_freq;//storge the freq currently
                        Sweep_mode_Vx_amp_display = (double)Vx_gain / FPGA_GAIN_COE;
                        sweep_freq_times++;
                        Send_Data2PC_str("FLAG;");
                        Data_interact_ON_cnt = 0;//start time for timing out 
                        Data_interact_ON_flag = 1;
                        break;
                    case AMP_SWEEP_MEASURE_MODE:
                        Workspace = DETERMINE_WORK_MODE;
                        amp_sweep_pointer_temp++;
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

                /*--------  measure Vx, for next cal_Rr  --------*/
                IA_CTRL_Analog = IA_CTRL_Analog & 0x00000e1f;
                IA_CTRL_Analog += 0;
                Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG26_OFFSET), IA_CTRL_Analog);
                Measure_Stray_C_state = 0;
        break;
    default :
        Measure_Stray_C_state = 0;
        break;
    }
    Measure_stray_Cb_delay_flag = 0;
}
