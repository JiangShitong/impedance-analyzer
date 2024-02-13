#include <xscutimer.h>
#include <xparameters_ps.h>
#include "xparameters.h"
#include "xil_printf.h"
#include "xparameters_ps.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xbasic_types.h"
#include "xscugic.h"
#include "Uart_Timer_intr_Initialize.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xscugic.h"
#include "ARM_interface.h"
#include "UART.h"
#include <sleep.h>

#define DeviceId 0
#define Time_constant_delay_cnt 70000 //1ms PSD is 18000, may be reduced, but must >15000

extern void UartHandler(void *CallBackRef, u32 Event, unsigned int EventData);
extern void UART(void);
extern void Cal_moving_aver_initialize(void);

XUartPs Uart_Ps;
XUartPs_Config *Config;

XScuGic InterruptController;	/* Instance of the Interrupt Controller */
XScuGic_Config *IntcConfig;

static  XScuTimer Timer;//timer
XScuTimer_Config *TMRConfigPtr;     //timer config
unsigned int TIMER_LOAD_VALUE;
volatile int sec = 0;
volatile u32 IntrMask;

volatile u32 dft_delay_count = 0;
volatile u32 Cal_dft_delay_count = 0;
volatile u32 Cal_Rr_dft_delay_count = 0;
volatile u8 Balance_dft_delay_flag = 0;
volatile u8 Cal_dft_delay_flag = 0;
volatile u8 Cal_Rr_dft_delay_flag = 0;
extern int DFT_delay_us;
volatile u32 delay_5s_count = 0;
extern u8 Master_switch_flag;


/*--------  for sweep  --------*/
extern u32 point_of_period;
extern u32 Vx_Vr_phase_inc;
extern u32 Vmod_phase_inc;
extern u32 Vref_phase_inc;
extern u32 sweep_freq_times;
extern u32 Vx_gain;
extern Xfloat32 *freq_sweep_pointer_temp;
extern Xfloat32 *amp_sweep_pointer_temp;

/*--------  for Auto_Rr  --------*/
volatile u8 Auto_Rr_dft_delay_flag = 0;
volatile u32 Auto_Rr_delay_count = 0;

/*--------  for measure stray C  --------*/
extern u8 Measure_stray_Cb_delay_flag;
volatile u32 Measure_stray_Cb_delay_count = 0;

extern u8 Workspace;

extern double debug_flag;

/*Timer interrupt function*/
static void TimerIntrHandler(void *CallBackRef)
{
    XScuTimer *TimerInstancePtr = (XScuTimer *) CallBackRef;
    XScuTimer_ClearInterruptStatus(TimerInstancePtr);

    if(Master_switch_flag && (Balance_dft_delay_flag == 0) && (Workspace == BALANCED_ALGORITHM))
    {
        dft_delay_count++;

    }
    if(Master_switch_flag && (Cal_dft_delay_flag == 0) && (Workspace == CAL_DUT))
    {
        Cal_dft_delay_count++;
    }
    if(Master_switch_flag && (Cal_Rr_dft_delay_flag == 0) && (Workspace == CAL_Rr))
    {
        Cal_Rr_dft_delay_count++;
    }
    if (Master_switch_flag && (Auto_Rr_dft_delay_flag == 0) && (Workspace == AUTO_Rr))
    {
        Auto_Rr_delay_count++;
    }
    if (Master_switch_flag && (Measure_stray_Cb_delay_flag == 0) && (Workspace == MEASURE_STRAY_C))
    {
        Measure_stray_Cb_delay_count++;
    }
    
    if (dft_delay_count >= Time_constant_delay_cnt) {

    	//usleep(20000);
        Balance_dft_delay_flag = 1;
        dft_delay_count = 0;
    }
    if (Cal_dft_delay_count >= Time_constant_delay_cnt)
    {
        Cal_dft_delay_flag = 1;
        Cal_dft_delay_count = 0;
    }
    if (Cal_Rr_dft_delay_count >= Time_constant_delay_cnt)
    {
        Cal_Rr_dft_delay_flag = 1;
        Cal_Rr_dft_delay_count = 0;
    }
    if (Auto_Rr_delay_count >= Time_constant_delay_cnt)
    {
        Auto_Rr_dft_delay_flag = 1;
        Auto_Rr_delay_count = 0;
    }
    if (Measure_stray_Cb_delay_count >= Time_constant_delay_cnt)
    {
        Measure_stray_Cb_delay_flag = 1;
        Measure_stray_Cb_delay_count = 0;
    }
}

/*uart and timer interupt init*/
void Uart_Timer_intr_Initialize()
{
    /*--------  set timer interupt  --------*/
    TIMER_LOAD_VALUE =  333;//1us 333333333->1s
    TIMER_LOAD_VALUE -= 1;
    TMRConfigPtr = XScuTimer_LookupConfig(0);//read timer config instance in CPU0
    XScuTimer_CfgInitialize(&Timer, TMRConfigPtr,TMRConfigPtr->BaseAddr);//init
    XScuTimer_SetPrescaler(&Timer, 0);//set prescale
    XScuTimer_LoadTimer(&Timer, TIMER_LOAD_VALUE);//set initial value of accumulator register
    XScuTimer_EnableAutoReload(&Timer);//Set auto reload
    XScuTimer_EnableInterrupt(&Timer);//enable interrupt on the timer


    
    /*--------  set uart1 interupt  --------*/
	Config = XUartPs_LookupConfig(DeviceId);
	XUartPs_CfgInitialize(&Uart_Ps, Config, Config->BaseAddress);
	XUartPs_SetHandler(&Uart_Ps, (XUartPs_Handler)UartHandler, &Uart_Ps);
	XUartPs_SetFifoThreshold(&Uart_Ps, 1);

	IntrMask = XUARTPS_IXR_RXOVR | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_TXFULL;
			// | XUARTPS_IXR_TOUT | XUARTPS_IXR_RXFULL | XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY;
	if ((&Uart_Ps)->Platform == XPLAT_ZYNQ_ULTRA_MP) {
			IntrMask |= XUARTPS_IXR_RBRK;
		}
	XUartPs_SetInterruptMask(&Uart_Ps, IntrMask);
	XUartPs_SetOperMode(&Uart_Ps, XUARTPS_OPER_MODE_NORMAL);


    /*--------  set Gic  --------*/
	IntcConfig = XScuGic_LookupConfig(DeviceId);
	XScuGic_CfgInitialize(&InterruptController, IntcConfig,
					IntcConfig->CpuBaseAddress);

    XScuGic_Connect(&InterruptController, 29,
                    (Xil_ExceptionHandler)TimerIntrHandler,//set up the timer interrupt
                    (void *)&Timer);
	XScuGic_Connect(&InterruptController, 82,
        			(Xil_ExceptionHandler) XUartPs_InterruptHandler,
				    (void *)&Uart_Ps);
    //Contact the interrupt source of number 29 (the timer interrupt number is 29) with its corresponding handler
    XScuGic_SetPriorityTriggerType(&InterruptController, 29, 0XA0, 3);//Set timer trigger conditions and priorities
    XScuGic_SetPriorityTriggerType(&InterruptController, 82, 0x90, 1);//Set the uart1 trigger condition and priority

    XScuGic_Enable(&InterruptController,29);
    XScuGic_Enable(&InterruptController,82);


    /*--------  Register c language array interrupt to scale  --------*/
     Xil_ExceptionInit();
     Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,(Xil_ExceptionHandler)XScuGic_InterruptHandler,&InterruptController);
     Xil_ExceptionEnable();

     Timer_start(&Timer);
}

void Timer_start(XScuTimer *TimerPtr)
{
	 XScuTimer_Start(TimerPtr);
}

void Timer_stop(XScuTimer *TimerPtr)
{
	 XScuTimer_Stop(TimerPtr);
}
