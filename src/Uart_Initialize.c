/*
 * Uart_Initialize.c
 *
 *  Created on: 2018Äê12ÔÂ30ÈÕ
 *      Author: Administrator
 */
#include "xparameters.h"
#include "xil_printf.h"
#include "xparameters_ps.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xbasic_types.h"
#include "xscugic.h"
#include "UART.h"

extern XUartPs Uart_Ps;
extern XScuGic InterruptController;	/* Instance of the Interrupt Controller */
XUartPs_Config *Config;
XScuGic_Config *IntcConfig;

//void Uart_Initialize(XScuGic *IntcInstPtr, XUartPs *UartInstPtr,u16 DeviceId, u16 UartIntrId);
static int SetupInterruptSystem(XScuGic *IntcInstancePtr,XUartPs *UartInstancePtr,u16 UartIntrId);
void XUartPs_SetFifoThreshold(XUartPs *InstancePtr, u8 TriggerLevel);

void Uart_Initialize(XScuGic *IntcInstPtr, XUartPs *UartInstPtr,u16 DeviceId, u16 UartIntrId)
{
	u32 IntrMask;
	Config = XUartPs_LookupConfig(DeviceId);
	XUartPs_CfgInitialize(UartInstPtr, Config, Config->BaseAddress);

//	UartInstPtr->ReceiveBuffer = 0U;
//	XUartPs_WriteReg(XPAR_PS7_UART_0_BASEADDR, XUARTPS_CR_OFFSET,(u32)XUARTPS_CR_RXRST);

	SetupInterruptSystem(IntcInstPtr, UartInstPtr, UartIntrId);
	XUartPs_SetHandler(UartInstPtr, (XUartPs_Handler)UartHandler, UartInstPtr);
	XUartPs_SetFifoThreshold(UartInstPtr, 1);
	IntrMask = XUARTPS_IXR_RXOVR | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_TXFULL;
			//| XUARTPS_IXR_TOUT | XUARTPS_IXR_RXFULL | XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY;

	if (UartInstPtr->Platform == XPLAT_ZYNQ_ULTRA_MP) {
			IntrMask |= XUARTPS_IXR_RBRK;
		}

	XUartPs_SetInterruptMask(UartInstPtr, IntrMask);

	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);
}

static int SetupInterruptSystem(XScuGic *IntcInstancePtr,
				XUartPs *UartInstancePtr,
				u16 UartIntrId)
{
	int Status;

#ifndef TESTAPP_GEN
	XScuGic_Config *IntcConfig; /* Config for interrupt controller */

	/* Initialize the interrupt controller driver */
	IntcConfig = XScuGic_LookupConfig(INTC_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the
	 * hardware interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler) XScuGic_InterruptHandler,
				IntcInstancePtr);
#endif

	/*
	 * Connect a device driver handler that will be called when an
	 * interrupt for the device occurs, the device driver handler
	 * performs the specific interrupt processing for the device
	 */
	Status = XScuGic_Connect(IntcInstancePtr, UartIntrId,
			(Xil_ExceptionHandler) XUartPs_InterruptHandler,
				  (void *) UartInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* Enable the interrupt for the device */
	XScuGic_Enable(IntcInstancePtr, UartIntrId);


#ifndef TESTAPP_GEN
	/* Enable interrupts */
	 Xil_ExceptionEnable();
#endif

	return XST_SUCCESS;
}


