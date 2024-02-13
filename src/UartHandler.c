/*
 * UartHandler.c
 *
 *  Created on: 2018Äê12ÔÂ30ÈÕ
 *      Author: Administrator
 */
#include "xparameters.h"
#include "xparameters_ps.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "xscugic.h"
#include "UART.h"

extern u8 Received_Count;//END POINT
extern u8 Recv_Buffer[];
extern XUartPs Uart_Ps;

volatile int TotalReceived_Count;
volatile int TotalSentCount;
//int TotalErrorCount;
extern u8 Received_Count;//END POINT

static u8 Received_index = 0;

u8 Recv_Buffer_temp[] = {0};

//void UartHandler(void *CallBackRef)
//{
////	XScuGic_Disconnect(&InterruptController, XPS_UART0_INT_ID);
//
//	if(XUartPs_IsReceiveData(XPAR_PS7_UART_0_BASEADDR))
//	{
//		XScuGic_Disable(&InterruptController, XPS_UART0_INT_ID);
//		if(Received_Count >= (TEST_BUFFER_SIZE-1))
//		{
//			Received_Count = 0;
//		}
//		Received_Count += XUartPs_Recv(&Uart_Ps, &Recv_Buffer[Received_Count],(TEST_BUFFER_SIZE - Received_Count));
//		XUartPs_WriteReg(XPAR_PS7_UART_0_BASEADDR, XUARTPS_CR_OFFSET,(u32)XUARTPS_CR_RXRST);
////	XScuGic_Connect(&InterruptController, XPS_UART0_INT_ID,
////			(Xil_ExceptionHandler) XScuGic_InterruptHandler, (void *)&Uart_Ps);
//	}
////	XUartPs_WriteReg(&Uart_Ps.Config.BaseAddress, XUARTPS_ISR_OFFSET, XUartPs_ReadReg(&Uart_Ps.Config.BaseAddress, XUARTPS_IMR_OFFSET));
//	Uart_Initialize(&InterruptController, &Uart_Ps, UART_DEVICE_ID, XPS_UART0_INT_ID);
//
//}


/*uart interupt function*/
void UartHandler(void *CallBackRef, u32 Event, unsigned int EventData)
{
	int i;
	/* All of the data has been sent */
	if (Event == XUARTPS_EVENT_SENT_DATA) {
		TotalSentCount = EventData;
	}

	/* All of the data has been received */
	if (Event == XUARTPS_EVENT_RECV_DATA) {

//		Received_Count += XUartPs_Recv(&Uart_Ps, &Recv_Buffer[Received_Count],(TEST_BUFFER_SIZE - Received_Count));
//		TotalReceived_Count = EventData;
//		Uart_Ps.ReceiveBuffer.NextBytePtr = NULL;
//		Uart_Ps.ReceiveBuffer.RemainingBytes = 0U;
//		Uart_Ps.ReceiveBuffer.RequestedBytes = 0U;
		XUartPs_Recv(&Uart_Ps, Recv_Buffer_temp,1);
		for(i=0; Recv_Buffer_temp[i]!='\0'; i++)
		{
			if(Received_index > (TEST_BUFFER_SIZE-1))
			{
				Received_index = 0;
			}
			Recv_Buffer[Received_index] = Recv_Buffer_temp[i];
			Received_index++;
		}
		Received_Count = Received_index - 1;
	}

}


