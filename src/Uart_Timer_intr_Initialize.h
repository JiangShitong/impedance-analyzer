#include<xscutimer.h>

static void TimerIntrHandler(void *CallBackRef);
void Uart_Timer_intr_Initialize();
void Timer_start(XScuTimer *TimerPtr);
void Timer_stop(XScuTimer *TimerPtr);
