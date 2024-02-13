/*
 * UART.h
 *
 *  Created on: 2018Äê12ÔÂ30ÈÕ
 *      Author: Administrator
 */

#ifndef UART_H_
#define UART_H_

#include "xparameters.h"
#include "xplatform_info.h"
#include "xuartps.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xil_printf.h"
#include "xbasic_types.h"



#define EXTERNAL 1
#define INTERNAL 2
#define SWEEP    3
#define TTL      0

#define FLOAT2P  0
#define FLOAT3P  1
#define EXP5P    2

#define NEOS    0
#define UEOS    1
#define UTOK    2

#define RALL    0
#define SNAP    1
#define OUTP    2
#define TRCA    3
#define SENS    4
#define FMOD    5
#define PHAS    6
#define FREQ    7
#define RSLP    8
#define HARM    9
#define SLVL    10
#define ISRC    11
#define IGND    12
#define ICPL    13
#define ILIN    14
#define RMOD    15
#define OFLT    16
#define OFSL    17
#define SYNC    18
#define AGAN    19
#define ARSV    20
#define APHS    21
#define ASCL    22
#define FPOP    23
#define OEXP    24
#define SSET    25
#define RSET    26
#define SRAT    27
#define STRD    28
#define SPTS    29
#define REST    30
#define _RST    31
#define _IDN    32
#define SWPT    33
#define SLLM    34
#define SULM    35
#define SSLL    36
#define SSLG    37
#define STLM    38
#define SWRM    39
#define SWVT    40
#define SVLL    41
#define SVUL    42
#define SVSL    43
#define SVSG    44
#define SVTM    45
#define SVRM    46
#define SLEN    47
#define SSLE    48
#define STRG    49
#define SPRM    50
#define PAUS    51
#define OAUX    52
#define EQCD    53
#define EQCS    54
#define SPED    55
#define FRAM    56

#define INOV    57
#define GNOV	58
#define _PLL    59
#define STHR    60
#define FMUL    61

//*****************define for IA-7005 **********************//
#define SFMD    62
#define SFRE    63
#define SPHS    64
#define FUNC    65

/* define for freq sweep and amplitude sweep */
#define SWEEP_MAXLINE 1001

#define TEST_BUFFER_SIZE 256
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#define UART_DEVICE_ID XPAR_XUARTPS_0_DEVICE_ID
#define   FREQ_MAX (60e6)
#define Reg_15_Default 0x000000A7 //external, sync_filter: off, sineout: off, time coefficient: 300ms, filter slop: 24db, TTL referece

//#define   FPGA_NCO_CODE		 11930464.71111111111    // 2^32/360 nco ip core
//#define   FPGA_FREQ_CODE   109951.1627776    		   // 27487.7906944 * 4 = 109951.1627776  2^42/160e6 = 27487.7906944  "x4" is channel number
//#define   FPGA_FREQ_CODE   6871.9477  //2M5  //2^40/160000000 zcj

#define   FPGA_FREQ_COE   17.179869184  //clk=250M  FPGA_FREQ_CODE=(2^32-1)/250M	zdw
#define   FPGA_GAIN_COE   13107  //16bits DAC  FPGA_GAIN_COE=(2^16-1)/5	zdw
#define   FPGA_PHASE_COE   11930464.71  //32bits  FPGA_PHASE_COE=(2^32-1)/360	zdw
#define   SYNC_SAMPLE_CODE 244140.625  //250000000/1024 = 244140.625
#define   HIGH_LOW_FREQ_LIMIT 10000.0

#define   NULL_WORKSPACE 0
#define   DETERMINE_WORK_MODE 1
#define   AUTO_Rr  2
#define   BALANCED_ALGORITHM  3
#define   CAL_DUT  4
#define   CAL_Rr  5
#define   MEASURE_STRAY_C  6

#define   NONE_MEASURE_MODE  0
#define   SINGLE_MEASURE_MODE  1
#define   REPEAT_MEASURE_MODE  2
#define   FREQ_SWEEP_MEASURE_MODE  3
#define   AMP_SWEEP_MEASURE_MODE  4

#define   NONE_MEASRUE_TERM  0
#define   DUT_MEASRUE_TERM  1
#define   Rr_MEASRUE_TERM  2
#define   STRAY_C_MEASRUE_TERM  3
#define   SHORT_OPEN_MEASRUE_TERM  4

#endif /* UART_H_ */

void UART(void);
void UART_main(void);
void UART_Instrution_Cmp(u16 Command_index_start,u16 Command_index_end);
u8 String_Tokens_Separate(char *string, u8 Tokens_len, u8 token_cmd);
void Instrution_Parameter_Parse(void);
void Execute_Instrution(u16 cmd_flag,u8 cmd_opt_type, u8 inquire);
void QueryIntArg(u32 query_i_data, u8 EndChar);
void QueryFloatArg(Xfloat32 query_f_data, u8 param_type, u8 EndChar);
void Send_Data2PC(char data);
void Send_Data2PC_str(char *str);
void UartHandler(void *CallBackRef, u32 Event, unsigned int EventData);
void Uart_Initialize(XScuGic *IntcInstPtr, XUartPs *UartInstPtr,u16 DeviceId, u16 UartIntrId);
void delay_us(u32 timeus);
void Rr_Sel(u8 CTRL_Rr_Sel);
