/*
 * AD9954_SPI.c
 *
 *  Created on: 2019Äê10ÔÂ6ÈÕ
 *      Author: Administrator
 */


/*

	GPIO_54 = AD9552_RST	= U5;should be edit later
	GPIO_55 = AD9552_Locked	= V7;
	GPIO_60 = AD9552_CS_n	= U7;
	GPIO_61 = AD9552_IOS	= N16;
	
	GPIO_56 = AD9508_RST_n	= J15;
	GPIO_57 = AD9508_SYNC	= G14;
	GPIO_58 = AD9508_CS_n	= M14;
	GPIO_59 = AD9508_IOS	= N16;
	
	GPIO_62 = DDS2_IOS		= J16;
	GPIO_63 = DDS2_UPDATE	= K16;
	GPIO_64 = DDS2_SYNC_IN	= H15;
	GPIO_65 = DDS2_CS		= F17;
	GPIO_66 = DDS2_IO_SYNC	= F16;
	GPIO_67 = DDS2_RESET	= D18;
	GPIO_68 = DDS2_PWR_DWN	= E17;
	
	GPIO_69 = DDS3_IOS		= U9;
	GPIO_70 = DDS3_UPDATE	= V10;
	GPIO_71 = DDS3_SYNC_IN	= V11;
	GPIO_72 = DDS3_CS		= K14;
	GPIO_73 = DDS3_IO_SYNC	= J14;
	GPIO_74 = DDS3_RESET	= L14;
	GPIO_75 = DDS3_PWR_DWN	= L15;
	
	GPIO_76 = DDS1_IOS		= D19;
	GPIO_77 = DDS1_UPDATE	= B20;
	GPIO_78 = DDS1_SYNC_IN	= C20;
	GPIO_79 = DDS1_CS		= E19;
	GPIO_80 = DDS1_IO_SYNC	= E18;
	GPIO_81 = DDS1_RESET	= A20;
	GPIO_82 = DDS1_PWR_DWN	= B19;

*/

#include "xparameters.h"
#include "xspips.h"
#include "xil_printf.h"
#include "sleep.h"

#include "xgpiops.h"
#include "xscugic.h"
#include "xil_exception.h"

#include "math.h"

#include "UART.h"
#include "xuartps_hw.h"
#include "xuartps.h"
#include "ARM_interface.h"

#include <stdlib.h>
#include <stdio.h>

#define SpiPs_RecvByte(BaseAddress) \
		(u8)XSpiPs_In32((BaseAddress) + XSPIPS_RXD_OFFSET)

#define SpiPs_SendByte(BaseAddress, Data) \
		XSpiPs_Out32((BaseAddress) + XSPIPS_TXD_OFFSET, (Data))
		
	
//Function

int spi0_init();
int gpio_init();
void spi0_one_write();
void SpiRead(int ByteCount);
void SpiWrite(u8 *Sendbuffer, int ByteCount);
void AD9552_Write();
void AD9508_Write();
void DDS2_Write();
void DDS3_Write();
void DDS1_Write();

//from guoquan
void Balanced_algorithm(void);
void UART(void);
void Send_Data2PC_str(char *str);
extern void Cal_single(void);
extern void Read_data(void);
extern void determine_work_mode();
extern void Uart_Timer_intr_Initialize();
extern void Auto_Rr(void);
void DDS_Buffer_Initialize();

volatile u8 Send_Buffer[TEST_BUFFER_SIZE];	/* Buffer for Transmitting Data */
volatile u8 Recv_Buffer[TEST_BUFFER_SIZE];	/* Buffer for Receiving Data */
volatile u8 Received_Count = 0;//END POINT
//volatile u8 SendCount = 0;//END POINT


/*--------  for sweep  --------*/
extern u8 Balance_dft_delay_flag;
extern void Data_combine(void);
extern u8 Master_switch_flag;
extern u8 Cal_dft_delay_flag;

/*--------  for Auto_Rr  --------*/
extern u8 Auto_Rr_dft_delay_flag;

/*--------  for measure stray C  --------*/
extern u8 Measure_stray_Cb_delay_flag;
extern void Measure_Stray_C(void);

/*--------  for measure Rr  --------*/
extern void Cal_Rr_single(void);
extern u8 Cal_Rr_dft_delay_flag;

/* for data interact Time out */
volatile u32 Data_interact_ON_cnt = 0;
extern u8 Data_interact_ON_flag;
extern void Send_Data2PC_str(char *str);
volatile u8 Workspace = NULL_WORKSPACE;
//SPI
XSpiPs Spi0, Spi1;
unsigned char ReadBuffer[1024];
//unsigned char WriteBuffer[1024]={1,2,3,4,5,6,7,8,9,0};

//GPIO
XGpioPs psGpioInstancePtr;
XGpioPs_Config *GpioConfigPtr;

//UART
XScuGic InterruptController;
extern XUartPs Uart_Ps;

//Define the variable of the AD9954

unsigned char DDS1_CFR1[5]={0x00,0x02,0x00,0x20,0x00};	//Enable OSK function to adjust the amplitude
unsigned char DDS2_CFR1[5]={0x00,0x02,0x00,0x20,0x00};	//Enable OSK function to adjust the amplitude
unsigned char DDS3_CFR1[5]={0x00,0x02,0x00,0x20,0x00};	//Enable OSK function to adjust the amplitude{0x00,0x02,0x00,0x20,0x00};
//CFR1[3]=0x20; auto clear
//CFR1[3]=0x04; manual clear



//unsigned char CFR2[4]={0x01,0x00,0x08,0x04};	//Enalbe the hign speed characteristic, disable the REFFCLK multiplier
unsigned char CFR2[4]={0x01,0x00,0x08,0x04};



unsigned char DDS1_ASF[3]={0x02,0x3f,0xff};		//default half-scale
unsigned char DDS2_ASF[3]={0x02,0x3f,0xff};		//default half-scale
unsigned char DDS3_ASF[3]={0x02,0x3f,0xff};		//default half-scale


unsigned char DDS1_FTW0[5]={0x04,0x13,0x33,0x33,0x33};	//default 30M output from 400MHz system clock
unsigned char DDS2_FTW0[5]={0x04,0x13,0x33,0x33,0x33};	//default 30M output from 400MHz system clock
unsigned char DDS3_FTW0[5]={0x04,0x13,0x33,0x33,0x33};	//default 30M output from 400MHz system clock



unsigned char DDS1_POW0[3]={0x05,0x00,0x00};		//default deg=0
unsigned char DDS2_POW0[3]={0x05,0x00,0x00};		//default deg=0
unsigned char DDS3_POW0[3]={0x05,0x0c,0x04};		//default deg=0   7ff=45degree,fff=90degree,1fff=180degree




volatile u32 DDS1_DDS2_freq_tunning_word;
volatile u32 DDS3_freq_tunning_word;
volatile double DDS_output_gain = 1;
volatile double DDS3_output_gain = 1;


volatile u32 DDS1_amplitude_factor;
volatile u32 DDS2_amplitude_factor;
volatile u32 DDS3_amplitude_factor;


volatile u32 DDS1_phase_factor;
volatile u32 DDS2_phase_factor;
volatile u32 DDS3_phase_factor;

//Define the variable of the AD9508
int AD9508_DIVIDER;
unsigned char AD9508_OUT2_PD[3]={0x00,0x25,0x80};     //power down the OUT2 of AD9508
unsigned char AD9508_OUT1_PD[3]={0x00,0x1F,0x80};     //power down the OUT1 of AD9508
unsigned char AD9508_OUT3_PD[3]={0x00,0x2B,0x80};     //power down the OUT3 of AD9508
unsigned char AD9508_OUT3_ratio[4]={0x20,0x28,0x00,0x00};     //set the OUT3 divide ratio of AD9508, default no freq division
unsigned char AD9508_OUT2_ratio[4]={0x20,0x22,0x00,0x00};     //set the OUT2 divide ratio of AD9508, default no freq division
unsigned char AD9508_OUT0_ratio[4]={0x20,0x16,0x00,0x00};     //set the OUT2 divide ratio of AD9508, default no freq division

//Define the variable of the AD9552
double AD9552_Freq_out;
unsigned char AD9552_INI[3]={0x00,0x00,0x38};     	//initial all registers to default
unsigned char AD9552_IO_UPDATE[3]={0x00,0x05,0x01};	//To transfer the data from IO buffer to core control buffer
unsigned char AD9552_VCO_enable[3]={0x00,0x0E,0x74};	//TO enable the SPI to control VCO calibration (maybe no need)
unsigned char AD9552_VCO_ini[3]={0x00,0x0E,0xF4};	//TO initial the VCO
unsigned char AD9552_N[3]={0x00,0x11,0xAD}; 	//N factor(default 5MHz)
unsigned char AD9552_MOD1[3]={0x00,0x12,0xFF}; //MOD factor [19:12]
unsigned char AD9552_MOD2[3]={0x00,0x13,0xFF}; //MOD factor [11:4]
unsigned char AD9552_MOD3[3]={0x00,0x14,0xC8}; //MOD factor [3:0] and spi output enable and SDM enable
unsigned char AD9552_FRAC1[3]={0x00,0x15,0x3F}; //FRAC factor[19:12]
unsigned char AD9552_FRAC2[3]={0x00,0x16,0xFF}; //FRAC factor[11:4]
unsigned char AD9552_FRAC3[3]={0x00,0x17,0xF1}; //FRAC factor [3:0] and P1[5]
unsigned char AD9552_P[3]={0x00,0x18,0xFF};	//P1[4:0] and P0[2:0]
unsigned char AD9552_E_SPI[3]={0x00,0x19,0x80};//spi OUT1 enable
unsigned char AD9552_Out2_Mode[3]={0x00,0x34,0xA1};	//Set the ch2 outpput mode is LVDS
unsigned char AD9552_Out1_Mode[3]={0x00,0x32,0xA1};	//Set the ch1 outpput mode is LVDS


extern u32 IA_CTRL_Analog;
extern Xfloat32 Vx_Vr_freq;





extern double debug_flag;
volatile unsigned long long debug_data1,debug_data2,debug_data3,debug_data4;



//define the operation of each GPIO
#define	AD9552_RST_valid	XGpioPs_WritePin(&psGpioInstancePtr, 54, 1)
#define	AD9552_RST_invalid 	XGpioPs_WritePin(&psGpioInstancePtr, 54, 0)

#define	AD9508_RST_valid	XGpioPs_WritePin(&psGpioInstancePtr, 56, 0)
#define	AD9508_RST_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 56, 1)

#define	AD9508_SYNC_valid	XGpioPs_WritePin(&psGpioInstancePtr, 57, 1)
#define	AD9508_SYNC_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 57, 0)

#define	AD9508_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 58, 0)
#define	AD9508_CS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 58, 1)

#define	AD9508_IOS_valid	XGpioPs_WritePin(&psGpioInstancePtr, 59, 1)
#define	AD9508_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 59, 0)

#define	AD9552_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 60, 0)
#define	AD9552_CS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 60, 1)

#define	AD9552_IOS_valid	XGpioPs_WritePin(&psGpioInstancePtr, 61, 1)
#define	AD9552_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 61, 0)

#define	DDS2_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 65, 0)
#define	DDS2_CS_invalid		XGpioPs_WritePin(&psGpioInstancePtr, 65, 1)

#define	DDS2_UPDATE_valid	XGpioPs_WritePin(&psGpioInstancePtr, 63, 0)
#define	DDS2_UPDATE_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 63, 1)

#define	DDS2_IOS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 62, 1)
#define	DDS2_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 62, 0)

#define	DDS2_RST_valid		XGpioPs_WritePin(&psGpioInstancePtr, 67, 1)
#define	DDS2_RST_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 67, 0)


#define	DDS3_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 72, 0)
#define	DDS3_CS_invalid		XGpioPs_WritePin(&psGpioInstancePtr, 72, 1)


#define	DDS3_IOS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 69, 1)
#define	DDS3_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 69, 0)

#define	DDS3_RST_valid		XGpioPs_WritePin(&psGpioInstancePtr, 74, 1)
#define	DDS3_RST_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 74, 0)

#define	DDS1_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 79, 0)
#define	DDS1_CS_invalid		XGpioPs_WritePin(&psGpioInstancePtr, 79, 1)


#define	DDS1_IOS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 76, 1)
#define	DDS1_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 76, 0)

#define	DDS1_RST_valid		XGpioPs_WritePin(&psGpioInstancePtr, 81, 1)
#define	DDS1_RST_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 81, 0)


#define	DDS_PWR_DOWN_valid		XGpioPs_WritePin(&psGpioInstancePtr,83,1)
#define	DDS_PWR_DOWN_invalid	XGpioPs_WritePin(&psGpioInstancePtr,83,0)



//power board add by jst
#define IIC_SLAVE_ADDR_CUR		0x58//AD5272 slave address in high 7 bits, the last bit 0 means write
#define IIC_SLAVE_ADDR_VOL		0x5c//AD5272 slave address in high 7 bits, the last bit 0 means write
#define CUR_EN		88       //gpio34
#define VOL_EN		87       //gpio33
#define IIC_SDA		84       //gpio16
#define IIC_SCL		83       //gpio28
#define	POWER_CS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 86, 0)
#define	POWER_CS_invalid		XGpioPs_WritePin(&psGpioInstancePtr, 86, 1)


#define	POWER_IOS_valid		XGpioPs_WritePin(&psGpioInstancePtr, 85, 1)
#define	POWER_IOS_invalid	XGpioPs_WritePin(&psGpioInstancePtr, 85, 0)
#define AD5761R_SPI_SELECT		0x00
void CUR_ON(void);
void CUR_OFF(void);
void VOL_ON(void);
void VOL_OFF(void);
void IIC_SDA_HIGH(void);
void IIC_SDA_LOW(void);
void IIC_SCL_HIGH(void);
void IIC_SCL_LOW(void);
void IIC_ON(void);
void IIC_OFF(void);
void IIC_Transmit(char data);
void AD5272_Write(char data_high, char data_low, char address);
void POWER_SPI(void);
int spi0_power_init();
int SpiPsPolledExample(XSpiPs *SpiInstancePtr, u16 SpiDeviceId);

u8 DAC_InitBuffer[3]={0x04,0x02,0x3b};
u8 DAC_WriteBuffer[3]={0x03,0x00,0x02};

XSpiPs SpiInstance;
#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID



int i;
int main(void) {
	int Status;
	int xStatus;
	int count;
    Uart_Timer_intr_Initialize();
    DDS_Buffer_Initialize();

	Uart_Initialize(&InterruptController, &Uart_Ps, UART_DEVICE_ID, XPS_UART0_INT_ID);	//when not used uart remember comment it, otherwise can't run without re-powerup

	xil_printf("SPI Selftest Example \r\n");




	xStatus = gpio_init();
	if (xStatus != XST_SUCCESS) {
		xil_printf("GPIO Selftest Example Failed\r\n");
		return XST_FAILURE;
	}


	//power board
	CUR_OFF();
	VOL_OFF();
	AD5272_Write(0x04, 0x54, IIC_SLAVE_ADDR_CUR);
	//AD5272_Write(0x04, 0x81, IIC_SLAVE_ADDR_VOL);
	AD5272_Write(0x05, 0xc9, IIC_SLAVE_ADDR_VOL);
	Status = SpiPsPolledExample(&SpiInstance,SPI_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		xil_printf("SPI Serial Polled Example Test Failed\r\n");
		return XST_FAILURE;
	}
	//CUR_ON();
	VOL_ON();





	Status = spi0_init();
	if (Status != XST_SUCCESS) {
		xil_printf("SPI Selftest Example Failed\r\n");
		return XST_FAILURE;
	}





	//configure the default clock path freq (MHz)
	AD9552_Freq_out =	400000000;	// configurate default output frequency of AD9552
	AD9508_DIVIDER 	=	0; 	// configurate default frequency divider of AD9508

	//wake up the chip
	AD9552_RST_invalid;
	AD9508_RST_invalid;
	AD9508_SYNC_valid;
	DDS2_RST_invalid;
	DDS3_RST_invalid;
	DDS1_RST_invalid;
	POWER_IOS_invalid;
	AD9552_Write();	//initial the AD9552
	usleep(10);
	AD9508_Write();	//initial the AD9508
	usleep(10);

	//1kHz¡¢5Vpp,4s
	Vx_Vr_freq =	0;	// configurate default output frequency of AD9954   100kHz   edit by jst
	DDS3_freq_tunning_word = (Vx_Vr_freq/AD9552_Freq_out*(AD9508_DIVIDER+1))*pow(2,32);
	DDS3_FTW0[4] = DDS3_freq_tunning_word;
	DDS3_FTW0[3] = DDS3_freq_tunning_word>>8;
	DDS3_FTW0[2] = DDS3_freq_tunning_word>>16;
	DDS3_FTW0[1] = DDS3_freq_tunning_word>>24;
	DDS3_phase_factor=45*45.5083;
	DDS3_POW0[2] =DDS3_phase_factor;
	DDS3_POW0[1] =DDS3_phase_factor>>8;
	DDS3_Write();
	usleep(10);
	DDS2_Write();
	usleep(10);






	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(2));
	usleep(10);
	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));

	while(1)
	{
	UART();


    if (Master_switch_flag && !Data_interact_ON_flag) {
        switch (Workspace) {
            case DETERMINE_WORK_MODE:
                determine_work_mode();
                break;
            case AUTO_Rr:
                if (Auto_Rr_dft_delay_flag)
                {
                    Read_data();
                    Data_combine();
                    Auto_Rr();
                }
                break;
            case BALANCED_ALGORITHM:
                Balanced_algorithm();
                break;
            case CAL_DUT:
                if(Cal_dft_delay_flag)
                {

                    Read_data();
                    Data_combine();
                    Cal_single();
                }
                break;
            case CAL_Rr:
                if(Cal_Rr_dft_delay_flag)
                {
                    Read_data();
                    Data_combine();
                    Cal_Rr_single();
                }
                break;
            case MEASURE_STRAY_C:
                if (Measure_stray_Cb_delay_flag)
                {
                    Read_data();
                    Data_combine();
                    Measure_Stray_C();
                }
                break;
            default:
                determine_work_mode();
                break;
        }
    }
    //Shake hands with interaction
    if (Data_interact_ON_flag){
        Data_interact_ON_cnt++;
        if (Data_interact_ON_cnt >= 5000000){//50000000 is for 10s
            Send_Data2PC_str("FLAG;");
            Data_interact_ON_flag = 1;
            Data_interact_ON_cnt = 0;
        }
    }
}



	return XST_SUCCESS;
}

void SpiRead(int ByteCount)
{
	int Count;
	u32 StatusReg;

	StatusReg = XSpiPs_ReadReg(Spi0.Config.BaseAddress,
					XSPIPS_SR_OFFSET);

	/*
	 * Polling the Rx Buffer for Data
	 */
	do{
		StatusReg = XSpiPs_ReadReg(Spi0.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	}while(!(StatusReg & XSPIPS_IXR_RXNEMPTY_MASK));

	/*
	 * Reading the Rx Buffer
	 */
	for(Count = 0; Count < ByteCount; Count++){
		ReadBuffer[Count] = SpiPs_RecvByte(
				Spi0.Config.BaseAddress);
	}

}

void SpiWrite(u8 *Sendbuffer, int ByteCount)
{
	u32 StatusReg;
	int TransCount = 0;

	StatusReg = XSpiPs_ReadReg(Spi0.Config.BaseAddress,
				XSPIPS_SR_OFFSET);

	while ((ByteCount > 0) &&
		(TransCount < XSPIPS_FIFO_DEPTH)) {
		SpiPs_SendByte(Spi0.Config.BaseAddress,
				*Sendbuffer);
		Sendbuffer++;
		++TransCount;
		ByteCount--;
	}

	/*
	 * Wait for the transfer to finish by polling Tx fifo status.
	 */
	do {
		StatusReg = XSpiPs_ReadReg(
				Spi0.Config.BaseAddress,
					XSPIPS_SR_OFFSET);
	} while ((StatusReg & XSPIPS_IXR_TXOW_MASK) == 0);

}


int gpio_init() {
	int xStatus;
	int pin_count;

	GpioConfigPtr = XGpioPs_LookupConfig(XPAR_XGPIOPS_0_DEVICE_ID);

	if(GpioConfigPtr == NULL)
		return XST_FAILURE;

	xStatus = XGpioPs_CfgInitialize(&psGpioInstancePtr,GpioConfigPtr,GpioConfigPtr->BaseAddr);
	if(xStatus != XST_SUCCESS)
		return XST_FAILURE;



/******	set the direction of each gpio	******/
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,54,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,55,0);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,56,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,57,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,58,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,59,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,60,1);
		XGpioPs_SetDirectionPin(&psGpioInstancePtr,61,1);
		for(pin_count=62;pin_count<94;pin_count++)
			XGpioPs_SetDirectionPin(&psGpioInstancePtr,pin_count,1);

/******	set the output ability of each gpio	******/
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,54,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,55,0);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,56,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,57,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,58,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,59,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,60,1);
		XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,61,1);
		for(pin_count=62;pin_count<94;pin_count++)
			XGpioPs_SetOutputEnablePin(&psGpioInstancePtr,pin_count,1);

/******	set the initial output of each gpio, default all io function disable	******/

		//clock path related
		XGpioPs_WritePin(&psGpioInstancePtr,54,1);	//AD9552 RESET
		XGpioPs_WritePin(&psGpioInstancePtr,56,0);	//AD9508 RESET
		XGpioPs_WritePin(&psGpioInstancePtr,57,0);	//AD9508 SYNC
		XGpioPs_WritePin(&psGpioInstancePtr,58,1);	//AD9508 CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,59,0);	//AD9508 IOS_VE2
		XGpioPs_WritePin(&psGpioInstancePtr,60,1);	//AD9552 CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,61,0);	//AD9552 CS_n

		//DDS2 related(U201)
		XGpioPs_WritePin(&psGpioInstancePtr,62,0);	//IOS_VE2
		XGpioPs_WritePin(&psGpioInstancePtr,63,1);	//IO_UPDATE
		XGpioPs_WritePin(&psGpioInstancePtr,64,0);	//SYNC_IN
		XGpioPs_WritePin(&psGpioInstancePtr,65,1);	//CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,66,0);	//IO_SYNC
		XGpioPs_WritePin(&psGpioInstancePtr,67,1);	//RESET
		//XGpioPs_WritePin(&psGpioInstancePtr,68,0);	//PWR_DWN (default not power down)

		//DDS3 related(U202)
		XGpioPs_WritePin(&psGpioInstancePtr,69,0);	//IOS_VE2
		XGpioPs_WritePin(&psGpioInstancePtr,70,1);	//IO_UPDATE
		XGpioPs_WritePin(&psGpioInstancePtr,71,0);	//SYNC_IN
		XGpioPs_WritePin(&psGpioInstancePtr,72,1);	//CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,73,0);	//IO_SYNC
		XGpioPs_WritePin(&psGpioInstancePtr,74,1);	//RESET
		XGpioPs_WritePin(&psGpioInstancePtr,75,0);	//PWR_DWN (default not power down)

		//DDS1 related(U102)
		XGpioPs_WritePin(&psGpioInstancePtr,76,0);	//IOS_VE2
		XGpioPs_WritePin(&psGpioInstancePtr,77,1);	//IO_UPDATE
		XGpioPs_WritePin(&psGpioInstancePtr,78,0);	//SYNC_IN
		XGpioPs_WritePin(&psGpioInstancePtr,79,1);	//CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,80,0);	//IO_SYNC
		XGpioPs_WritePin(&psGpioInstancePtr,81,1);	//RESET
		XGpioPs_WritePin(&psGpioInstancePtr,82,0);	//PWR_DWN (default not power down)

		//DDS4 related(U106)
		XGpioPs_WritePin(&psGpioInstancePtr,83,0);	//IOS_VE2
		XGpioPs_WritePin(&psGpioInstancePtr,84,1);	//IO_UPDATE
		XGpioPs_WritePin(&psGpioInstancePtr,85,0);	//SYNC_IN
		XGpioPs_WritePin(&psGpioInstancePtr,86,1);	//CS_n
		XGpioPs_WritePin(&psGpioInstancePtr,87,0);	//IO_SYNC
		XGpioPs_WritePin(&psGpioInstancePtr,88,1);	//RESET
		XGpioPs_WritePin(&psGpioInstancePtr,89,0);	//PWR_DWN (default not power down)

		//Spare IO related
		XGpioPs_WritePin(&psGpioInstancePtr,83,0);	//for DDS all power_down sync output
		XGpioPs_WritePin(&psGpioInstancePtr,91,1);
		XGpioPs_WritePin(&psGpioInstancePtr,92,1);
		XGpioPs_WritePin(&psGpioInstancePtr,93,1);

	xil_printf("gpio config finish\n");
	return XST_SUCCESS;
}


int spi0_init() {
	int Status;
	XSpiPs_Config *SpiConfig;

	/*
	 * Initialize the SPI device.
	 */
	SpiConfig = XSpiPs_LookupConfig(XPAR_XSPIPS_0_DEVICE_ID);
	if (NULL == SpiConfig) {
		return XST_FAILURE;
	}

	Status = XSpiPs_CfgInitialize(&Spi0, SpiConfig, SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build.
	 */
	Status = XSpiPs_SelfTest(&Spi0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	xil_printf("%s self test succ\r\n", __func__);

	Status = XSpiPs_SetOptions(&Spi0, (XSPIPS_MASTER_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION | XSPIPS_CLK_PHASE_1_OPTION));
	if (Status != XST_SUCCESS) {
		xil_printf("%s XSpiPs_SetOptions fail\n", __func__);
		return XST_FAILURE;
	}
	Status = XSpiPs_SetClkPrescaler(&Spi0, XSPIPS_CLK_PRESCALE_8);
	if (Status != XST_SUCCESS) {
		xil_printf("%s XSpiPs_SetClkPrescaler fail\n", __func__);
		return XST_FAILURE;
	}
	XSpiPs_Enable(&Spi0);
	xil_printf("spi 0 config finish\n");
	return XST_SUCCESS;

}


void AD9552_Write()	{

	int Freq_out;
	
	/***** enable the AD9552 SPI communication through the isolation chip, diasble others *****/
	AD9552_IOS_valid;
	AD9508_IOS_invalid;
	DDS2_IOS_invalid;
	DDS3_IOS_invalid;
	DDS1_IOS_invalid;
	POWER_IOS_invalid;

//	Freq_out = AD9552_Freq_out/1000000;
	Freq_out = 400;

	switch (Freq_out)
	{
		case 5:	//5MHZ
			AD9552_N[2]=0xAD;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xC8;
			AD9552_FRAC1[2]=0x3F;
			AD9552_FRAC2[2]=0xFF;
			AD9552_FRAC3[2]=0xF1;
			AD9552_P[2]=0xFF;
			break;
		case 10:	//10MHZ
			AD9552_N[2]=0xC6;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x01;
			AD9552_P[2]=0x27;
			break;
		case 20:	//20MHZ
			AD9552_N[2]=0xC6;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x00;
			AD9552_P[2]=0x97;
			break;
		case 50:	//50MHZ
			AD9552_N[2]=0xC0;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xE8;
			AD9552_FRAC1[2]=0x7F;
			AD9552_FRAC2[2]=0xFF;
			AD9552_FRAC3[2]=0xF0;
			AD9552_P[2]=0x3F;
			break;
		case 100:	//100MHZ
			AD9552_N[2]=0xA5;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x00;
			AD9552_P[2]=0x1F;
			break;
		case 125:	//125MHZ
			AD9552_N[2]=0xBB;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xE8;
			AD9552_FRAC1[2]=0x7F;
			AD9552_FRAC2[2]=0xFF;
			AD9552_FRAC3[2]=0xF0;
			AD9552_P[2]=0x1E;
			break;
		case 200:	//200MHZ
			AD9552_N[2]=0xC8;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x00;
			AD9552_P[2]=0x16;
			break;
		case 250:	//250MHZ
			AD9552_N[2]=0xC8;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x00;
			AD9552_P[2]=0x14;
			break;
		case 400:	//400MHZ
			AD9552_N[2]=0xC8;
			AD9552_MOD1[2]=0x3F;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xF8;
			AD9552_FRAC1[2]=0x00;
			AD9552_FRAC2[2]=0x00;
			AD9552_FRAC3[2]=0x00;
			AD9552_P[2]=0x0E;
			break;
		default:	//5MHZ
			AD9552_N[2]=0xAD;
			AD9552_MOD1[2]=0xFF;
			AD9552_MOD2[2]=0xFF;
			AD9552_MOD3[2]=0xC8;
			AD9552_FRAC1[2]=0x3F;
			AD9552_FRAC2[2]=0xFF;
			AD9552_FRAC3[2]=0xF1;
			AD9552_P[2]=0xFF;
			break;
	}


	AD9552_CS_valid;
	SpiWrite(AD9552_INI,3);
	AD9552_CS_invalid;
	
	AD9552_CS_valid;
	SpiWrite(AD9552_IO_UPDATE,3);		
	AD9552_CS_invalid;

 	AD9552_CS_valid;
 	SpiWrite(AD9552_Out1_Mode,3);
 	AD9552_CS_invalid;
 
 	AD9552_CS_valid;
 	SpiWrite(AD9552_Out2_Mode,3);
 	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_VCO_enable,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_IO_UPDATE,3);
	AD9552_CS_invalid; 

	sleep(1);  //The user should wait at least 3 ms for the VCO calibration routine to finish before programming the VCO.

	AD9552_CS_valid;
	SpiWrite(AD9552_VCO_ini,3);
	AD9552_CS_invalid; //calibrate VCO

	AD9552_CS_valid;
	SpiWrite(AD9552_N,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_MOD1,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_MOD2,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_MOD3,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_FRAC1,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_FRAC2,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_FRAC3,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_P,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_E_SPI,3);
	AD9552_CS_invalid;

	AD9552_CS_valid;
	SpiWrite(AD9552_IO_UPDATE,3);
	AD9552_CS_invalid;
	AD9552_IOS_invalid;
}


void AD9508_Write()	{

	/***** enable the AD9508 SPI communication through the isolation chip, diasble others *****/
	AD9552_IOS_invalid;
	AD9508_IOS_valid;
	DDS2_IOS_invalid;
	DDS3_IOS_invalid;
	DDS1_IOS_invalid;
	POWER_IOS_invalid;

	/***** configure the divide ratio of output *****/
	AD9508_OUT2_ratio[3] = AD9508_DIVIDER;	//take the low 8 bits of ratio
	AD9508_OUT2_ratio[2] = AD9508_DIVIDER>>8;	//take the low 8 bits of ratio
	AD9508_OUT0_ratio[3] = AD9508_DIVIDER;	//take the low 8 bits of ratio
	AD9508_OUT0_ratio[2] = AD9508_DIVIDER>>8;	//take the low 8 bits of ratio
	AD9508_OUT3_ratio[3] = AD9508_DIVIDER;	//take the low 8 bits of ratio
	AD9508_OUT3_ratio[2] = AD9508_DIVIDER>>8;	//take the low 8 bits of ratio

	AD9508_CS_valid;
//	SpiWrite(AD9508_OUT3_PD,3);
	SpiWrite(AD9508_OUT2_ratio,4);
	SpiWrite(AD9508_OUT0_ratio,4);
	SpiWrite(AD9508_OUT3_ratio,4);
	AD9508_CS_invalid;
	AD9508_IOS_invalid;

//	/***** rising edge to update the output driver, then pull low to hold static *****/
//	AD9508_SYNC_valid;
//	usleep(100);
//	AD9508_SYNC_invalid;
}


void DDS2_Write()	{

	/***** enable the DDS2(U201) SPI communication through the isolation chip, diasble others *****/
	AD9552_IOS_invalid;
	usleep(10);
	AD9508_IOS_invalid;
	usleep(10);
	DDS3_IOS_invalid;
	usleep(10);
	DDS1_IOS_invalid;

	POWER_IOS_invalid;
	usleep(10);
	DDS2_IOS_valid;
	usleep(10);


	DDS2_CS_valid;
	usleep(10);
	SpiWrite(DDS2_CFR1,5);
	usleep(10);
	DDS2_CS_invalid;
	usleep(10);

	DDS2_CS_valid;
	usleep(10);
	SpiWrite(CFR2,4);
	usleep(10);
	DDS2_CS_invalid;
	usleep(10);

	DDS2_CS_valid;
	usleep(10);
	SpiWrite(DDS2_FTW0,5);
	usleep(10);
	DDS2_CS_invalid;
	usleep(10);

	DDS2_CS_valid;
	usleep(10);
	SpiWrite(DDS2_POW0,3);
	usleep(10);
	DDS2_CS_invalid;
	usleep(10);

	DDS2_CS_valid;
	usleep(10);
	SpiWrite(DDS2_ASF,3);
	usleep(10);
	DDS2_CS_invalid;
	usleep(10);

	/***** rising edge to update the control register *****/
	DDS2_UPDATE_valid;
	usleep(10);//seem if set to 1us could not output signal from 2MHz or lower. beacause when system clock is 2MHz, that sync clock is 2ns, so the CS siganl will be invalid before IO UPADATE refresh
	DDS2_UPDATE_invalid;
	usleep(10);
	DDS2_IOS_invalid;
	usleep(10);
}

void DDS3_Write()	{

	/***** enable the DDS3(U202) SPI communication through the isolation chip, diasble others *****/
	AD9552_IOS_invalid;
	usleep(10);
	AD9508_IOS_invalid;
	usleep(10);
	DDS2_IOS_invalid;
	usleep(10);
	DDS3_IOS_valid;
	usleep(10);
	DDS1_IOS_invalid;
	usleep(10);
	POWER_IOS_invalid;

	DDS3_CS_valid;
	usleep(10);
	SpiWrite(DDS3_CFR1,5);
	usleep(10);
	DDS3_CS_invalid;
	usleep(10);

	DDS3_CS_valid;
	usleep(10);
	SpiWrite(DDS3_POW0,3);
	usleep(10);
	DDS3_CS_invalid;
	usleep(10);

	DDS3_CS_valid;
	usleep(10);
	SpiWrite(CFR2,4);
	usleep(10);
	DDS3_CS_invalid;
	usleep(10);

	DDS3_CS_valid;
	usleep(10);
	SpiWrite(DDS3_FTW0,5);
	usleep(10);
	DDS3_CS_invalid;


	DDS3_CS_valid;
	usleep(10);
	SpiWrite(DDS3_ASF,3);
	usleep(10);
	DDS3_CS_invalid;
	usleep(10);


	/***** rising edge to update the control register *****/
	DDS2_UPDATE_valid;
	usleep(500);//seem if set to 1us could not output signal from 2MHz or lower. beacause when system clock is 2MHz, that sync clock is 2ns, so the CS siganl will be invalid before IO UPADATE refresh
	DDS2_UPDATE_invalid;
	usleep(10);
	DDS3_IOS_invalid;
	usleep(10);
}


void DDS1_Write()	{

	/***** enable the DDS1(U102) SPI communication through the isolation chip, diasble others *****/
	AD9552_IOS_invalid;
	AD9508_IOS_invalid;
	DDS2_IOS_invalid;
	DDS3_IOS_invalid;
	POWER_IOS_invalid;
	DDS1_IOS_valid;

	DDS1_CS_valid;
	usleep(10);
	SpiWrite(DDS1_CFR1,5);
	usleep(10);
	DDS1_CS_invalid;
	usleep(10);

	DDS1_CS_valid;
	usleep(10);
	SpiWrite(CFR2,4);
	usleep(10);
	DDS1_CS_invalid;
	usleep(10);


	DDS1_CS_valid;
	usleep(10);
	SpiWrite(DDS1_FTW0,5);
	usleep(10);
	DDS1_CS_invalid;
	usleep(10);

	DDS1_CS_valid;
	usleep(10);
	SpiWrite(DDS1_POW0,3);
	usleep(10);
	DDS1_CS_invalid;
	usleep(10);

	DDS1_CS_valid;
	usleep(10);
	SpiWrite(DDS1_ASF,3);
	usleep(10);
	DDS1_CS_invalid;
	usleep(10);

	/***** rising edge to update the control register *****/
	DDS2_UPDATE_valid;
	usleep(10);
	usleep(10);//seem if set to 1us could not output signal from 2MHz or lower. beacause when system clock is 2MHz, that sync clock is 2ns, so the CS siganl will be invalid before IO UPADATE refresh
	DDS2_UPDATE_invalid;
	usleep(10);
	DDS1_IOS_invalid;
	usleep(10);
}

/*Initialize DDS,Send_Buffer and Recv_Buffer*/
void DDS_Buffer_Initialize()
{
	int Index;
	//initialize the DDS
	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG28_OFFSET), (u32)(1));

	//SET THE DDS PHASE
//	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG17_OFFSET), (u32)(90 * FPGA_PHASE_COE));	//Tony add for Vx
//	Xil_Out32((XPAR_ARM_INTERFACE_0_S00_AXI_BASEADDR) + (ARM_INTERFACE_S00_AXI_SLV_REG16_OFFSET), (u32)(90 * FPGA_PHASE_COE));	//Tony add for Vr

//	Xil_Out32((XPAR_LIA_60M_INTERFACE_0_S00_AXI_BASEADDR) + (LIA_60M_INTERFACE_S00_AXI_SLV_REG15_OFFSET), (u32)(Reg_15_Default));
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		Send_Buffer[Index] = 0;
		Recv_Buffer[Index] = 0;
	}
}



void CUR_ON(void)
{
	/* Set the GPIO output to be high. */
	XGpioPs_WritePin(&psGpioInstancePtr,CUR_EN,1);	//RESET
}

void CUR_OFF(void)
{
	/* Set the GPIO output to be low. */
	XGpioPs_WritePin(&psGpioInstancePtr, CUR_EN, 0x0);
}

void VOL_ON(void)
{
	/* Set the GPIO output to be low. */
	XGpioPs_WritePin(&psGpioInstancePtr, VOL_EN, 0x0);
}

void VOL_OFF(void)
{
	/* Set the GPIO output to be high. */
	XGpioPs_WritePin(&psGpioInstancePtr, VOL_EN, 0x1);
}

void IIC_SDA_HIGH(void)
{
	/* Set the GPIO output to be high. */
	XGpioPs_WritePin(&psGpioInstancePtr, IIC_SDA, 0x1);
}

void IIC_SDA_LOW(void)
{
	/* Set the GPIO output to be low. */
	XGpioPs_WritePin(&psGpioInstancePtr, IIC_SDA, 0x0);
}

void IIC_SCL_HIGH(void)
{
	/* Set the GPIO output to be high. */
	XGpioPs_WritePin(&psGpioInstancePtr, IIC_SCL, 0x1);
}

void IIC_SCL_LOW(void)
{
	/* Set the GPIO output to be low. */
	XGpioPs_WritePin(&psGpioInstancePtr, IIC_SCL, 0x0);
}

void IIC_ON(void)
{
	IIC_SDA_HIGH();
	IIC_SCL_HIGH();
	(void)usleep(5);
	IIC_SDA_LOW();
	(void)usleep(5);
	IIC_SCL_LOW();
	(void)usleep(5);
}

void IIC_OFF(void)
{
	IIC_SDA_LOW();
	(void)usleep(5);
	IIC_SCL_HIGH();
	(void)usleep(5);
	IIC_SDA_HIGH();
	(void)usleep(5);
}

void IIC_Transmit(char data)
{
    int bit_count;
    char temp;
    temp=data;
    XGpioPs_SetDirectionPin(&psGpioInstancePtr, IIC_SDA, 1);
    (void)usleep(30);
    for(bit_count=0; bit_count<8; bit_count++)
    {

    	(void)usleep(5);
        if((temp&0x80)==0x80)
        {
        	IIC_SDA_HIGH();
        }
        else
        {
        	IIC_SDA_LOW();
        }
        (void)usleep(10);
        IIC_SCL_HIGH();
        (void)usleep(15);
        IIC_SCL_LOW();
        temp = temp<<1;
    }

    XGpioPs_SetDirectionPin(&psGpioInstancePtr, IIC_SDA, 0);
    IIC_SDA_HIGH();
    (void)usleep(5);

    IIC_SCL_HIGH();
    (void)usleep(5);

    if(XGpioPs_ReadPin(&psGpioInstancePtr, IIC_SDA)==0)
    {
    IIC_SCL_LOW();
    XGpioPs_SetDirectionPin(&psGpioInstancePtr, IIC_SDA, 1);
    }
}

void AD5272_Write(char data_high, char data_low, char address)
{
    IIC_ON();
    IIC_Transmit(address);//ADDR of CUR/VOL chip
    IIC_Transmit(0x1c);//WRITE CONTENTS OF SERIAL REGISTER DATA TO CONTROL REGISTER£¬ENABLE PROGRAMMING£¬AD5272 CMD7
    IIC_Transmit(0x02);//SET CONTROL REGISTER bit C1 allow update of wiper position through a digital interface
    IIC_Transmit(data_high);
    IIC_Transmit(data_low);
    IIC_OFF();
    (void)usleep(1000);
}

void POWER_SPI()
{
	AD9552_IOS_invalid;
	usleep(10);
	AD9508_IOS_invalid;
	usleep(10);
	DDS3_IOS_invalid;
	usleep(10);
	DDS1_IOS_invalid;
	usleep(10);
	DDS2_IOS_invalid;
	usleep(10);
	POWER_IOS_valid;
	usleep(10);


	POWER_CS_valid;
	usleep(10);
	SpiWrite(DAC_InitBuffer,3);
	usleep(10);
	POWER_CS_invalid;
	usleep(10);

	POWER_CS_valid;
	usleep(10);
	SpiWrite(DAC_WriteBuffer,3);
	usleep(10);
	POWER_CS_invalid;
	usleep(10);


	POWER_IOS_invalid;
	usleep(10);
}

int spi0_power_init() {
	int Status;
	XSpiPs_Config *SpiConfig;

	/*
	 * Initialize the SPI device.
	 */
	SpiConfig = XSpiPs_LookupConfig(XPAR_XSPIPS_0_DEVICE_ID);
	if (NULL == SpiConfig) {
		return XST_FAILURE;
	}

	Status = XSpiPs_CfgInitialize(&Spi0, SpiConfig, SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build.
	 */
	Status = XSpiPs_SelfTest(&Spi0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}
	xil_printf("%s self test succ\r\n", __func__);

	Status = XSpiPs_SetOptions(&Spi0, (XSPIPS_MASTER_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION | XSPIPS_CLK_PHASE_1_OPTION));
	if (Status != XST_SUCCESS) {
		xil_printf("%s XSpiPs_SetOptions fail\n", __func__);
		return XST_FAILURE;
	}
	Status = XSpiPs_SetClkPrescaler(&Spi0, XSPIPS_CLK_PRESCALE_128);
	if (Status != XST_SUCCESS) {
		xil_printf("%s XSpiPs_SetClkPrescaler fail\n", __func__);
		return XST_FAILURE;
	}
	XSpiPs_Enable(&Spi0);
	xil_printf("spi 0 config finish\n");
	return XST_SUCCESS;

}

int SpiPsPolledExample(XSpiPs *SpiInstancePtr,
			 u16 SpiDeviceId)
{
	int Status;
	u32 options = 0;
	u32 ChipSelect = AD5761R_SPI_SELECT;
	XSpiPs_Config *SpiConfig;

	if (XGetPlatform_Info() == XPLAT_ZYNQ_ULTRA_MP) {
		//MaxSize = 1024 * 10;
		ChipSelect = AD5761R_SPI_SELECT;	/* Device is on CS 0 */
	}

	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	if (NULL == SpiConfig) {
		return XST_FAILURE;
	}

	Status = XSpiPs_CfgInitialize(SpiInstancePtr, SpiConfig,
					SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	Status = XSpiPs_SelfTest(SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Set the SPI device as a master with manual start and manual
	 * chip select mode options
	 */
	XSpiPs_SetOptions(SpiInstancePtr, XSPIPS_MANUAL_START_OPTION | \
			XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION);

	/*
	 * Set the SPI device pre-scalar to divide by 8
	 */
	XSpiPs_SetClkPrescaler(SpiInstancePtr, XSPIPS_CLK_PRESCALE_128);

	/*
	 * Set the chip select
	 */
	XSpiPs_SetSlaveSelect(SpiInstancePtr, ChipSelect);

	options = XSpiPs_GetOptions(SpiInstancePtr);

	AD9552_IOS_invalid;
	usleep(10);
	AD9508_IOS_invalid;
	usleep(10);
	DDS3_IOS_invalid;
	usleep(10);
	DDS1_IOS_invalid;
	usleep(10);
	DDS2_IOS_invalid;
	usleep(10);
	POWER_IOS_valid;
	usleep(10);
	POWER_CS_valid;
	/*
	 * Write the init data
	 */
	XSpiPs_PolledTransfer(SpiInstancePtr, DAC_InitBuffer, NULL, 3);
	POWER_CS_invalid;

	/*
	 * Write the data in the write buffer
	 */
	POWER_CS_valid;
	XSpiPs_PolledTransfer(SpiInstancePtr, DAC_WriteBuffer, NULL, 3);
	POWER_CS_invalid;
	usleep(10);
	POWER_IOS_invalid;
	usleep(10);
	options = XSpiPs_GetOptions(SpiInstancePtr);


	return XST_SUCCESS;
}
