#ifndef __MultiRotor_radio_H
#define	__MultiRotor_radio_H
#include "stm32f10x.h"
#include "include.h"

void Float2Byte(float *target,unsigned char *buf,unsigned char beg);
unsigned char HtoEs_Version_Data_Generate(void);
unsigned char HtoEs_Chart_Data_Generate(void);
unsigned char HtoEs_GPS_Data_Generate(void);
unsigned char HtoEs_Attitude_Data_Generate(void);
unsigned char HtoEs_HUD_Data_Generate(void);
unsigned char HtoEs_RC_Data_Generate(void);
unsigned char HtoEs_MOTO_Data_Generate(void);
unsigned char HtoEs_PID_Data_Generate(void);
void HT_GCS_Link(void);
void UsbCmdPro(void);
void usart_data_RX(void);
void Usart1_Send(unsigned char *DataToSend ,u8 data_num);
extern u8 NRF24L01_Txframes(u8 *txbuf,u8 packlen);
extern unsigned int CHK_SUM;
extern u8 Flag_Uart_Send;
extern u8 Flag_Request_Data;
#endif













