#ifndef __SPI_H
#define __SPI_H
#include "include.h"


#define SPI_CE_H()   GPIO_SetBits(GPIOD, GPIO_Pin_3) 
#define SPI_CE_L()   GPIO_ResetBits(GPIOD, GPIO_Pin_3)

#define SPI_CSN_H()  GPIO_SetBits(GPIOA, GPIO_Pin_15)
#define SPI_CSN_L()  GPIO_ResetBits(GPIOA, GPIO_Pin_15)

void SPI3_Init(void);			 //初始化SPI口
void SPI3_Config(void);
void SPI3_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI3_ReadWriteByte(u8 TxData);
	 
#endif


