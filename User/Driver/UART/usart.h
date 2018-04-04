#ifndef __USART_H_
#define __USART_H_
#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
uint8_t UART1_Put_Char(unsigned char DataToSend);
int fputc(int ch, FILE *f);
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void PC_Debug_Show(u8 num,u16 sta);

void usart2_config(void);
uint8_t UART2_Put_Char(unsigned char DataToSend);

void USART3_Config(void);
uint8_t UART3_Put_Char(unsigned char DataToSend);

void DMA_USART1_Configuration(void);
#endif
