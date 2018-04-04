#ifndef __DELAY_H
#define __DELAY_H 			   
#include "stm32f10x.h"

#define micros() TIM5->CNT


void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay(u32 x);
void Initial_System_Timer(void);
#endif

//------------------End of File----------------------------
