#ifndef _MOTO_H_
#define _MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 1000


void PWM_OUT_Config(void);
void moto_STOP(void);
void moto_PwmRflash(u16 *Moter);
#endif
