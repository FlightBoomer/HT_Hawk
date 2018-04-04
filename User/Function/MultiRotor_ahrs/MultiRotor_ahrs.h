#ifndef __MultiRotor_ahrs_H
#define	__MultiRotor_ahrs_H
#include "stm32f10x.h"
#include "include.h"

#define RtA 		57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426



extern int16_t MAG[3];			
//extern Gravity V;
void AHRS_getValues(void);
void AHRS_Geteuler(void);
#endif













