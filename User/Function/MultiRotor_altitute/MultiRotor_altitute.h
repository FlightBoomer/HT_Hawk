#ifndef __MultiRotor_altitute_H
#define	__MultiRotor_altitute_H
#include "stm32f10x.h"
#include "include.h"

//#define RtA 		57.324841				
//#define AtR    	0.0174533				
//#define Acc_G 	0.0011963				
//#define Gyro_G 	0.03051756				
//#define Gyro_Gr	0.0005426

#define ACCDeadband 0.05

extern u8  Ultrasonic_OK,Acc_OK,Pressure_OK;
extern void Get_High(void);			
extern uint32_t Pressure;
extern float Keephigh_Out;
extern double ACChighout;
extern double SpeedZ,acc_now;
extern float g_HightPwm;


void Altitute_calculate(void);

//void AHRS_getValues(void);
//void AHRS_Geteuler(void);
#endif













