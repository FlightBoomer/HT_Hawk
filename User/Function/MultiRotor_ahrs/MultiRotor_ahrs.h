#ifndef __MultiRotor_ahrs_H
#define	__MultiRotor_ahrs_H
#include "stm32f10x.h"
#include "include.h"
#include <ekf.h>


#define RtA 	57.324841				
#define AtR    	0.0174533				
#define Acc_G 	0.0011963				
#define Gyro_G 	0.03051756				
#define Gyro_Gr	0.0005426



extern int16_t MAG[3];			
//extern Gravity V;
void AHRS_getValues(void);
void AHRS_Geteuler(void);

// QKF算法相关函数
void AHRS_init(void);
void AHRS_get_Euler_QKF(void);

extern float roll_qkf, pitch_qkf, yaw_qkf;

#endif













