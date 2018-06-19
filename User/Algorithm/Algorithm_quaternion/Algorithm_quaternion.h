#ifndef __Algorithm_quaternion_H
#define	__Algorithm_quaternion_H

#include "stm32f10x.h"

typedef   struct {
  float Pitch;
  float Roll;
  float Yaw;
} EulerAngle;

typedef __IO struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef __IO struct {
  float x;
  float y;
  float z;
} Gravity;


extern EulerAngle AngE,IMU;

Gravity Quaternion_vectorGravity( Quaternion *pNumQ );
void Quaternion_ToNumQ( Quaternion *pNumQ, EulerAngle *pAngE );
void Quaternion_ToAngE( Quaternion *pNumQ, EulerAngle *pAngE );
Quaternion Quaternion_Multiply( Quaternion NowQ, Quaternion OldQ );
void Quaternion_Normalize( Quaternion *pNumQ );
void Quaternion_RungeKutta( Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes );
#endif /* __Algorithm_quaternion_H */
