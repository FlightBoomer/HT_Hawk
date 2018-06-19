#ifndef __HEIGHT_CTRL_H
#define __HEIGHT_CTRL_H

#include "stm32f10x.h"

typedef struct
{
	float err;
	float err_old;
	float err_d;
	float err_i;
	float pid_out;

}_st_height_pid_v;

typedef struct
{
	float kp;
	float kd;
	float ki;

}_st_height_pid;

void Height_Ctrl(float T,float thr);
void Ultra_PID_Init(void);
void Baro_PID_Init(void);
void WZ_Speed_PID_Init(void);
void height_speed_ctrl(float T,float thr,float exp_z_speed,float );
void Baro_Ctrl(float T,float thr);
void Ultra_Ctrl(float T,float thr);
void LockForKeepHigh(float THROTTLE);
void Ultra_dataporcess(float T);
void Baro_dataporcess(float T);

extern float ultra_ctrl_out;
extern float height_ctrl_out;
extern float ultra_speed,ultra_dis_lpf,baro_height,baro_speed,Pressure_groud;
extern _st_height_pid wz_speed_pid;
extern _st_height_pid ultra_pid;
extern _st_height_pid baro_pid;
extern _st_height_pid baro_wz_speed_pid,ultra_wz_speed_pid;
#endif

