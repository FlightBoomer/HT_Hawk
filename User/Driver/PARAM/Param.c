/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：PARAM.c
 * 描述    ：参数加载         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "include.h"

uint16_t VirtAddVarTab[NumbOfVar] = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 
																		 0xAA0A,0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E,};

static void EE_READ_ACC_OFFSET(void)
{
	EE_ReadVariable(VirtAddVarTab[0], &sensor.acc.quiet.x);
	EE_ReadVariable(VirtAddVarTab[1], &sensor.acc.quiet.y);
	EE_ReadVariable(VirtAddVarTab[2], &sensor.acc.quiet.z);
}

void EE_SAVE_ACC_OFFSET(void)
{
  EE_WriteVariable(VirtAddVarTab[0],sensor.acc.quiet.x);
  EE_WriteVariable(VirtAddVarTab[1],sensor.acc.quiet.y);
	EE_WriteVariable(VirtAddVarTab[2],sensor.acc.quiet.z);
}	


//**************************************************************************
//参数加载
//**************************************************************************
void	paramLoad(void)
{
	EE_READ_ACC_OFFSET(); //读取加速度零偏
	Gyro_OFFSET();        //采集陀螺仪零偏
	
	// The data of pitch
	ctrl.pitch.shell.kp = 9;
	ctrl.pitch.shell.ki = 0.04;
	ctrl.pitch.shell.kd = 3;
	
	ctrl.pitch.core.kp = 10;
	ctrl.pitch.core.kd = 3.5;
	
	//The data of roll
	ctrl.roll.shell.kp = 9;
	ctrl.roll.shell.ki = 0.04;
	ctrl.roll.shell.kd = 3;

	ctrl.roll.core.kp = 10;
	ctrl.roll.core.kd = 3.5;
	
	//The data of yaw
	ctrl.yaw.shell.kp = 15;
	ctrl.yaw.shell.kd = 0.04;
	
	ctrl.yaw.core.kp = 13;
	ctrl.yaw.core.kd = 4.5;
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 200;
	ctrl.roll.shell.increment_max = 200;
	
	ctrl.ctrlRate = 0;
	
	RC_Data.pitch_offset = 1900;
	RC_Data.roll_offset = 2000;
	RC_Data.yaw_offset = 2050;
}
