/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MultiRotor_altitute.c
 * 描述     ：气压计数据采集与处理
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/

#include "board_config.h"
#include "MultiRotor_altitute.h"
#include "ms5611.h"
u8 timetoconver;
u8 keephigh;
u8  Ultrasonic_OK,Acc_OK,Pressure_OK;

void Altitute_calculate(void)
{
 if(timetoconver==1)
 {
     timetoconver=0;
		 Get_High();//间隔10ms执行一次气压、温度轮询,更新气压
 }
 
 if(flag.Loop_27Hz == 1)
 {
	   flag.Loop_27Hz =0;
		 Ultrasonic_Pulsing();//在这里每隔50ms触发一次超声波测高	
 }
}

/***********************************************
  * @brief  用三角函数的泰勒展开式
  * @param  None
  * @retval None
************************************************/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}

