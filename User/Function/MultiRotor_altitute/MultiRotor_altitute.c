/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��MultiRotor_altitute.c
 * ����     ����ѹ�����ݲɼ��봦��
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
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
		 Get_High();//���10msִ��һ����ѹ���¶���ѯ,������ѹ
 }
 
 if(flag.Loop_27Hz == 1)
 {
	   flag.Loop_27Hz =0;
		 Ultrasonic_Pulsing();//������ÿ��50ms����һ�γ��������	
 }
}

/***********************************************
  * @brief  �����Ǻ�����̩��չ��ʽ
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

