/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��Ultrasonic.c
 * ����     ��������ģ������
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
/*******************************������ģ��Ĺ���ԭ��*******************************               

1������IO��TRIG������࣬������10us�ĸߵ�ƽ�źţ�
2��ģ���Զ�����8��40KHZ�ķ������Զ�����Ƿ����źŷ��أ�
3�����źŷ��أ�ͨ��IO��ECHO���һ���ߵ�ƽ���ߵ�ƽ������ʱ����ǳ������ӷ��䵽���ص�ʱ��;
4�����Ծ���=���ߵ�ƽʱ��*���٣�340M/s����/2��
5��������Ĭ�ϲ���US-100���������ģ��;
6��ע�ⳬ����ģ����Ч�߶�Ϊ2M�����޲�����3M;
7�����ʱ��������������������0.5ƽ������ƽ�澡��Ҫ��ƽ��������Ӱ������Ľ����
8�����߷�����VCC---5V
             ECHO--CH7��PB0��
             TRIG--CH8��PB1��
						 GND---GND

***********************************************************************************/
#include "include.h"

#define	ECHO_PORT      GPIOB		  //������ģ��ECHO�����ź�����˿�---�ɿ�CH7����PWM 
#define	ECHO_PIN       GPIO_Pin_0	//ECHO--CH7��PB0�� 

#define	TRIG_PORT      GPIOB		  //������ģ��TRIG�����ź�����˿�---�ɿ�CH8���PWM
#define	TRIG_PIN       GPIO_Pin_1 //TRIG--CH8��PB1��


float US100_Alt;
float US100_Alt_delta,US100_Alt_old;
unsigned int g_Hight=0,g_HightOld=0;
float g_Alt_Hight=0,g_Alt_HightOld=0;
float g_HightControl=0,g_HightControlold=0,hight_increment=0;
unsigned char RcvIndex,GLengthHigh, GLengthLow; 
float g_hight_Kp=0.8,g_hight_Ki=0.015,g_hight_Kd=10;  
float hight_error=0,hight_errorold=0,hight_erroroldd,cao;

/*====================================================================================================*/
/*====================================================================================================*
**���� : Ultrasonic_Config
**���� : ����������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;	       
  /* config the extiline(PB0) clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
    
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN;					    
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		//��Ϊ�������ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	         
  GPIO_Init(TRIG_PORT, &GPIO_InitStructure);	        //��ʼ������GPIO 	
	GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Ultrasonic_Pulsing
**���� : ����������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Ultrasonic_Pulsing(void)
{
  GPIO_SetBits(TRIG_PORT,TRIG_PIN);		  //��>10US�ĸߵ�ƽ
	delay_us(20);
  GPIO_ResetBits(TRIG_PORT,TRIG_PIN);
}

