/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��led.c
 * ����     ��led����
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "board_config.h"

led_Fsm LED;
LEDBuf_t LEDBuf;

/*
 * ��������LED_GPIO_Config
 * ����  ������LED�õ���I/O��
 * ����  ����
 * ���  ����
 */
 
void LED_GPIO_Config(void)
{		
	/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*����GPIOB������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_GPIO_LED, ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE); 
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//����LEDʹ�õ��ùܽ�
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  GPIO_InitStructure.GPIO_Pin = LED_R | LED_G | LED_B;	
	/*��������ģʽΪͨ���������*/
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   
	/*������������Ϊ50MHz */   
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	/*���ÿ⺯������ʼ��GPIOB*/
  GPIO_Init(GPIO_LED, &GPIO_InitStructure);		  
	/* �ر�����led��	*/
	GPIO_SetBits(GPIO_LED, LED_R | LED_G | LED_B);	
	
	GPIO_InitStructure.GPIO_Pin = LED_BLUE_1 | LED_BLUE_2 | LED_BLUE_4;	//��Ӱ��ϵ�LED��
  GPIO_Init(GPIOE_LED, &GPIO_InitStructure);	
  GPIO_SetBits(GPIOE_LED, LED_BLUE_1 | LED_BLUE_2 | LED_BLUE_4);	
	
	GPIO_InitStructure.GPIO_Pin = LED_BLUE_3;	//��Ӱ��ϵ�LED��
  GPIO_Init(GPIOB, &GPIO_InitStructure);	
  GPIO_SetBits(GPIOB, LED_BLUE_3);	
}

void LED_SHOW(void)
{
   LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
	 delay(900);
	 LED_ALLOFF();
	 delay(16000);
	 LED_ALLON();
}

void LED_BLUE_SHOW(void)
{
	for(u8 i=0;i<5;i++)
	{
		 LED_BLUE1_ON;LED_BLUE2_ON;LED_BLUE3_ON;LED_BLUE4_ON;
		 delay(200);
		 LED_BLUE1_OFF;LED_BLUE2_OFF;LED_BLUE3_OFF;LED_BLUE4_OFF;
		 delay(1000);
	}
	for(u8 j=0;j<2;j++)
	{
		 delay(1500);
		 LED_BLUE1_ON;
		 delay(1500);
		 LED_BLUE1_OFF;
		 delay(300);
		 LED_BLUE2_ON;
		 delay(1500);
		 LED_BLUE2_OFF;
		 delay(300);
		 LED_BLUE4_ON;
		 delay(1500);
		 LED_BLUE4_OFF;
		 delay(300);
		 LED_BLUE3_ON;
		 delay(1500);
		 LED_BLUE3_OFF;
		 delay(1500);
		 LED_BLUE3_ON;
		 delay(1500);
		 LED_BLUE3_OFF;
		 delay(300);
		 LED_BLUE4_ON;
		 delay(1500);
		 LED_BLUE4_OFF;
		 delay(300);
		 LED_BLUE2_ON;
		 delay(1500);
		 LED_BLUE2_OFF;
		 delay(300);
		 LED_BLUE1_ON;
		 delay(1500);
		 LED_BLUE1_OFF;
 }
}

void LEDReflash(void)
{
	if(LEDBuf.bits.R)
		Ledr_on;
	else
		Ledr_off;

	if(LEDBuf.bits.G)
		Ledg_on;
	else
		Ledg_off;

	if(LEDBuf.bits.B)
		Ledb_on;
	else
		Ledb_off;
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : LED_Fsm
**���� : LED״̬��
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void LED_Fsm(void)
{
 	switch(LED.event)
 	{
	  case Ht_ARMED:       
			if(++LED.cnt >= 300)  
				LED.cnt=0;
			if(LED.cnt<10 || (LED.cnt>40 && LED.cnt<50)) 
				LEDBuf.byte =LG|LB;
			else
				LEDBuf.byte =0;
		break;
			
		case Ht_DISARMED:       
			if(++LED.cnt >= 200)
				LED.cnt=0;
		  if(LED.cnt<=100)
				LEDBuf.byte =LB;
			else
			  LEDBuf.byte =0;
		break;	
			
		case Ht_CALIBRATA:           
        LEDBuf.byte =LB;
	  break;
		
 		case Ht_CALIBRATM_X:         
        LEDBuf.byte =LR;
 		break;
		
		case Ht_CALIBRATM_Y:         
        LEDBuf.byte =LG;
 		break;
		
		case Ht_CALIBRATM_Z:         
        LEDBuf.byte =LB;
 		break;
		
		case Ht_CALIBRATG:         
			  LEDBuf.byte =LR;
		break;
 	}
	LEDReflash();
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Hto_LED_Reflash
**���� : LED״̬��
**���� :  
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void FailSafeLEDAlarm(void)
{
	if(flag.ARMED)
		LED.event=Ht_ARMED;//������
	
	if(!flag.ARMED)
		LED.event=Ht_DISARMED;//û����
	
	if(flag.calibratingA)
		LED.event=Ht_CALIBRATA;
	
//	if(flag.calibratingG)		
// 		LED.event=Ht_CALIBRATG;
	
	if(flag.calibratingM){
		switch(flag.calibratingM)
 	  {
			case 1:LED.event=Ht_CALIBRATM_X; break;
			case 2:LED.event=Ht_CALIBRATM_Y; break;
			case 3:LED.event=Ht_CALIBRATM_Z; break;
		}
	}
	LED_Fsm();
}

