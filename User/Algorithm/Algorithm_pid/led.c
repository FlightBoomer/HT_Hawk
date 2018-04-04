/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * �ļ���  ��led.c
 * ����    ��led����Ӧ��         
 * ʵ��ƽ̨��Air Nano���������
 * ��汾  ��ST3.5.0
 * ����    ��Air Nano Team 
 * �Ա�    ��http://byd2.taobao.com
**********************************************************************************/
#include "led.h"

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

	//����LEDʹ�õ��ùܽ�
	
	
	/*����GPIOB������ʱ��*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*ѡ��Ҫ���Ƶ�GPIOC����*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;	

	/*��������ģʽΪͨ���������*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*������������Ϊ50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*���ÿ⺯������ʼ��GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		  

	/* �ر�����led��	*/
	GPIO_SetBits(GPIOC, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8);	 
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

}

void LED_Running(int tim)
{
   static u16 c=1,q=0;
   switch(c)
   {
    case 1:  LED8(OFF);LED1(ON);  break; 
	  case 2:	 LED1(OFF);LED2(ON);  break;
		case 3:  LED2(OFF);LED3(ON);  break; 
	  case 4:	 LED3(OFF);LED4(ON);  break;
		case 5:  LED4(OFF);LED5(ON);  break; 
	  case 6:	 LED5(OFF);LED6(ON);  break;
		case 7:  LED6(OFF);LED7(ON);  break; 
	  case 8:	 LED7(OFF);LED8(ON);  break;
	default: break;
   }
	 q++;
	 if(q >= tim)
	 {
		 c++;
	   q=0;
	 }
   if(c>=9) c=1;
}
void LED_Sailing(int rate)
{
	static u16 r,g;
	switch(r)
  {
		case 1:  LED_ALLON();  break;
		case 2:  LED_ALLOFF(); break;
		case 3:  LED_ALLON();  break;
		case 4:  LED_ALLOFF(); break;
		default: LED_ALLOFF(); break;
	}
	g++;
	if(g > rate)
	{ 
	  g=0;
		r++;
	}
	if(r > 15)	 r = 0;	
}


/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
