/******************** (C) COPYRIGHT 2014 Air Nano Team ***************************
 * 文件名  ：led.c
 * 描述    ：led函数应用         
 * 实验平台：Air Nano四轴飞行器
 * 库版本  ：ST3.5.0
 * 作者    ：Air Nano Team 
 * 淘宝    ：http://byd2.taobao.com
**********************************************************************************/
#include "led.h"

/*
 * 函数名：LED_GPIO_Config
 * 描述  ：配置LED用到的I/O口
 * 输入  ：无
 * 输出  ：无
 */
void LED_GPIO_Config(void)
{		
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	//设置LED使用到得管脚
	
	
	/*开启GPIOB的外设时钟*/
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC, ENABLE); 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/*选择要控制的GPIOC引脚*/															   
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;	

	/*设置引脚模式为通用推挽输出*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;   

	/*设置引脚速率为50MHz */   
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 

	/*调用库函数，初始化GPIOB*/
  	GPIO_Init(GPIOC, &GPIO_InitStructure);		  

	/* 关闭所有led灯	*/
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
