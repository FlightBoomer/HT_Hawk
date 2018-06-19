/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：moto.c
 * 描述     ：moto电机驱动配置
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "moto.h"
#include "board_config.h"

void Tim1_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1 ,ENABLE);  
	
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;//重复寄存器，用于自动更新pwm占空比
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	//下面几个参数是高级定时器才会用到，通用定时器不用配置
  TIM_OCInitStructure.TIM_OCNPolarity=TIM_OCNPolarity_Low; //设置互补端输出极性
	TIM_OCInitStructure.TIM_OutputNState=TIM_OutputNState_Enable;//使能互补端输出
	TIM_OCInitStructure.TIM_OCIdleState=TIM_OCIdleState_Reset;   //死区后输出状态
	TIM_OCInitStructure.TIM_OCNIdleState=TIM_OCNIdleState_Reset;//死区后互补端输出状态
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void Tim4_init(void)
{
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  				TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	/**********************************************************
	72 000 000/72=1M
	1000 000/2500=400Hz
	所以产生的PWM为400Hz
	周期为2.5ms，对应2500的计算值，1ms~2ms对应的计算值为1000~2000；
	**********************************************************/
	TIM_TimeBaseStructure.TIM_Period = 2499;		//计数上线	
	TIM_TimeBaseStructure.TIM_Prescaler = 71;	//pwm时钟分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;//？？	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;		//向上计数
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
}

void PWM_OUT_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

  /* GPIOA and GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	Tim4_init();	
	Tim1_init();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmWriteMotor
**功能 : PWM写入电机
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void pwmWriteMotor(uint8_t index, uint16_t value)
{    
	if(value > Moto_PwmMax)  value = Moto_PwmMax;
	if(value <= 0)           value = 0;
	// pwmWritePtr(index, value);
}

void writeMotors(int16_t *Moter)
{
  uint8_t i;
  for(i = 0; i < 4; i++)
     pwmWriteMotor(i, Moter[i]);
}

void moto_PwmRflash(u16 *Moter)
{		
	for(u8 i=0;i<MOTOR_NUM;i++)
	{
     if(*(Moter+i) > Moto_PwmMax)  *(Moter+i) = Moto_PwmMax;
  }
	for(u8 i=0;i<MOTOR_NUM;i++)
	{
     if(*(Moter+i) <= 0 )  *(Moter+i) = 0;
  }
	
	if(MOTOR_NUM ==4 ){
		TIM4->CCR1 = 1000 + *(Moter++);
		TIM4->CCR2 = 1000 + *(Moter++);
		TIM4->CCR3 = 1000 + *(Moter++);
		TIM4->CCR4 = 1000 + *Moter;
	}
	else if(MOTOR_NUM == 6){
		TIM4->CCR1 = 1000 + *(Moter++);
		TIM4->CCR2 = 1000 + *(Moter++);
		TIM4->CCR3 = 1000 + *(Moter++);
		TIM4->CCR4 = 1000 + *(Moter++);
		TIM1->CCR1 = 1000 + *(Moter++);
		TIM1->CCR2 = 1000 + *Moter;
	}
}

void moto_STOP(void)
{
	if(MOTOR_NUM ==4 ){	
		TIM4->CCR1 = 1000;
		TIM4->CCR2 = 1000;
		TIM4->CCR3 = 1000;
		TIM4->CCR4 = 1000;
	}
	else if(MOTOR_NUM == 6){
		TIM4->CCR1 = 1000;
		TIM4->CCR2 = 1000;
		TIM4->CCR3 = 1000;
		TIM4->CCR4 = 1000;
		TIM1->CCR1 = 1000;
		TIM1->CCR2 = 1000;
	}
}
