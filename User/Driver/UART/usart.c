/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：usart.c
 * 描述     ：usart串口配置
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "usart.h"
#include <stdarg.h>
#include "board_config.h"


#define USART1_DR_Base  0x40013804
u8 sendbuf[5]={0x01,0x02,0x03,0x04,0x05},datanum=5;//测试用

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//开启DMA1的时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// 设置波特率
	USART_InitStructure.USART_BaudRate = HAWK_USART1_BAUD;   
	
	// 设置数据位为8位
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; 

  // 设置停止位为1位	
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  

  // 无奇偶校验
  USART_InitStructure.USART_Parity = USART_Parity_No;  

  // 没有硬件流控
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	
	// 发送与接收
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     
    
	/*完成串口COM1的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);////只使用接收中断，不使用发送中断
	USART_Cmd(USART1, ENABLE);
	DMA_USART1_Configuration();//用DMA有误码待解决
}


void DMA_USART1_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;//已经宏定义了，内存地址&串口数据寄存器地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)sendbuf;  //这个地址无所谓，后边发送的时候会重新赋值。
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = datanum; //传输数据大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA模式一次传输，不循环;DMA_Mode_Circular//
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//优先级中
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	/* Enable DMA1 Channel4Transfer Complete interrupt */  
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC, ENABLE);   
  //USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);    //不需要DMA方式接收                                                                //串口接收器DMA  
  //采用DMA方式发送  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //使能串口1的DMA发送 
}


/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
uint8_t UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART1->SR & USART_FLAG_TXE));
	return DataToSend;
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
 */
static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */

/*
 * 函数名：USART1_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口1，即USART1
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用USART1_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART1_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART1_printf( USART1, "\r\n %s \r\n", j );
 */
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //换行符
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  //字符串
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//十进制
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}

/**************【目前暂未使用串口2】*******************/
void usart2_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
		/* config USART2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO功能的时钟
  GPIO_PinRemapConfig(GPIO_Remap_USART2 ,ENABLE);  //进行重映射
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PD.09) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 9600;                 /*设置波特率为115200*/
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
	USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
	/*完成串口COM3的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART2_Put_Char(unsigned char DataToSend)
*功　　能:		蓝牙发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
uint8_t UART2_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART2, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART2->SR & USART_FLAG_TXE));
	return DataToSend;
}



