/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��usart.c
 * ����     ��usart��������
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "usart.h"
#include <stdarg.h>
#include "board_config.h"


#define USART1_DR_Base  0x40013804
u8 sendbuf[5]={0x01,0x02,0x03,0x04,0x05},datanum=5;//������

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//����DMA1��ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// ���ò�����
	USART_InitStructure.USART_BaudRate = HAWK_USART1_BAUD;   
	
	// ��������λΪ8λ
  USART_InitStructure.USART_WordLength = USART_WordLength_8b; 

  // ����ֹͣλΪ1λ	
  USART_InitStructure.USART_StopBits = USART_StopBits_1;  

  // ����żУ��
  USART_InitStructure.USART_Parity = USART_Parity_No;  

  // û��Ӳ������
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
	
	// ���������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;     
    
	/*��ɴ���COM1��ʱ�����á�GPIO���ã���������������ʼ����ʹ��*/
	USART_Init(USART1, &USART_InitStructure);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);////ֻʹ�ý����жϣ���ʹ�÷����ж�
	USART_Cmd(USART1, ENABLE);
	DMA_USART1_Configuration();//��DMA����������
}


void DMA_USART1_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	DMA_DeInit(DMA1_Channel4);
	DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;//�Ѿ��궨���ˣ��ڴ��ַ&�������ݼĴ�����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)sendbuf;  //�����ַ����ν����߷��͵�ʱ������¸�ֵ��
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = datanum; //�������ݴ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMAģʽһ�δ��䣬��ѭ��;DMA_Mode_Circular//
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//���ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	/* Enable DMA1 Channel4Transfer Complete interrupt */  
	DMA_ITConfig(DMA1_Channel4,DMA_IT_TC, ENABLE);   
  //USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);    //����ҪDMA��ʽ����                                                                //���ڽ�����DMA  
  //����DMA��ʽ����  
	USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);  //ʹ�ܴ���1��DMA���� 
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
uint8_t UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & USART_FLAG_TXE));
	return DataToSend;
}

/*
 * ��������itoa
 * ����  ������������ת�����ַ���
 * ����  ��-radix =10 ��ʾ10���ƣ��������Ϊ0
 *         -value Ҫת����������
 *         -buf ת������ַ���
 *         -radix = 10
 * ���  ����
 * ����  ����
 * ����  ����USART1_printf()����
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
 * ��������USART1_printf
 * ����  ����ʽ�������������C���е�printf��������û���õ�C��
 * ����  ��-USARTx ����ͨ��������ֻ�õ��˴���1����USART1
 *		     -Data   Ҫ���͵����ڵ����ݵ�ָ��
 *			   -...    ��������
 * ���  ����
 * ����  ���� 
 * ����  ���ⲿ����
 *         ����Ӧ��USART1_printf( USART1, "\r\n this is a demo \r\n" );
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

	while ( *Data != 0)     // �ж��Ƿ񵽴��ַ���������
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //�س���
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //���з�
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
				case 's':										  //�ַ���
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//ʮ����
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

/**************��Ŀǰ��δʹ�ô���2��*******************/
void usart2_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
		/* config USART2 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //ʹ��AFIO���ܵ�ʱ��
  GPIO_PinRemapConfig(GPIO_Remap_USART2 ,ENABLE);  //������ӳ��
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PD.09) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 9600;                 /*���ò�����Ϊ115200*/
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*��������λΪ8λ*/
	USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*����ֹͣλΪ1λ*/
	USART_InitStructure.USART_Parity = USART_Parity_No;          /*����żУ��*/    
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*û��Ӳ������*/
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*���������*/
	/*��ɴ���COM3��ʱ�����á�GPIO���ã���������������ʼ����ʹ��*/
	
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART2, ENABLE);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART2_Put_Char(unsigned char DataToSend)
*��������:		��������һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
uint8_t UART2_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART2, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART2->SR & USART_FLAG_TXE));
	return DataToSend;
}



