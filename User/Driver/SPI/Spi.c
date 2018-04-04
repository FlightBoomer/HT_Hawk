/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：Spi.c
 * 描述     ：Spi配置
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "spi.h"
uint8 rxdata=0;
uint8 rxdata1=0;

void SPI3_Config(void)
{ 
		GPIO_InitTypeDef GPIO_InitStructure;
		SPI_InitTypeDef  SPI_InitStructure;
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
	                         RCC_APB2Periph_GPIOB|
	                         RCC_APB2Periph_GPIOD|
	                         RCC_APB2Periph_AFIO, ENABLE);  
    
    //  PA5--CLK  PA7--MOSI  
    GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_3|GPIO_Pin_5; 
    GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode  =  GPIO_Mode_AF_PP; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    //PA6--MISO 
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    //PA4--NSS
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOA, &GPIO_InitStructure);
 
		//PC2--CE
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
		
		//PC3--IRQ  
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;   
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU; //上拉输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);
	                     
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //全双工
		SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                      //主模式
		SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //一次转换8位
		SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;                        //空闲电平低电平
		SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;                      //第一个上升沿采样
		SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                         // NSS为软件模式
		SPI_InitStructure.SPI_BaudRatePrescaler =SPI_BaudRatePrescaler_32; //
		SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;        					//数据传输低位在前
		SPI_InitStructure.SPI_CRCPolynomial = 7;                           //CRC校验模式
    SPI_Init(SPI3, &SPI_InitStructure);                                //初始化
    
   // SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);
    
    SPI_Cmd(SPI3, ENABLE); //SPI1
		rxdata=SPI3_ReadWriteByte(0xff);//启动传输	
}
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   
//SPI_BaudRatePrescaler_8   8分频   
//SPI_BaudRatePrescaler_16  16分频  
//SPI_BaudRatePrescaler_256 256分频 
  
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
  assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI3->CR1&=0XFFC7;
	SPI3->CR1|=SPI_BaudRatePrescaler;	//设置SPI3速度 
	SPI_Cmd(SPI3,ENABLE); 
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI3_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
		{
		retry++;
		if(retry>200)return 0;
		}			  
	SPI_I2S_SendData(SPI3, TxData); //通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
		{
		retry++;
		if(retry>200)return 0;
		}	  						    
	return SPI_I2S_ReceiveData(SPI3); //返回通过SPIx最近接收的数据					    
}
