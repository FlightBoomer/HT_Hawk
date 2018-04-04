/*
*********************************************************************************************************
*	                                  
*	模块名称 : USB端点事务处理模块    
*	文件名称 : usb_istr.c
*	版    本 : V2.0
*	USB固件库驱动 : V3.3.0
*	说    明 : USB端点事务处理子程序
*	修改记录 :
*		版本号  日期       作者    说明
*		v0.1    2009-12-27 armfly  创建该文件，ST固件库版本为V3.1.2
*		v1.0    2011-01-11 armfly  ST固件库升级到V3.4.0版本。
*		v2.0    2011-10-16 armfly  ST固件库升级到V3.5.0版本。优化工程结构。
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "usb_hw.h"
#include "usb_istr.h"
#include "usb_pwr.h"

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

extern  uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern uint8_t  USB_Tx_State;

/*
*********************************************************************************************************
*	函 数 名: EP1_IN_Callback
*	功能说明: 端点1 IN包（设备->PC）回调函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EP1_IN_Callback (void)
{
	/*
	为了提高传输效率，并且方便FIFO操作，将 UserToPMABufferCopy() 函数就地展开 
	void UserToPMABufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes)
	{
	  uint32_t n = (wNBytes + 1) >> 1;
	  uint32_t i, temp1, temp2;
	  uint16_t *pdwVal;
	  pdwVal = (uint16_t *)(wPMABufAddr * 2 + PMAAddr);
	  for (i = n; i != 0; i--)
	  {
	    temp1 = (uint16_t) * pbUsrBuf;
	    pbUsrBuf++;
	    temp2 = temp1 | (uint16_t) * pbUsrBuf << 8;
	    *pdwVal++ = temp2;
	    pdwVal++;
	    pbUsrBuf++;
	  }
	}		
	*/
	uint16_t i;
	uint16_t usWord;
	uint8_t ucByteNum;
	uint16_t *pdwVal;
	uint16_t usTotalSize;
	
	usTotalSize = 0;
	pdwVal = (uint16_t *)(ENDP1_TXADDR * 2 + PMAAddr);	
	for (i = 0 ; i < VIRTUAL_COM_PORT_DATA_SIZE / 2; i++)
	{
		usWord = usb_GetTxWord(&ucByteNum);
		if (ucByteNum == 0)
		{
			break;
		}
		
		usTotalSize += ucByteNum;
		
		*pdwVal++ = usWord;
		pdwVal++;		

		/*
			STM32的USB缓冲区是一个双端口的RAM，CPU一端需要使用32位方式访问，但USB模块一端使用16位方式访问。
			也就是说每个USB模块中的地址*2才能对应到控制器中的实际地址，这样每四个字节地址空间后两个字节
			地址空间是空的
		*/
	}
	
	if (usTotalSize == 0)
	{
		return;
	}
	
	SetEPTxCount(ENDP1, usTotalSize);
	SetEPTxValid(ENDP1); 
}

/*
*********************************************************************************************************
*	函 数 名: EP3_OUT_Callback
*	功能说明: 端点3 OUT包（PC->设备）回调函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void EP3_OUT_Callback(void)
{
	uint16_t usRxCnt;
	uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
	
	/* 将USB端点3收到的数据存储到USB_Rx_Buffer， 数据大小保存在USB_Rx_Cnt */
	usRxCnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
	
	/* 立即将接收到的数据缓存到内存 */
	usb_SaveHostDataToBuf(USB_Rx_Buffer, usRxCnt);
	
	/* 允许 EP3 端点接收数据 */
	SetEPRxValid(ENDP3);
}

/*
*********************************************************************************************************
*	函 数 名: SOF_Callback
*	功能说明: SOF回调函数  .SOF是host用来指示frame的开头的。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void SOF_Callback(void)
{
	static uint32_t FrameCount = 0;
	
	if (bDeviceState == CONFIGURED)
	{
		if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
		{
			/* Reset the frame counter */
			FrameCount = 0;
			
			/* Check the data to be sent through IN pipe */
			EP1_IN_Callback();
			//Handle_USBAsynchXfer();
		}
	}  
}

