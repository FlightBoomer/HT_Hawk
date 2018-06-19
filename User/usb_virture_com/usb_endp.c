/*
*********************************************************************************************************
*	                                  
*	ģ������ : USB�˵�������ģ��    
*	�ļ����� : usb_istr.c
*	��    �� : V2.0
*	USB�̼������� : V3.3.0
*	˵    �� : USB�˵��������ӳ���
*	�޸ļ�¼ :
*		�汾��  ����       ����    ˵��
*		v0.1    2009-12-27 armfly  �������ļ���ST�̼���汾ΪV3.1.2
*		v1.0    2011-01-11 armfly  ST�̼���������V3.4.0�汾��
*		v2.0    2011-10-16 armfly  ST�̼���������V3.5.0�汾���Ż����̽ṹ��
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
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
*	�� �� ��: EP1_IN_Callback
*	����˵��: �˵�1 IN�����豸->PC���ص�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EP1_IN_Callback (void)
{
	/*
	Ϊ����ߴ���Ч�ʣ����ҷ���FIFO�������� UserToPMABufferCopy() �����͵�չ�� 
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
			STM32��USB��������һ��˫�˿ڵ�RAM��CPUһ����Ҫʹ��32λ��ʽ���ʣ���USBģ��һ��ʹ��16λ��ʽ���ʡ�
			Ҳ����˵ÿ��USBģ���еĵ�ַ*2���ܶ�Ӧ���������е�ʵ�ʵ�ַ������ÿ�ĸ��ֽڵ�ַ�ռ�������ֽ�
			��ַ�ռ��ǿյ�
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
*	�� �� ��: EP3_OUT_Callback
*	����˵��: �˵�3 OUT����PC->�豸���ص�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void EP3_OUT_Callback(void)
{
	uint16_t usRxCnt;
	uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
	
	/* ��USB�˵�3�յ������ݴ洢��USB_Rx_Buffer�� ���ݴ�С������USB_Rx_Cnt */
	usRxCnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);
	
	/* ���������յ������ݻ��浽�ڴ� */
	usb_SaveHostDataToBuf(USB_Rx_Buffer, usRxCnt);
	
	/* ���� EP3 �˵�������� */
	SetEPRxValid(ENDP3);
}

/*
*********************************************************************************************************
*	�� �� ��: SOF_Callback
*	����˵��: SOF�ص�����  .SOF��host����ָʾframe�Ŀ�ͷ�ġ�
*	��    ��: ��
*	�� �� ֵ: ��
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

