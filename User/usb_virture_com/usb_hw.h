/*
*********************************************************************************************************
*	                                  
*	ģ������ : STM32 USBӲ������ģ��    
*	�ļ����� : usb_hw.h
*	��    �� : V2.0
*	USB�̼������� : V3.3.0
*	˵    �� :  ͷ�ļ�
*
*	Copyright (C), 2010-2011, ���������� www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include "usb_type.h"

#define USB_TX_BUF_SIZE		2048		/* �豸->PC�����ͻ�������С */
#define USB_RX_BUF_SIZE		2048		/* PC->�豸�����ջ�������С */

typedef struct
{
	uint8_t aTxBuf[USB_TX_BUF_SIZE];	/* ���ͻ�����, �豸->PC */
	uint8_t aRxBuf[USB_RX_BUF_SIZE];	/* ���ջ�����, PC->�豸 */
	
	uint16_t usTxRead;					/* ���ͻ�������ָ�� */
	uint16_t usTxWrite;					/* ���ͻ�����дָ�� */
	uint16_t usRxRead;					/* ���ջ�������ָ�� */
	uint16_t usRxWrite;					/* ���ջ�����дָ�� */
	
	uint16_t usTxState;					/* ����״̬ */		
}USB_COM_FIFO_T;

extern USB_COM_FIFO_T g_tUsbFifo;

void bsp_InitUsb(void);
void usb_EnterLowPowerMode(void);
void usb_LeaveLowPowerMode(void);
void usb_CableConfig(uint8_t _ucMode);
void Get_SerialNum(uint8_t *_pBuf);

void usb_SaveHostDataToBuf(uint8_t *_pInBuf, uint16_t _usLen);
uint16_t usb_GetTxWord(uint8_t *_pByteNum);
uint8_t usb_GetRxByte(uint8_t *_pByteNum);
void usb_SendDataToHost(uint8_t *_pTxBuf, uint16_t _usLen);

#endif
