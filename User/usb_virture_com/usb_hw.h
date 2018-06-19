/*
*********************************************************************************************************
*	                                  
*	模块名称 : STM32 USB硬件配置模块    
*	文件名称 : usb_hw.h
*	版    本 : V2.0
*	USB固件库驱动 : V3.3.0
*	说    明 :  头文件
*
*	Copyright (C), 2010-2011, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

#include "usb_type.h"

#define USB_TX_BUF_SIZE		2048		/* 设备->PC，发送缓冲区大小 */
#define USB_RX_BUF_SIZE		2048		/* PC->设备，接收缓冲区大小 */

typedef struct
{
	uint8_t aTxBuf[USB_TX_BUF_SIZE];	/* 发送缓冲区, 设备->PC */
	uint8_t aRxBuf[USB_RX_BUF_SIZE];	/* 接收缓冲区, PC->设备 */
	
	uint16_t usTxRead;					/* 发送缓冲区读指针 */
	uint16_t usTxWrite;					/* 发送缓冲区写指针 */
	uint16_t usRxRead;					/* 接收缓冲区读指针 */
	uint16_t usRxWrite;					/* 接收缓冲区写指针 */
	
	uint16_t usTxState;					/* 发送状态 */		
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
