/*
*********************************************************************************************************
*	                                  
*	ģ������ : STM32 USBӲ������ģ��    
*	�ļ����� : usb_hw.c
*	��    �� : V2.0
*	USB�̼������� : V3.3.0
*	˵    �� :  USBӲ������ģ�� 
*	�޸ļ�¼ :
*
*********************************************************************************************************
*/

#include "stm32f10x.h"
#include "board_config.h"

#include "stm32f10x_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_hw.h"
#include "usb_pwr.h"

/* �������USB���������GPIO, PC4 */
#define	RCC_USB_PULL_UP		RCC_APB2Periph_GPIOA
#define	PORT_USB_PULL_UP	GPIOA
#define	PIN_USB_PULL_UP		GPIO_Pin_8

#define USB_CABLE_ENABLE()	GPIO_ResetBits(PORT_USB_PULL_UP, PIN_USB_PULL_UP)	/* ����USB�豸  */
#define USB_CABLE_DISABLE()	GPIO_SetBits(PORT_USB_PULL_UP, PIN_USB_PULL_UP)		/* �Ͽ�USB�豸 */

USB_COM_FIFO_T g_tUsbFifo;		/* ����һ��ȫ�ֵĽṹ�壬����FIFO */

static void IntToUnicode (uint32_t _ulValue , uint8_t *_pBuf , uint8_t _ucLen);

/*
*********************************************************************************************************
*	�� �� ��: bsp_InitUsb
*	����˵��: ��ʼ��CPU��USBӲ���豸�����ÿ������������GPIO������USB����ʱ��48MHz������USB�ж�
*	��    �Σ���
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_InitUsb(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* ʹ�ܿ���USB��������GPIO��ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_USB_PULL_UP, ENABLE);
	
	/* ���� USB �������� */
	USB_CABLE_DISABLE();	/* �Ͽ�USB�豸 */
	GPIO_InitStructure.GPIO_Pin = PIN_USB_PULL_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(PORT_USB_PULL_UP, &GPIO_InitStructure);

	/* ����USBʱ��Դ */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
	
	/* ʹ��USBʱ�� */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
	
	/* ����USB�ж� */
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		
		NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		#if 0	/* ������Ҫʹ���ж�:USB�ӹ���״̬���ָ� */
		{
			EXTI_InitTypeDef EXTI_InitStructure;
			
			/* �����ⲿ�ж���18���ӵ�USB IP�ж�(CPU�ڲ�����) */
			EXTI_ClearITPendingBit(EXTI_Line18);
			EXTI_InitStructure.EXTI_Line = EXTI_Line18; /* USB�ӹ���״̬���ָ� */
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
			EXTI_InitStructure.EXTI_LineCmd = ENABLE;
			EXTI_Init(&EXTI_InitStructure);
		}
		#endif
	}

	g_tUsbFifo.usRxWrite = 0;
	g_tUsbFifo.usRxRead = 0;
	g_tUsbFifo.usTxWrite = 0;
	g_tUsbFifo.usTxRead = 0;

	USB_Init();	
}

/*
*********************************************************************************************************
*	�� �� ��: usb_EnterLowPowerMode
*	����˵��: ����͹���ģʽ����USB�豸�������ģʽʱ���ر�ϵͳʱ�Ӻ͵�Դ
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void usb_EnterLowPowerMode(void)
{
	/* �����豸״̬Ϊ suspend, bDeviceState ȫ�ֱ����� usb_pwr.c �ж��� */
	bDeviceState = SUSPENDED;

	#if 0	/* ������Ҫ�رյ�Դ ���������豸�ɽ������״̬�����⴮�ڵ��豸��ñ��ֳ����� */
		/* �� EXTI Line18 �жϱ�־ */
		EXTI_ClearITPendingBit(EXTI_Line8);
		
		/* �ڵ͹���״̬���رյ�ѹ������ */
		PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);	
	#endif
}

/*
*********************************************************************************************************
*	�� �� ��: usb_LeaveLowPowerMode
*	����˵��: �˳��͹���ģʽ���ָ�ϵͳʱ�ӡ�
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void usb_LeaveLowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;  /* ȫ�ֱ���Device_Info ��USB�̼��� usb_init.c �ж��� */	

#if 0	/* ������Ҫ�ָ�ϵͳʱ�� */
	/* ʹ�� HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	
	/* �ȴ� HSE ���� */
	RCC_WaitForHSEStartUp();
	
	/* ʹ�� HSE */
	RCC_HSEConfig(RCC_HSE_ON);
	
	/* �ȴ� HSE ���� */
	while (RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET);
	
	/* ʹ��PLL */
	RCC_PLLCmd(ENABLE);
	
	/* �ȴ�PLL���� */
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);
	
	/* ѡ��PLL��Ϊϵͳʱ��Դ */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	/* �ȴ�PLLϵͳʱ��Դ�ȶ� */
	while (RCC_GetSYSCLKSource() != 0x08);
#endif
	
	/* ����ȫ�ֱ���bDeviceState����ʾusb�豸״̬ */
	if (pInfo->Current_Configuration != 0)
	{
		bDeviceState = CONFIGURED;	/* ���óɹ� */
	}
	else
	{
		bDeviceState = ATTACHED;	/* USB�豸�����ӣ�����δ���� */
	}	
}

/*
*********************************************************************************************************
*	�� �� ��: usb_CableConfig
*	����˵��: �������USB�������ӺͶϿ�
*	��    ��: _ucMode �� ENABLE��ʾ���ӣ�DISABLE��ʾ�Ͽ�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void usb_CableConfig(uint8_t _ucMode)
{
	if (_ucMode == DISABLE)
	{
		USB_CABLE_DISABLE();	/* �Ͽ�USB�豸 */
	}
	else
	{
		USB_CABLE_ENABLE();		/* ����USB�豸, ʵ���Ͻ�D+��������ʱUSB�豸���ܱ���������⵽�� */
	}
}
				   
/*
*********************************************************************************************************
*	�� �� ��: usb_SendBuf
*	����˵��: ��PC��������һ������
*	��    �Σ�_pBuf ���뻺����; 	_ucLen : ���ݳ���
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
void usb_SendBuf(uint8_t *_pTxBuf, uint8_t _ucLen)
{
    UserToPMABufferCopy(_pTxBuf, ENDP1_TXADDR, _ucLen);
    SetEPTxCount(ENDP1, _ucLen);
    SetEPTxValid(ENDP1); 
}

/*
*********************************************************************************************************
*	�� �� ��: Get_SerialNum
*	����˵��: ��ȡһ���豸���кţ���Ҫת��ΪUNICODE��ʽ
*	��    �Σ�_pBuf Ŀ�껺����
*	�� �� ֵ: �������(���账��)
*********************************************************************************************************
*/
void Get_SerialNum(uint8_t *_pBuf)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
	
	Device_Serial0 = *(__IO uint32_t*)(0x1FFFF7E8);
	Device_Serial1 = *(__IO uint32_t*)(0x1FFFF7EC);
	Device_Serial2 = *(__IO uint32_t*)(0x1FFFF7F0);
	
	Device_Serial0 += Device_Serial2;
	
	if (Device_Serial0 != 0)
	{
		IntToUnicode (Device_Serial0, &_pBuf[2] , 8);
		IntToUnicode (Device_Serial1, &_pBuf[18], 4);
	}
}

/*
*********************************************************************************************************
*	�� �� ��: IntToUnicode
*	����˵��: ��һ��32λ��HEXֵת��ΪUnicode�����ʽ���ַ���
*	��    �Σ�_ulValue : 32λHex�� 
*			  _pBuf : Ŀ�껺����
*			 _ucLen : Ŀ���볤��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
static void IntToUnicode (uint32_t _ulValue , uint8_t *_pBuf , uint8_t _ucLen)
{
	uint8_t idx = 0;
	
	for( idx = 0 ; idx < _ucLen ; idx ++)
	{
		if( ((_ulValue >> 28)) < 0xA )
		{
			_pBuf[2 * idx] = (_ulValue >> 28) + '0';
		}
		else
		{
			_pBuf[2 * idx] = (_ulValue >> 28) + 'A' - 10; 
		}
		
		_ulValue = _ulValue << 4;
		
		_pBuf[ 2 * idx + 1] = 0;
	}
}

/*
*********************************************************************************************************
*	�� �� ��: SaveHostDataToBuf
*	����˵��: ��USB�������͵����ݻ��浽ȫ�ֻ��������ú�����USB�жϷ��������á�
*	��    ��: _pInBuf :���뻺������PC�����豸������ 
*			  _pBuf : Ŀ�껺����
*			 _ucLen : Ŀ���볤��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void exeplay_write_appmask(u16 val)
{
  RCC->APB1ENR|=1<<28;  	//ʹ�ܵ�Դʱ��	    
	RCC->APB1ENR|=1<<27; 	//ʹ�ܱ���ʱ��	    
	PWR->CR|=1<<8;      	//ȡ��������д���� 
	BKP->DR2=val;			//���ҪҪд���־ֵ
}
/*
*********************************************************************************************************
*	�� �� ��: SaveHostDataToBuf
*	����˵��: ��USB�������͵����ݻ��浽ȫ�ֻ��������ú�����USB�жϷ��������á�
*	��    ��: _pInBuf :���뻺������PC�����豸������ 
*			  _pBuf : Ŀ�껺����
*			 _ucLen : Ŀ���볤��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void usb_SaveHostDataToBuf(uint8_t *_pInBuf, uint16_t _usLen)
{
	uint16_t i;
	u8 datacheck=0;
	
	extern void delay_ms(u16 nms);
	extern void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite);
	if(_pInBuf[0]==0x8A)//���´���¿���Ҫ����FLASH����PID�޸Ĺ�����λ���·��ĸ���FLASH����
	if(_pInBuf[1]==0x5B)
	if(_pInBuf[2]==0x1C)
	if(_pInBuf[3]==0xAC)
	{
					 
		 for(i=0;i<4;i++)datacheck+=_pInBuf[i];
		 if(datacheck ==_pInBuf[4])
			{
				exeplay_write_appmask(0x5050);//д��Ҫ���³���
				i=0x0000;
				STMFLASH_Write(pro_FALG_ADD,&i,1);
				USB_CABLE_DISABLE();	/* �Ͽ�USB�豸 */
				delay_ms(1500);    //1.5���λ
				NVIC_SystemReset();//��λһ��
			}
		}
	/* δ���ǻ�������������� */
	for (i = 0 ; i < _usLen; i++)
	{
		g_tUsbFifo.aRxBuf[g_tUsbFifo.usRxWrite] = _pInBuf[i];
		
		if (++g_tUsbFifo.usRxWrite >= USB_RX_BUF_SIZE)
		{
			g_tUsbFifo.usRxWrite = 0;
		}
	}
}
/*
*********************************************************************************************************
*	�� �� ��: usb_GetRxByte
*	����˵��: ��USB���ջ�������ȡ1���ֽ�. �����������
*	��    ��: _pInBuf :���뻺������PC�����豸������ 
*			  _pBuf : Ŀ�껺����
*			 _ucLen : Ŀ���볤��
*	�� �� ֵ: ��ȡ���ֽ�
*********************************************************************************************************
*/
uint8_t usb_GetRxByte(uint8_t *_pByteNum)
{
	uint8_t ucData;
	uint16_t usRxWrite;
	
	__set_PRIMASK(1);  		/* ���жϣ�����USB�жϳ������������ʱ�����ͻ */
	usRxWrite = g_tUsbFifo.usRxWrite;
	__set_PRIMASK(0);  		/* ���ж� */

	/* ���ͻ�����Ϊ��ʱ�������ֽ��� = 0 */
	if (g_tUsbFifo.usRxRead == usRxWrite)
	{
		*_pByteNum = 0;
		return 0;
	}
	
	/* �����1���ֽ� */
	ucData = g_tUsbFifo.aRxBuf[g_tUsbFifo.usRxRead];
	
	/* �ƶ���ָ�� */
	if (++g_tUsbFifo.usRxRead >= USB_RX_BUF_SIZE)
	{
		g_tUsbFifo.usRxRead = 0;
	}

	*_pByteNum = 1;		/* ��Ч�ֽڸ��� = 1 */
	return ucData;		
}

/*
*********************************************************************************************************
*	�� �� ��: SendDataToHost
*	����˵��: �������ݵ������� ��������á�
*	��    ��: _pInBuf :���뻺������PC�����豸������ 
*			  _pBuf : Ŀ�껺����
*			 _ucLen : Ŀ���볤��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void usb_SendDataToHost(uint8_t *_pTxBuf, uint16_t _usLen)
{
	uint16_t i;
	
	/* δ���ǻ�������������� */
	/* �Ƚ����ݻ��浽�ڴ� */
	for (i = 0 ; i < _usLen; i++)
	{
		g_tUsbFifo.aTxBuf[g_tUsbFifo.usTxWrite] = _pTxBuf[i];

		__set_PRIMASK(1);  		/* ���жϣ�����USB�жϳ������������� usTxWrite ������ͻ */
		
		if (++g_tUsbFifo.usTxWrite >= USB_RX_BUF_SIZE)
		{
			g_tUsbFifo.usTxWrite = 0;
		}

		__set_PRIMASK(0);  		/* ���ж� */
	}
}

/*
*********************************************************************************************************
*	�� �� ��: usb_GetTxWord
*	����˵��: �ӷ��ͻ�������ȡ1���֣�2�ֽ�. ����USB�ж�
*	��    ��: _pByteNum : �洢��ȡ���ֽ����ı�����ָ��
*	�� �� ֵ: �����֣�2�ֽڣ�
*********************************************************************************************************
*/
uint16_t usb_GetTxWord(uint8_t *_pByteNum)
{
	uint16_t usData;
	
	/* ���ͻ�����Ϊ��ʱ�������ֽ��� = 0 */
	if (g_tUsbFifo.usTxRead == g_tUsbFifo.usTxWrite)
	{
		*_pByteNum = 0;
		return 0;
	}
	
	/* �����1���ֽ� */
	usData = g_tUsbFifo.aTxBuf[g_tUsbFifo.usTxRead];
	
	/* �ƶ���ָ�� */
	if (++g_tUsbFifo.usTxRead >= USB_TX_BUF_SIZE)
	{
		g_tUsbFifo.usTxRead = 0;
	}
	
	/* ����2�ֽڣ�ֱ�ӷ��� */
	if (g_tUsbFifo.usTxRead == g_tUsbFifo.usTxWrite)
	{
		*_pByteNum = 1;		/* ��Ч�ֽڸ��� = 1 */
		return usData;
	}	
	
	/* �����2���ֽ� */
	usData += g_tUsbFifo.aTxBuf[g_tUsbFifo.usTxRead] << 8;

	/* �ƶ���ָ�� */
	if (++g_tUsbFifo.usTxRead >= USB_TX_BUF_SIZE)
	{
		g_tUsbFifo.usTxRead = 0;
	}

	*_pByteNum = 2;		/* ��Ч�ֽڸ��� = 2 */
	return usData;		
}
