/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��board_config.h
 * ����     ��ϵͳ��������     
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"

/*-----------------����汾��Ϣ---------------------*/
#define VERSION_OLED "v2.3.5"
#define VERSION_USART 235

/*-------------------����ѡ��-----------------------*/
#define QUADROTOR    //��������� Xģʽ
//#define HEXACOPTER    //��������� Xģʽ

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
		#define MULTIROTOR "QUADROTOR"
#elif defined HEXACOPTER
		#define MOTOR_NUM 6
		#define MULTIROTOR "HEXAROTOR"
#endif 

/*-------------------����վѡ��---------------------*/
#define HT_HAWK//ʹ�ú��ص���վ

#ifndef HT_HAWK //���û�ж���HT_HAWK
#define ANO//��ʹ����������վ
#endif

/*----------------�ɿش���1������-------------------*/
#define HAWK_USART1_BAUD  115200

/*------------------������ƫ����--------------------*/
/*------------------��Ҫ  ��Ҫ��--------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD     0x0801FFF0

/*--------------------�������----------------------*/
#define IDLING  200

/*------------------ң�ؿ��Ʒ�ʽѡ��----------------*/
#define RC_CONTROL_USE_NRF24l01

/*----------------�����ǲɼ��޷�--------------------*/
#define GYRO_GATHER   30 

/*--------------------���ż��----------------------*/
#define RC_MINCHECK   1150
#define RC_MAXCHECK   1850

/*---------------��ǰң�ص�RCֵ���-----------------*/
#define MINRCVALUE    1100 
#define MIDRCVALUE    1500
#define MAXRCVALUE    1900

/*-------------------����/����ʱ��------------------*/
#define ARMED_TIME         600    //��������ʱ��
#define AUTODISARMED_TIME  2000   //�Զ�����ʱ��

/*----------------------�ظ�ģʽ--------------------*/
#define MANUAL_High      1
#define ULTRASONIC_High  2
#define ATMOSPHERE_High  3
#define AUTO_High        4
#define ERROR_High       5
#define ACC_High         6

/*-------------------RCģʽ/PCģʽ------------------*/
#define RC_MODE          0x01
#define PC_MODE          0x02

/*---------------------����ģʽ---------------------*/
#define STABILIZE_MODE   0x00
#define ALTHOLD_MODE     0x02
#define POSHOLD_MODE     0x03
#define AUTO_MODE        0x04
#define LAND_MODE        0x05
#define CIRCLE_MODE      0x06
#define RTL_MODE         0x07

/*-------------------����ģʽ����-------------------*/
#define SecondHigh_Factor 0.02       //������Ĳ���һ�����ڸ߶ȿ��ƻ����˲�
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //�����������Ч�߶ȣ���λ��mm
#define Ultrasonic_MIN_Height 200    //�����������Ч�߶ȣ���λ��mm
#define Baro_MAX_Height       8000   //��ѹ�������Ч�߶ȣ���λ��mm,���Ը���ʵ���޸�
#define Baro_MIN_Height       1000   //��ѹ�������Ч�߶ȣ���λ��mm,�ٵ���ʵ��̫�ɿ������Ը���ʵ���޸�
#define TT                    0.0025 //��������2.5ms���붨ʱ��5�ж�ʱ���Ӧ
#define CTRL_HEIGHT           1      //0ʧ�ܣ�1ʹ�ܿظ߹���
#define TAKE_OFF_THR          550    //���ݸ��˷ɻ�ʵ��������������������ţ����ڶ���ģʽ�����������
#define MAX_PWM				        100		 //���PWM���Ϊ100%����
#define MAX_THR               80 		 //����ͨ�����ռ��80%����20%��������
typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
