/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：board_config.h
 * 描述     ：系统参数配置     
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#ifndef __BOARD_CONFIG_H
#define	__BOARD_CONFIG_H

#include "include.h"

/*-----------------程序版本信息---------------------*/
#define VERSION_OLED "v2.3.5"
#define VERSION_USART 235

/*-------------------机型选择-----------------------*/
#define QUADROTOR    //四旋翼机型 X模式
//#define HEXACOPTER    //六旋翼机型 X模式

#ifdef QUADROTOR 
		#define MOTOR_NUM 4
		#define MULTIROTOR "QUADROTOR"
#elif defined HEXACOPTER
		#define MOTOR_NUM 6
		#define MULTIROTOR "HEXAROTOR"
#endif 

/*-------------------地面站选择---------------------*/
#define HT_HAWK//使用恒拓地面站

#ifndef HT_HAWK //如果没有定义HT_HAWK
#define ANO//便使用匿名地面站
#endif

/*----------------飞控串口1波特率-------------------*/
#define HAWK_USART1_BAUD  115200

/*------------------向量表偏移量--------------------*/
/*------------------重要  不要动--------------------*/
#define FLASH_EXCURSION  0x20000
#define pro_FALG_ADD     0x0801FFF0

/*--------------------电机怠速----------------------*/
#define IDLING  200

/*------------------遥控控制方式选择----------------*/
#define RC_CONTROL_USE_NRF24l01

/*----------------陀螺仪采集限幅--------------------*/
#define GYRO_GATHER   30 

/*--------------------油门检查----------------------*/
#define RC_MINCHECK   1150
#define RC_MAXCHECK   1850

/*---------------当前遥控的RC值情况-----------------*/
#define MINRCVALUE    1100 
#define MIDRCVALUE    1500
#define MAXRCVALUE    1900

/*-------------------上锁/解锁时间------------------*/
#define ARMED_TIME         600    //上锁解锁时间
#define AUTODISARMED_TIME  2000   //自动上锁时间

/*----------------------控高模式--------------------*/
#define MANUAL_High      1
#define ULTRASONIC_High  2
#define ATMOSPHERE_High  3
#define AUTO_High        4
#define ERROR_High       5
#define ACC_High         6

/*-------------------RC模式/PC模式------------------*/
#define RC_MODE          0x01
#define PC_MODE          0x02

/*---------------------飞行模式---------------------*/
#define STABILIZE_MODE   0x00
#define ALTHOLD_MODE     0x02
#define POSHOLD_MODE     0x03
#define AUTO_MODE        0x04
#define LAND_MODE        0x05
#define CIRCLE_MODE      0x06
#define RTL_MODE         0x07

/*-------------------定高模式参数-------------------*/
#define SecondHigh_Factor 0.02       //与下面的参数一起用于高度控制互补滤波
#define MainHigh_Factor   0.98

#define Ultrasonic_MAX_Height 2500   //超声波最高有效高度，单位是mm
#define Ultrasonic_MIN_Height 200    //超声波最低有效高度，单位是mm
#define Baro_MAX_Height       8000   //气压计最高有效高度，单位是mm,可以根据实际修改
#define Baro_MIN_Height       1000   //气压计最低有效高度，单位是mm,再低其实不太可靠，可以根据实际修改
#define TT                    0.0025 //控制周期2.5ms，与定时器5中断时间对应
#define CTRL_HEIGHT           1      //0失能，1使能控高功能
#define TAKE_OFF_THR          550    //根据个人飞机实际情况，设置起飞离地油门，用于定高模式下稍推油起飞
#define MAX_PWM				        100		 //最大PWM输出为100%油门
#define MAX_THR               80 		 //油门通道最大占比80%，留20%给控制量
typedef void (*rcReadRawData)(void);        

#endif /* __BOARD_CONFIG_H */
