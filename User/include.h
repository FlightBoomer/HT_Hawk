#ifndef __INCLUDE_H
#define	__INCLUDE_H

typedef unsigned short uint16_t;
typedef unsigned char  uint8_t;
typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		  有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef unsigned int   uint32;                  /* defined for unsigned 32-bits integer variable 	无符号32位整型变量 */
typedef signed   int   int32;                   /* defined for signed 32-bits integer variable 		有符号32位整型变量 */
typedef float          fp32;                    /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double         fp64;                    /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */


#include "stm32f10x.h"
#include "board_config.h"
#include "math.h"
//#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>			/* 因为用到了printf函数，所以必须包含这个文件 */
#include <string.h>			/* 因为用到了memcmp函数，所以必须包含这个文件 */
#include "usb_hw.h"			/* USB模块 */
#include "LED.h"
#include "ADC.h"
#include "I2C.h"
#include "hard_i2c.h"
#include "delay.h"
#include "TIME.h"
#include "USART.h"
#include "spi.h"
#include "MOTO.h"
#include "eeprom.h"
#include "pwm_in.h"
#include "stmflash.h"

#include "oled.h"
#include "MPU6050.h"
#include "HMC5883.h"
#include "MS5611.h"
#include "NRF24L01.h"
#include "GPS.h"
#include "Ultrasonic.h"

#include "MultiRotor_rc.h"
#include "MultiRotor_app.h"
#include "MultiRotor_ahrs.h"
#include "MultiRotor_radio.h"
#include "Algorithm_sqLite.h"
#include "MultiRotor_control.h"
#include "MultiRotor_altitute.h"

#include "Algorithm_math.h"
#include "Algorithm_filter.h"
#include "Algorithm_quaternion.h"
#include "ANO_DT.h"
#include "height_ctrl.h"
#include "mymath.h"
#endif /* __INCLUDE_H */
