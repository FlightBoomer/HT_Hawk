#ifndef __MS5611_H
#define	__MS5611_H
#include<stdint.h>
#include "stm32f10x.h"
/* MPU6050 Register Address ------------------------------------------------------------*/
#define MS561101BA_ADC_RD          0x00
#define	MS561101BA_PROM_RD 	       0xA0
#define MS561101BA_PROM_CRC        0xAE

#define MS561101BA_SlaveAddress    0xEE  //MS5611的地址
#define MS561101BA_RST             0x1E  //cmd 复位

#define	MS561101BA_D2_OSR_4096   0x58	// 9.04 mSec conversion time ( 110.62 Hz)
#define	MS561101BA_D1_OSR_4096   0x48

#define MS5611_OSR256					 		 0x40
#define MS5611_OSR512					 		 0x42
#define MS5611_OSR1024					   0x44
#define MS5611_OSR2048					   0x46
#define MS5611_OSR4096					   0x48
#define FILTER_num 20

#define PRESSURE_P  5
#define PRESSURE_I  2
#define PRESSURE_D  0

#define PRESSURE_I_OUTMAX 400
#define PRESSURE_I_OUTMIN -400

#define IPressure_time 0.025 	//单位是秒

extern u8 lockpressure;//锁定当前大气压
extern float baro_dis_old,baro_dis_delta,Baro_Height_Source;
extern s8 baro_start_f;
extern double Pressure_out;
u8  MS5611_init(void);
void MS561101BA_getTemperature(void);
void Get_High(void);
void Pressure_High(void);
#endif
