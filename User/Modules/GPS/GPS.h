#ifndef __GPS_H
#define	__GPS_H
#include<stdint.h>
#include "stm32f10x.h"
#include<stdio.h>
#include<string.h>

#define NMEA_COUNT_MAX 500
#ifndef PI 
#define PI 3.1415926535898
#endif
#define EARTHR 6371004 
typedef struct
{
	u8 UtcTime[11]; //时间
	u8 Statue;   //定位状态
	u8  Latitude[10];	//纬度
	u8 LatitudeNS; //纬度半球
	u8 Longitude[11];	 //经度	
	u8 LongitudeEW;//经度半球
	u8 Speed[6];	     //地面速率
	u8 Azimuth[6];	 //航向
	u8 UtcData[7]; //日期	  

	u8 Altitude[8];   //高度
		//u8 SatelliteNum;//卫星数 
	}GPSINFO;


//查找字符串str中ptr出现的位置值
u16 FindStr(u8 *str,u8 *ptr);
//GPS解析															 	
void GPSParse(void);
void GpsDataInit(void);
//将经纬度转换为弧度
float LatToRad(u8 *Lat);
float LonToRad(u8 *Lon);
//已经两点的经纬度求解两点的距离
double DistanceCal(float LatFrom,float LonFrom,float LatTo,float LonTo);
	











#endif
