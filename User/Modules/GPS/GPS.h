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
	u8 UtcTime[11]; //ʱ��
	u8 Statue;   //��λ״̬
	u8  Latitude[10];	//γ��
	u8 LatitudeNS; //γ�Ȱ���
	u8 Longitude[11];	 //����	
	u8 LongitudeEW;//���Ȱ���
	u8 Speed[6];	     //��������
	u8 Azimuth[6];	 //����
	u8 UtcData[7]; //����	  

	u8 Altitude[8];   //�߶�
		//u8 SatelliteNum;//������ 
	}GPSINFO;


//�����ַ���str��ptr���ֵ�λ��ֵ
u16 FindStr(u8 *str,u8 *ptr);
//GPS����															 	
void GPSParse(void);
void GpsDataInit(void);
//����γ��ת��Ϊ����
float LatToRad(u8 *Lat);
float LonToRad(u8 *Lon);
//�Ѿ�����ľ�γ���������ľ���
double DistanceCal(float LatFrom,float LonFrom,float LatTo,float LonTo);
	











#endif
