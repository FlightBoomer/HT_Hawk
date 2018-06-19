/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��GPS.c
 * ����     ��GPS����
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "include.h"
#include "GPS.h"

#include"GPS.h"
#include"USART.h"
#include<math.h>

u8 GpsBuffer[NMEA_COUNT_MAX]={0}; 
//char FindString[80];
char *pFindString="$GPRMC,015152.589,V,0000.0000,N,00000.0000,E,,,020504,,*11\r\n";
u8 GpsFlag;
GPSINFO GpsInfo;

////����1�������� ����GpsBuffer�У�
//void USART1_IRQHandler(void)
//{
//	u8 Buffer;
//	static u16 Count=0;
//	static u8 Num=0;
//	if(SET==USART_GetITStatus(USART1,USART_IT_RXNE))
//	{
//		Buffer=USART_ReceiveData(USART1);

//		if('$'==Buffer)
//		{
//			Num++;
//			if(7==Num)//һ��GPS��ϢΪ6����
//			{
//				GpsBuffer[0]='$';
//				Count=1;
//				GpsFlag=1;
//				Num=1;
//			}
//			else
//			{
//				GpsBuffer[Count++]=Buffer; 	
//			}		
//		}
//		else
//		{
//			GpsBuffer[Count++]=Buffer;
//		}
//	}
//	USART_ClearITPendingBit(USART1, USART_IT_RXNE);//����жϱ�־
//}
void GpsDataInit(void)
{
	GpsInfo.UtcTime[10]='\0';
	GpsInfo.Latitude[9]='\0';
	GpsInfo.Longitude[10]='\0';
	GpsInfo.Speed[5]='\0';
	GpsInfo.Azimuth[5]='\0';
	GpsInfo.UtcData[6]='\0';
	GpsInfo.Altitude[7]='\0';
	
}
u16 FindStr(u8 *str,u8 *ptr)
{
	u16 index=0;
	u8 *STemp=NULL;
	u8 *PTemp=NULL;
	u8 *MTemp=NULL;
	if(0==str||0==ptr)
		return 0;
	for(STemp=str;*STemp!='\0';STemp++)	 //���β����ַ���
	{
		index++;  //��ǰƫ������1
		MTemp=STemp; //ָ��ǰ�ַ���
		//�Ƚ�
		for(PTemp=ptr;*PTemp!='\0';PTemp++)
		{	
			if(*PTemp!=*MTemp)
			break;
			MTemp++;
		}
		if(*PTemp=='\0')  //��������Ҫ���ҵ��ַ����˳�
			break;
	}
	return index;
}
//GPS����
 
void GPSParse(void)
{
 	u8 CommaNum=0; //������
	u8 BufIndex=0; //������
 	u8 Sbuf;
	u8 *Pstr;
	u16 index;
	//memset(&GpsInfo,0x00,sizeof(GpsInfo));//����ṹ��	,ò��û�⹦��
	index= FindStr(GpsBuffer,"$GPRMC,");//����
	if(index)
	{
		CommaNum=0;
		Pstr=GpsBuffer+index+6;	 //�ҵ�GPRMC������ĵ�ַ
		do
		{
			Sbuf=*Pstr++;	
		//	USART_SendData(USART1,Sbuf);
			switch(Sbuf)
			{
				case ',':CommaNum++;  //ͨ�����ŵ���Ŀ������״̬����
						 BufIndex=0;
						 break;
				default:
						switch(CommaNum)
						{
							case 0:GpsInfo.UtcTime[BufIndex]=Sbuf;break; 
							case 1:GpsInfo.Statue=Sbuf;             break;
							case 2:GpsInfo.Latitude[BufIndex]=Sbuf;break;
							case 3:GpsInfo.LatitudeNS=Sbuf;break;
							case 4:GpsInfo.Longitude[BufIndex]=Sbuf;break;
							case 5:GpsInfo.LongitudeEW=Sbuf;break;
							case 6:GpsInfo.Speed[BufIndex]=Sbuf;break;
							case 7:GpsInfo.Azimuth[BufIndex]=Sbuf;break;
							case 8:GpsInfo.UtcData[BufIndex]=Sbuf;break;
							default:break;
						}
						BufIndex++;	//
						break;
			}
		}while(Sbuf!='*');//ֱ�����֡�*���˳�	
		
	}
	//���ϲ���
	index= FindStr(GpsBuffer,"$GPGGA,");
	if(index)
	{
		CommaNum=0;
		Pstr=GpsBuffer+index+6;
		do
		{
			Sbuf=*Pstr++ ;
			switch(Sbuf)
			{
				case ',':CommaNum++;
						 BufIndex=0;
						 break;
				default:
						switch(CommaNum)
						{
							case 0:break;
							case 1:break;
							case 2:break;
							case 3:break;
							case 4:break;
							case 5:break;
							case 6:break;
							case 7:break;
							case 8:GpsInfo.Altitude[BufIndex]=Sbuf;break;
							default:break;
						}
						BufIndex++;
						break;
			}
		}while(Sbuf!='*');	
	} 
}
 
float LatToRad(u8 *Lat)
{
 	float Rad;
	u16 Data;
	Data=Lat[0]*10+Lat[1];
	Rad=Lat[2]*10+Lat[3]+Lat[5]*0.1+Lat[6]*0.01+Lat[7]*0.001+Lat[8]*0.0001;
	Rad=Rad/60;
	Rad=Rad+Data;
	return Rad;			
}
 
float LonToRad(u8 *Lon)
{
 	float Rad;
	u16 Data;
	Data=Lon[0]*100+Lon[1]*10+Lon[2];
	Rad=Lon[3]*10+Lon[4]+Lon[6]*0.1+Lon[7]*0.01+Lon[8]*0.001+Lon[9]*0.0001;
	Rad=Rad/60;
	Rad=Rad+Data;
	return Rad;
}

double DistanceCal(float LatFrom,float LonFrom,float LatTo,float LonTo)
{
	double LatFrom1,LonFrom1,LatTo1,LonTo1,LonDiff;
	double Temp1,Temp2,Temp3;
	double Distance;
	LatFrom1=LatFrom*PI/180;
	LonFrom1=LonFrom*PI/180;
	LatTo1=LatTo*PI/180;
	LonTo1=LonTo*PI/180;
	LonDiff=LonTo1-LonFrom1;
	Temp1=cos(LatTo1)*sin(LonDiff);	
	Temp1=Temp1*Temp1;
	Temp2=cos(LatFrom1)*sin(LatTo1)-sin(LatFrom1)*cos(LatTo1)*cos(LonDiff);
	Temp2=Temp2*Temp2;
	Temp3=sin(LatFrom1)*sin(LatTo1)+cos(LatFrom1)*cos(LatTo1)*cos(LonDiff);
	Distance=atan(sqrt(Temp1+Temp2)/Temp3);
	Distance=EARTHR*Distance;
	return Distance ;		
}

//typedef struct 
//{
//    uint8_t GPS_data_Count;
//    uint8_t Frame;
//    uint64_t pd;
//    uint64_t pd1;
//    uint8_t GPS_Buffer[256];
//    uint8_t *pstr;              //����
//}Gps_data_addr_st;

//Gps_data_addr_st Gps_data_addr;

////�ж�

//void UART0_IRQHandler(void)
//{
//    uint8_t u8InChar=0xFF;
//    static uint8_t KeyWord_Count;
//    uint32_t u32_IQR= UART0->ISR;
//    uint32_t u32_status = UART0->FSR;
//    UART0->ISR = u32_IQR;
//    //���ڽ�������
//    if (((_UART_IS_RX_READY(UART0) > 0) || (_UART_IS_RX_TIMEOUT(UART0) > 0)) && (_UART_IS_RX_BUF_ERR(UART0) == 0))
//    {
//        while(_UART_IS_RX_EMPTY(UART0) == 0)
//        {
//            __disable_irq();
//            _UART_RECEIVEBYTE(UART0, u8InChar);
//            Gps_data_addr.GPS_Buffer[Gps_data_addr.GPS_data_Count] = u8InChar;        
//            switch(u8InChar)
//            {   //NEMA0183 ��ʽ������$��ʼ,\r\n��β(0x0d 0x0a)
//                case '$': 
//                    Gps_data_addr.pd <<= 8;
//                    Gps_data_addr.pd |= '$';//���'$'
//                    Gps_data_addr.pd <<= 8;
//                    Gps_data_addr.pd |= Gps_data_addr.GPS_data_Count;//'$'�������е��±�
//                    KeyWord_Count = Gps_data_addr.GPS_data_Count;
//                break;
//                case 0x0D: 
//                    Gps_data_addr.pd <<= 8;
//                    Gps_data_addr.pd |= 0x0d;//���0x0d
//                    Gps_data_addr.pd <<= 8;
//                    Gps_data_addr.pd |= Gps_data_addr.GPS_data_Count;////0x0d�������е��±�
//                    //����һ֡����,����GPS������Ϣ,5us���������
//                    //ģ����ռʽ����,�����ж��˳��������Կ�ʼ,���ȼ���ͬTMR0�ж�,����ʱ�������14ms�����
//                    if( (Gps_data_addr.GPS_Buffer[KeyWord_Count+5] == 'C') ||      //GPRMC
//                        (Gps_data_addr.GPS_Buffer[KeyWord_Count+5] == 'A') ||      //GPGSA
//                        (Gps_data_addr.GPS_Buffer[KeyWord_Count+5] == 'V') ||      //GPGSV
//                        (Gps_data_addr.GPS_Buffer[KeyWord_Count+5] == 'L') )       //GPGLL
//                    {
//                        GPS_Decode_Run();//����TMR1�ж�
//                    }
//                    else
//                    {
//                        Gps_data_addr.pd = 0;
//                    }
//                break;
//                default:    break;
//            }
//            __enable_irq();        
//            Gps_data_addr.GPS_data_Count++;
//        }
//    }
//    else
//    {//����BUFFER�����������
//        while(_UART_IS_RX_EMPTY(UART0) == 0)
//        {
//            _UART_FLUSH_FIFO(UART0, UART_FCR_TFR_Msk | UART_FCR_RFR_Msk);
//            _UART_RECEIVEBYTE(UART0, u8InChar);
//        }  
//    }
//}
