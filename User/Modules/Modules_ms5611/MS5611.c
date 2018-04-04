/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MS5611.c
 * 描述     ：MS5611气压计配置
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "include.h"
#include "MS5611.h"
#include "math.h"

//气压计状态机
#define SCTemperature    0x01	  //开始温度转换
#define CTemperatureing  0x02   //正在转换温度
#define SCPressure       0x03	  //开始气压转换
#define CPressureing     0x04	  //正在转换气压
/*********************************************
C1  压力灵敏度 SENS|T1
C2  压力补偿  OFF|T1
C3	温度压力灵敏度系数 TCS
C4	温度系数的压力补偿 TCO
C5	参考温度 T|REF
C6 	温度系数的温度 TEMPSENS
*********************************************/
uint32_t  Cal_C[7];	 //用于存放PROM中的6组数据1-6
double OFF_;
float Aux;
float Altitude;
uint64_t dT,TEMP;//dT 实际和参考温度之间的差异;TEMP 实际温度	
#define SAMPLE_NUM 10//大气压平滑滤波
/*
OFF 实际温度补偿
SENS 实际温度灵敏度
*/
uint64_t OFf,SENS;
uint32_t D1_Pres,D2_Temp;	// 数字压力值,数字温度值
uint32_t Pressure,Pressure_old,qqp;				//大气压
uint32_t TEMP2,T2,OFF2,SENS2;	//温度校验值
uint32_t Pres_BUFFER[20];
uint32_t Temp_BUFFER[100];

double Pressure_Filterbuffer[SAMPLE_NUM],IPressure,DPressure,Pressure_error,Pressure_out;
double Pressure_new,Pressure_last;
u8 lockpressure=200;//锁定当前大气压

float Baro_Height_Source,baro_dis_old,baro_dis_delta;
s8 baro_start_f;
/*******************************************************************************
  * @函数名称	MS561101BA_RESET
  * @函数说明   复位MS5611
  * @输入参数   无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
void MS561101BA_RESET(void)
{
  I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(MS561101BA_RST);
	I2C_WaitAck();
	I2C_Stop();
}
/*******************************************************************************
  * @函数名称	MS5611_init
  * @函数说明   初始化5611
  * @输入参数  	无
  * @输出参数   无
  * @返回参数   无
*******************************************************************************/
u8 MS5611_init(void)
 {	 
   uint8_t d1,d2,i;
   MS561101BA_RESET();	 // Reset Device
	 delay_ms(20);  
	 for(i=1;i<=6;i++)
	 {
			I2C_Start();
			I2C_SendByte(MS561101BA_SlaveAddress);
			I2C_WaitAck();
			I2C_SendByte((MS561101BA_PROM_RD+i*2));
			I2C_WaitAck();
			I2C_Stop();
			delay_ms(1);

			I2C_Start();
			I2C_SendByte(MS561101BA_SlaveAddress+1);
			I2C_WaitAck();
			d1=I2C_RadeByte();
			I2C_Ack();
			d2=I2C_RadeByte();
			I2C_NoAck();
			I2C_Stop();

			I2C_delay();
			Cal_C[i]=((uint16_t)d1<<8)|d2;
			delay_ms(10);
	 }
   if (Cal_C[0]!=0)
		 return 1;
	 else
     return 0; 
 }

void MS561101BA_startConversion(uint8_t command) 
{	
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(command);
  I2C_WaitAck();
	I2C_Stop();
}

unsigned long MS561101BA_getConversion(void) 
{
	unsigned long conversion = 0;
	uint8_t conv1,conv2,conv3; 
	
	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress);
	I2C_WaitAck();
	I2C_SendByte(0);
	I2C_WaitAck();
  I2C_Stop();

	I2C_Start();
	I2C_SendByte(MS561101BA_SlaveAddress+1);//+1是读
	I2C_WaitAck();
	conv1=I2C_RadeByte();
	I2C_Ack();
	conv2=I2C_RadeByte();
	I2C_Ack();
	conv3=I2C_RadeByte();

	I2C_NoAck();
	I2C_Stop();

	conversion=conv1*65535+conv2*256+conv3;
	return conversion;
}

/***********************************************
  * @brief  读取温度转换
  * @param  None
  * @retval None
************************************************/
void MS561101BA_getTemperature(void)
{  
  static u8 p=0;
	uint32_t sum=0,max=0,min=200000000;
	
	Temp_BUFFER[p] = MS561101BA_getConversion();
	p++;
	if(p==100) p=0;
	for(u8 i=0;i<100;i++) 
  {
		if(Temp_BUFFER[i] > max)  max = Temp_BUFFER[i];
		else if(Temp_BUFFER[i] < min)  min = Temp_BUFFER[i];
		sum += Temp_BUFFER[i];
	}	
	D2_Temp =(sum -  max -min)/98;
	dT=D2_Temp - (((uint32_t)Cal_C[5])<<8);
	TEMP=2000+dT*((uint32_t)Cal_C[6])/8388608;
}

/***********************************************
  * @brief  读取气压
  * @param  None
  * @retval None
************************************************/
void MS561101BA_getPressure(void)
{
	uint32_t sum=0,max=0,min=200000;
	uint32_t Altitude_sum=0,Altitude_max=0,Altitude_min=200000;
	static u8 p=0,q=0;
	static u16 inum;
//	float height_temp;
	D1_Pres= MS561101BA_getConversion();
	I2C_delay();
	OFF_=(uint32_t)Cal_C[2]*65536+((uint32_t)Cal_C[4]*dT)/128;
	SENS=(uint32_t)Cal_C[1]*32768+((uint32_t)Cal_C[3]*dT)/256;

	if(TEMP<2000)
	{
		Aux = TEMP*TEMP;
		OFF2 = 2.5*Aux;
		SENS2 = 1.25*Aux;
		TEMP = TEMP - TEMP2;
		OFF_ = OFF_ - OFF2;
		SENS = SENS - SENS2;	
	}
  Pres_BUFFER[p] = qqp =((D1_Pres*SENS/2097152-OFF_)/32768);
	p++;
	if(p==20) p=0;
	for(u8 i=0;i<20;i++) 
  {
		if(Pres_BUFFER[i] > max)  max = Pres_BUFFER[i];
		else if(Pres_BUFFER[i] < min)  min = Pres_BUFFER[i];
		sum +=Pres_BUFFER[i];
	}	
	Pressure=(sum -  max -min)/18;
	if(Pressure>150000)
		Pressure=100000;//防止初始化时的值很大
	baro_height=(Pressure_groud-Pressure)*85;//经验计算，近似高度，单位mm
	if(baro_height < -200) 
		baro_height= - 100;
	
	Baro_Height_Source=(Pressure_groud-qqp)*85;//经验计算，近似高度，单位mm
	if(Baro_Height_Source < -100) 
		Baro_Height_Source= - 100;
	
	if(inum<300)//等待气压稳定后自动读取零米时的气压值
	{
		inum++;Pressure_groud=Pressure;//当前气压值保存为地面气压
	}
	
	//Altitude_BUFFER[q] = zzz 
	//Altitude = 4433000.0 * (1 - pow((Pressure / Pressure_groud), 0.1903));
//	q++;
//	if(q==50) q=0;
//	for(u8 i=0;i<50;i++) 
//  {
//		if(Altitude_BUFFER[i] > Altitude_max)  Altitude_max = Altitude_BUFFER[i];
//		else if(Altitude_BUFFER[i] < Altitude_min)  Altitude_min = Altitude_BUFFER[i];
//		Altitude_sum +=Altitude_BUFFER[i];
//	}	
	//Altitude=(Altitude_sum -  Altitude_max - Altitude_min)/48;
	
	baro_start_f=1;
}
/***********************************************
  * @brief  得到高度
  * @param  None
  * @retval None由于测量时间为最多9ms，保险点隔10ms执行一次这个函数更新数据
************************************************/
void Get_High(void)//OSR为压强转换速率
{
	static u8 Now_doing = SCTemperature;
	static u8 flag=0;
	switch(Now_doing)
	 {
		 case SCTemperature: 
		 {		
			 if(flag)    MS561101BA_getPressure();			 
			 MS561101BA_startConversion(MS561101BA_D2_OSR_4096);  
			 Now_doing = CTemperatureing;//切换到下一个状态，温度与压力轮着测量。
		 }
		 break;
		 case CTemperatureing: 
		 {		
			 MS561101BA_getTemperature();
			 MS561101BA_startConversion(MS561101BA_D1_OSR_4096); 
			 flag = 1;
			 Now_doing = SCTemperature;
		 }
		 break;
     default: 
		 {
			 Now_doing = SCTemperature; //出错了 重新开始
			 flag=0;
		 }
 		 break;			 
	 }
}


//float MS561101BA_get_altitude(void)
//{
//  static 
//	
//	Altitude = 4433000.0 * (1 - pow((Pressure / Pressure_groud), 0.1903));
//	//Altitude = Altitude + Alt_Offset_cm ;  
//	//MS561101BA_NewAlt(Altitude);
//	//Altitude = MS561101BA_getAvg(Alt_buffer,MOVAVG_SIZE);
//	return (Altitude);
//}
