/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��MS5611.c
 * ����     ��MS5611��ѹ������
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "include.h"
#include "MS5611.h"
#include "math.h"

//��ѹ��״̬��
#define SCTemperature    0x01	  //��ʼ�¶�ת��
#define CTemperatureing  0x02   //����ת���¶�
#define SCPressure       0x03	  //��ʼ��ѹת��
#define CPressureing     0x04	  //����ת����ѹ
/*********************************************
C1  ѹ�������� SENS|T1
C2  ѹ������  OFF|T1
C3	�¶�ѹ��������ϵ�� TCS
C4	�¶�ϵ����ѹ������ TCO
C5	�ο��¶� T|REF
C6 	�¶�ϵ�����¶� TEMPSENS
*********************************************/
uint32_t  Cal_C[7];	 //���ڴ��PROM�е�6������1-6
double OFF_;
float Aux;
float Altitude;
uint64_t dT,TEMP;//dT ʵ�ʺͲο��¶�֮��Ĳ���;TEMP ʵ���¶�	
#define SAMPLE_NUM 10//����ѹƽ���˲�
/*
OFF ʵ���¶Ȳ���
SENS ʵ���¶�������
*/
uint64_t OFf,SENS;
uint32_t D1_Pres,D2_Temp;	// ����ѹ��ֵ,�����¶�ֵ
uint32_t Pressure,Pressure_old,qqp;				//����ѹ
uint32_t TEMP2,T2,OFF2,SENS2;	//�¶�У��ֵ
uint32_t Pres_BUFFER[20];
uint32_t Temp_BUFFER[100];

double Pressure_Filterbuffer[SAMPLE_NUM],IPressure,DPressure,Pressure_error,Pressure_out;
double Pressure_new,Pressure_last;
u8 lockpressure=200;//������ǰ����ѹ

float Baro_Height_Source,baro_dis_old,baro_dis_delta;
s8 baro_start_f;
/*******************************************************************************
  * @��������	MS561101BA_RESET
  * @����˵��   ��λMS5611
  * @�������   ��
  * @�������   ��
  * @���ز���   ��
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
  * @��������	MS5611_init
  * @����˵��   ��ʼ��5611
  * @�������  	��
  * @�������   ��
  * @���ز���   ��
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
	I2C_SendByte(MS561101BA_SlaveAddress+1);//+1�Ƕ�
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
  * @brief  ��ȡ�¶�ת��
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
  * @brief  ��ȡ��ѹ
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
		Pressure=100000;//��ֹ��ʼ��ʱ��ֵ�ܴ�
	baro_height=(Pressure_groud-Pressure)*85;//������㣬���Ƹ߶ȣ���λmm
	if(baro_height < -200) 
		baro_height= - 100;
	
	Baro_Height_Source=(Pressure_groud-qqp)*85;//������㣬���Ƹ߶ȣ���λmm
	if(Baro_Height_Source < -100) 
		Baro_Height_Source= - 100;
	
	if(inum<300)//�ȴ���ѹ�ȶ����Զ���ȡ����ʱ����ѹֵ
	{
		inum++;Pressure_groud=Pressure;//��ǰ��ѹֵ����Ϊ������ѹ
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
  * @brief  �õ��߶�
  * @param  None
  * @retval None���ڲ���ʱ��Ϊ���9ms�����յ��10msִ��һ�����������������
************************************************/
void Get_High(void)//OSRΪѹǿת������
{
	static u8 Now_doing = SCTemperature;
	static u8 flag=0;
	switch(Now_doing)
	 {
		 case SCTemperature: 
		 {		
			 if(flag)    MS561101BA_getPressure();			 
			 MS561101BA_startConversion(MS561101BA_D2_OSR_4096);  
			 Now_doing = CTemperatureing;//�л�����һ��״̬���¶���ѹ�����Ų�����
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
			 Now_doing = SCTemperature; //������ ���¿�ʼ
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
