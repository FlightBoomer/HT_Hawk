/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MultiRotor_radio.c
 * 描述     ：串口通信 
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "board_config.h"
#include "MultiRotor_radio.h"
#include "MultiRotor_control.h"

int i,F,sw; 

unsigned char HtoEs_OutPut_Buffer[64] = {0};	   //串口发送缓冲区
unsigned char HtoEs_test[64]={1,2,3,4,5,6,7,8,9,10};
unsigned int CHK_SUM;  //校验和
extern s16 Moto_duty[MOTOR_NUM];
extern int16_t magdata[6];
extern fp32 yawangle;
extern vu16 US100_Alt_Temp;
extern u8 TxBuffer[256],Rx_Buf[256],TxCounter,count,Rxcounter,count_rx;
u8 Flag_Uart_Send;
u8 Flag_Request_Data=0;
u8 Flag_HT_GCS_Link=0;
///////////////////////////////////////////////////////////////////
// 15通道数据

float CH1_data  = 0;
float CH2_data  = 0;
float CH3_data  = 0;
float CH4_data  = 0;
float CH5_data  = 0;
float CH6_data  = 0;
float CH7_data  = 0;  //15通道独立数据
float CH8_data  = 0;
float CH9_data  = 0;
float CH10_data = 0;
float CH11_data = 0;
float CH12_data = 0;
float CH13_data = 0;
float CH14_data = 0;
float CH15_data = 0;

///////////////////////////////////////////////////////////////////
//  GPS 数据

float Longitude_val;  //经度数值
float Latitude_Val ;  //纬度数值

float Altitude_Val;  //高度数值
float Dir_Val;       //方位数值
float SPD_Val;       //速度数值

unsigned char Satellite_Val;   //卫星个数
unsigned int  Voltage_Val;     //电池电压
unsigned int  Temperture_Val;  //温度数值

unsigned char Longitude_WE; //标志经度方向，true=W；false=E；
unsigned char Latitude_NS;  //标志纬度方向，true=N；false=S；
unsigned char Location_Sta; //定位状态标识

///////////////////////////////////////////////////////////////////
// PID 参数

int Pitch_PID_P;
int Pitch_PID_I;
int Pitch_PID_D;

int Roll_PID_P;
int Roll_PID_I;
int Roll_PID_D;

int Yaw_PID_P;
int Yaw_PID_I;
int Yaw_PID_D;

int Alt_PID_P;
int Alt_PID_I;
int Alt_PID_D;

int Pos_PID_P;
int Pos_PID_I;
int Pos_PID_D;

//target:目标单精度数据
//buf:待写入数组
//beg:指定从数组第几个元素开始写入
void Float2Byte(float *target,unsigned char *buf,unsigned char beg)
{
    unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[beg]   = point[0];
    buf[beg+1] = point[1];
    buf[beg+2] = point[2];
    buf[beg+3] = point[3];
}

//生成飞控固件版本数据帧
unsigned char HtoEs_Version_Data_Generate(void)
{
	  unsigned char i,Count=0;
		vs16 _temp;
	
	  HtoEs_OutPut_Buffer[Count++] = 0xFE; //起始帧
	  HtoEs_OutPut_Buffer[Count++] = 0x07; //帧长度
		HtoEs_OutPut_Buffer[Count++] = 0x08; //功能码

		HtoEs_OutPut_Buffer[Count++]=0x01;//Hawk 0x01 / Hawk A 0x02 / PPM 0x03
			
		_temp = (unsigned int)(VERSION_USART);   
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < Count; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //计算校验值
		
	  return Count; 
}

//生成独立通道数据帧
unsigned char HtoEs_Chart_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x3F; //帧长度 63字节
		HtoEs_OutPut_Buffer[1] = 0x01; //功能码
		
	  CH1_data = (float)sensor.acc.origin.x/100;
	  CH2_data = (float)sensor.acc.origin.y/100;
	  CH3_data = (float)sensor.acc.origin.z/100;
	  CH4_data = (float)sensor.gyro.origin.x;
	  CH5_data = (float)sensor.gyro.origin.y;
	  CH6_data = (float)sensor.gyro.origin.z;
	  CH7_data = (float)Moto_duty[0];//
	  CH8_data = (float)Moto_duty[1];//
	  CH9_data = (float)Moto_duty[2];//
	  CH10_data = (float)Moto_duty[3];//
	  CH11_data = (float)magdata[0];//
	  CH12_data = (float)magdata[1];//
		CH13_data = (float)magdata[2];
		CH14_data = (float)baro_height;//
		CH15_data = (float)ultra_dis_lpf;//
	
		Float2Byte(&CH1_data ,HtoEs_OutPut_Buffer,2);
		Float2Byte(&CH2_data ,HtoEs_OutPut_Buffer,6);
		Float2Byte(&CH3_data ,HtoEs_OutPut_Buffer,10);
		Float2Byte(&CH4_data ,HtoEs_OutPut_Buffer,14);
		Float2Byte(&CH5_data ,HtoEs_OutPut_Buffer,18);
		Float2Byte(&CH6_data ,HtoEs_OutPut_Buffer,22);
		Float2Byte(&CH7_data ,HtoEs_OutPut_Buffer,26);
		Float2Byte(&CH8_data ,HtoEs_OutPut_Buffer,30);
		Float2Byte(&CH9_data ,HtoEs_OutPut_Buffer,34);
		Float2Byte(&CH10_data,HtoEs_OutPut_Buffer,38);
		Float2Byte(&CH11_data,HtoEs_OutPut_Buffer,42);
		Float2Byte(&CH12_data,HtoEs_OutPut_Buffer,46);
		Float2Byte(&CH13_data,HtoEs_OutPut_Buffer,50);
		Float2Byte(&CH14_data,HtoEs_OutPut_Buffer,54);
		Float2Byte(&CH15_data,HtoEs_OutPut_Buffer,58);
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 62; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[62] = CHK_SUM % 2; //计算校验值
 
	  return 63; 
}

//生成GPS数据帧
unsigned char HtoEs_GPS_Data_Generate(void)
{
	  unsigned char i;
	
	  HtoEs_OutPut_Buffer[0] = 0x1D; //帧长度 29字节
		HtoEs_OutPut_Buffer[1] = 0x02; //功能码
	
	  Float2Byte(&Longitude_val ,HtoEs_OutPut_Buffer,2);  //经度
	  Float2Byte(&Latitude_Val  ,HtoEs_OutPut_Buffer,6);  //纬度
	  Float2Byte(&Altitude_Val  ,HtoEs_OutPut_Buffer,10); //高度
	  Float2Byte(&Dir_Val ,HtoEs_OutPut_Buffer,14);       //方位角
	  Float2Byte(&SPD_Val ,HtoEs_OutPut_Buffer,18);       //速度
	
	  HtoEs_OutPut_Buffer[22] = (Voltage_Val & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[23] = (Voltage_Val & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[24] = (Temperture_Val & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[25] = (Temperture_Val & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[26] = Satellite_Val; //卫星个数
	
	//============================================================================
	  
		HtoEs_OutPut_Buffer[27] = 0; //先将状态标识清除
		
		if( Location_Sta ) //定位模式
			HtoEs_OutPut_Buffer[27] |= 0x01; //置高
		else               //导航模式
			HtoEs_OutPut_Buffer[27] &= 0xFE; //清零
		
		
		if( Longitude_WE == 'W' )  //经度方向
			HtoEs_OutPut_Buffer[27] |= 0x02;  //W,西经
		else if( Longitude_WE == 'E' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFD;  //E,东经
 
		if( Latitude_NS == 'N' )  //纬度方向
			HtoEs_OutPut_Buffer[27] |= 0x04;  //N,北纬
		else if( Latitude_NS == 'S' ) 
			HtoEs_OutPut_Buffer[27] &= 0xFB;  //S,南纬
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 28; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[28] = CHK_SUM % 2; //计算校验值
	
	  return 29; 
}

//生成姿态数据帧
unsigned char HtoEs_Attitude_Data_Generate(void)
{
	  unsigned char i,Count=0;
	  vs16 _temp;
	
	  HtoEs_OutPut_Buffer[Count++] = 0xFE; //起始帧
	  HtoEs_OutPut_Buffer[Count++] = 0x0C; //帧长度
		HtoEs_OutPut_Buffer[Count++] = 0x01; //功能码

		_temp = (int)(IMU.Roll*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
		
		_temp = (int)(-IMU.Pitch*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
		
		_temp = (int)((IMU.Yaw+180)*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
		
		_temp = (int)(baro_height/10);//ultra_dis_lpf/10*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
	  
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < Count; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //计算校验值
		
	  return Count; 
}

//生成HUD显示数据帧
unsigned char HtoEs_HUD_Data_Generate(void)
{
	  unsigned char i,Count=0;
		vs16 _temp;
	
	  HtoEs_OutPut_Buffer[Count++] = 0xFE; //起始帧
	  HtoEs_OutPut_Buffer[Count++] = 0x0B; //帧长度
		HtoEs_OutPut_Buffer[Count++] = 0x02; //功能码

		HtoEs_OutPut_Buffer[Count++]=flag.HUDMode;
		HtoEs_OutPut_Buffer[Count++]=flag.ARMED;
		HtoEs_OutPut_Buffer[Count++]=0x01;//flag.ControlMode;
		
		_temp = (unsigned int)(0*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
		
		_temp = (unsigned int)(0*100);
		HtoEs_OutPut_Buffer[Count++]=BYTE1(_temp);
		HtoEs_OutPut_Buffer[Count++]=BYTE0(_temp);
	  
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < Count; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //计算校验值
		
	  return Count; 
}

//生成RC通道数据帧
unsigned char HtoEs_RC_Data_Generate(void)
{
	  unsigned char i,Count=0;
	
	  HtoEs_OutPut_Buffer[Count++] = 0xFE; //起始帧
	  HtoEs_OutPut_Buffer[Count++] = 0x18; //帧长度
		HtoEs_OutPut_Buffer[Count++] = 0x03; //功能码
	 
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[0]); //取高8位ROLL
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[0]);      //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[1]); //取高8位PITCH
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[1]);      //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[2]); //取高8位THRrc_data[3]
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[2]);      //取低8位rc_data[3]
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[3]); //取高8位YAW
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[3]);      //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[4]); //取高8位CH5
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[4]);      //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[5]); //取高8位CH6
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[5]);      //取低8位
		
		HtoEs_OutPut_Buffer[Count++] = BYTE1(RC_Pwm_In[6]); //取高8位超声波
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(RC_Pwm_In[6]);      //取低8位
		
		HtoEs_OutPut_Buffer[Count++] = BYTE1(US100_Alt_Temp); //取高8位CH7
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(US100_Alt_Temp);      //取低8位
		
		HtoEs_OutPut_Buffer[Count++] =0;// (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[Count++] =0;// (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[Count++] =0;// (RC_Data.rc_data[0] & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[Count++] =0;// (RC_Data.rc_data[0] & 0x00FF) ;      //取低8位
	 
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < Count; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //计算校验值
	
	  return Count; 
}

//生成电机通道数据帧
unsigned char HtoEs_MOTO_Data_Generate(void)
{
	  unsigned char i,Count=0;
	
	  HtoEs_OutPut_Buffer[Count++] = 0xFE; //起始帧
	  HtoEs_OutPut_Buffer[Count++] = 0x14; //帧长度
		HtoEs_OutPut_Buffer[Count++] = 0x06; //功能码
	 
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(Moto_duty[0]); //取高8位
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(Moto_duty[0]); //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(Moto_duty[1]); //取高8位
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(Moto_duty[1]); //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(Moto_duty[2]); //取高8位
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(Moto_duty[2]); //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = BYTE1(Moto_duty[3]); //取高8位
	  HtoEs_OutPut_Buffer[Count++] = BYTE0(Moto_duty[3]); //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = 0; //取高8位
	  HtoEs_OutPut_Buffer[Count++] = 0; //取低8位
	
	  HtoEs_OutPut_Buffer[Count++] = 0; //取高8位
	  HtoEs_OutPut_Buffer[Count++] = 0; //取低8位
	
		HtoEs_OutPut_Buffer[Count++] = 0; //取高8位
	  HtoEs_OutPut_Buffer[Count++] = 0; //取低8位
		
		HtoEs_OutPut_Buffer[Count++] = 0; //取高8位
	  HtoEs_OutPut_Buffer[Count++] = 0; //取低8位
	 
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < Count; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[Count++] = CHK_SUM % 2; //计算校验值
	
	  return Count; 
}

//生成PID数据帧
unsigned char HtoEs_PID_Data_Generate(void)
{
	  unsigned char i;
	
	  Pitch_PID_P = ctrl.pitch.core.kp * 100;
	  Pitch_PID_I = ctrl.pitch.core.ki * 1000;
	  Pitch_PID_D = ctrl.pitch.core.kd * 100;
	  Roll_PID_P  = ctrl.roll.core.kp * 100;
	  Roll_PID_I  = ctrl.roll.core.ki * 1000;
	  Roll_PID_D  = ctrl.roll.core.kd * 100;
	  Yaw_PID_P   = ctrl.yaw.core.kp * 100;
	  Yaw_PID_I   = ctrl.yaw.core.ki * 1000;
	  Yaw_PID_D   = ctrl.yaw.core.kd * 100;
	  Alt_PID_P   = baro_wz_speed_pid.kp*100;
	  Alt_PID_I   = baro_wz_speed_pid.ki*1000;
	  Alt_PID_D   = baro_wz_speed_pid.kd*100;
	  HtoEs_OutPut_Buffer[0] = 0x21; //帧长度 15字节
		HtoEs_OutPut_Buffer[1] = 0x05; //功能码
	
	  HtoEs_OutPut_Buffer[2] = (Pitch_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[3] = (Pitch_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[4] = (Pitch_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[5] = (Pitch_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[6] = (Pitch_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[7] = (Pitch_PID_D & 0x00FF) ;      //取低8位
	
	  HtoEs_OutPut_Buffer[8] = (Roll_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[9] = (Roll_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[10] = (Roll_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[11] = (Roll_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[12] = (Roll_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[13] = (Roll_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[14] = (Yaw_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[15] = (Yaw_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[16] = (Yaw_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[17] = (Yaw_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[18] = (Yaw_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[19] = (Yaw_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[20] = (Alt_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[21] = (Alt_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[22] = (Alt_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[23] = (Alt_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[24] = (Alt_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[25] = (Alt_PID_D & 0x00FF) ;      //取低8位
		
		HtoEs_OutPut_Buffer[26] = (Pos_PID_P & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[27] = (Pos_PID_P & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[28] = (Pos_PID_I & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[29] = (Pos_PID_I & 0x00FF) ;      //取低8位
	  HtoEs_OutPut_Buffer[30] = (Pos_PID_D & 0xFF00) >> 8 ; //取高8位
	  HtoEs_OutPut_Buffer[31] = (Pos_PID_D & 0x00FF) ;      //取低8位
		
	//============================================================================	
		
		CHK_SUM =0; 
	
	  for(i = 0 ; i < 32; i++)  //计算和
			CHK_SUM += HtoEs_OutPut_Buffer[i];
		
		HtoEs_OutPut_Buffer[32] = CHK_SUM % 2; //计算校验值
	
	  return 33; 
}

// //生成传感器数据帧
// unsigned char HtoEs_Senosrs_Data_Generate(void)
// {
// 	  unsigned char i;
// 	
// 	  HtoEs_OutPut_Buffer[0] = 0x27; //帧长度 15字节
// 		HtoEs_OutPut_Buffer[1] = 0x06; //功能码
// 	
// 	  Float2Byte(&Gyro_X  ,HtoEs_OutPut_Buffer,2);
// 		Float2Byte(&Gyro_Y  ,HtoEs_OutPut_Buffer,6);
// 		Float2Byte(&Gyro_Z  ,HtoEs_OutPut_Buffer,10);
// 		Float2Byte(&Accel_X ,HtoEs_OutPut_Buffer,14);
// 		Float2Byte(&Accel_Y ,HtoEs_OutPut_Buffer,18);
// 		Float2Byte(&Accel_Z ,HtoEs_OutPut_Buffer,22);
// 		Float2Byte(&Mag_X   ,HtoEs_OutPut_Buffer,26);
// 		Float2Byte(&Mag_Y   ,HtoEs_OutPut_Buffer,30);
// 		Float2Byte(&Mag_Z   ,HtoEs_OutPut_Buffer,34);
// 	
// //============================================================================	
// 		
// 		CHK_SUM =0; 
// 	
// 	  for(i = 0 ; i < 38; i++)  //计算和
// 			CHK_SUM += HtoEs_OutPut_Buffer[i];
// 		
// 		HtoEs_OutPut_Buffer[38] = CHK_SUM % 255; //计算校验值	
// 	
// 	  return 39; 
// }

s8 fg=10;

void HT_GCS_Link(void)
{
  u8 senddataflag;
	if(Flag_Request_Data){
		F = HtoEs_Version_Data_Generate();	
		senddataflag=1;
		Flag_Request_Data=0;
		Flag_HT_GCS_Link=1;
	}
  else if(Flag_HT_GCS_Link){	
		switch(sw) //循环发送各模块数据
		{
				case 1: F = HtoEs_RC_Data_Generate();       senddataflag=1;break;  //测试RC通道，返回需发送字节数		 
				case 2: F = HtoEs_Attitude_Data_Generate(); senddataflag=1;break;  //测试姿态,返回需发送字节数	
				case 3: F = HtoEs_HUD_Data_Generate();      senddataflag=1;break;  //测试HUD显示，返回需发送字节数
				case 4: F = HtoEs_MOTO_Data_Generate();     senddataflag=1;break;  //测试电机显示，返回需发送字节数
				//case 3: if(fg ==10 ) {F = HtoEs_PID_Data_Generate(); senddataflag=1;}
	//										if(fg<10 && fg >0){
	//											fg--; 
	//											F = HtoEs_PID_Data_Generate();       //测试PID参数，返回需发送字节数
	//											senddataflag=1;
	//										}break;
				//case 4:F = HtoEs_Chart_Data_Generate(); senddataflag=1; break;  //测试独立通道,返回需发送字节数										
				//case 5: F = HtoEs_GPS_Data_Generate(); 		  break;     //测试GPS,返回需发送字节数
				//case 6: F = HtoEs_Senosrs_Data_Generate();  break;  //测试传感器标定，返回需发送字节数					
				case 7: sw = 0; break;
				default: break;
		}
		sw++;
	}
	 
	if(senddataflag==1){
		senddataflag=0;
		usb_SendDataToHost(&HtoEs_OutPut_Buffer[0], F);//USB虚拟串口发送数据，不需要时关了它	
		Usart1_Send(&HtoEs_OutPut_Buffer[0] ,F);
	}
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : UsbCmdPro
**功能 : USB数据接收
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void UsbCmdPro(void)
{
  u8 ucData,ucNum;
  volatile static u16 aCmdBuf[32];
	static u8 aCmdBuf1[32];
	static u16 usPos;
	static u16 Free_heart=0;
	
	// 空闲心跳
	Free_heart++;  
	if(Free_heart>=60) {
		usPos = 0;
		Free_heart = 60;
	 for(u8 i=0;i<32;i++)
	 {
		 aCmdBuf[i] =0;
		 aCmdBuf1[i]=0;
	 } 
  }
	
	if(Free_heart==3)//字符间隔超时认为是一帧结束
	{
	// 数据解析
	#ifdef HT_HAWK 
		if(fg!=10) Data_Parser(aCmdBuf);
	#elif defined ANO
		if(fg!=10) 	ANO_DT_Data_Receive_Anl(&aCmdBuf1[0],aCmdBuf1[3]+5);//使用匿名上位机时		
	#endif	
	usPos = 0;//一帧结束，开始下一帧	
	 for(u8 i=0;i<32;i++)
	 {
		 aCmdBuf[i] =0;
		 aCmdBuf1[i]=0;	 
	 }	
	}
	// 从USB口读取一个字节 ucNum存放读到的字节个数 
	ucData = usb_GetRxByte(&ucNum);	
	
	// 没有接收到输出 退出
	if (ucNum == 0)		return;
	
	Free_heart=0;
	
 	// 接收到的数据放入缓存
	aCmdBuf1[usPos] = ucData;
 	aCmdBuf[usPos++] = ucData;
  
	if(usPos>=31) usPos=31;
	fg=5;
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : usart_data_RX
**功能 : 串口数据接收
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void usart_data_RX(void)
{
		volatile static u16 aCmdBuf[32];
		static u8 aCmdBuf1[32];
		static u16 usPos;
		static u16 Free_heart=0;
		// 空闲心跳
		Free_heart++;  
		if(Free_heart>=60) {
			usPos = 0;
			Free_heart = 60;
		 for(u8 i=0;i<32;i++)
		 {
			 aCmdBuf[i] =0;
			 aCmdBuf1[i]=0;
		 } 
		}

		if(Free_heart==3)//字符间隔超时认为是一帧结束
		{
			// 数据解析
			#ifdef HT_HAWK 
				if(fg!=10) Data_Parser(aCmdBuf);
			#elif defined ANO
				if(fg!=10) 	ANO_DT_Data_Receive_Anl(&aCmdBuf1[0],aCmdBuf1[3]+5);//使用匿名上位机时		
			#endif 	

			usPos = 0;//一帧结束，开始下一帧	
			 for(u8 i=0;i<32;i++)
			 {aCmdBuf[i] =0;
				 aCmdBuf1[i]=0;
			 }	
		}		
		if(count_rx!=Rxcounter)
		{ aCmdBuf1[usPos] = Rx_Buf[count_rx];
			aCmdBuf[usPos++] = Rx_Buf[count_rx++];	// 接收到的数据放入缓存	
		}
		// 没有接收到输出 退出
		else	return;
		fg=5;
		Free_heart=0;
		if(usPos>=31) usPos=31;
}

void Usart1_Send(unsigned char *DataToSend ,u8 data_num)
{
/*下面是采用串口移位中断方式发送，需在串口中断函数里面装载下一个数据，不推荐*/	
//  u8 i;
//	for(i=0;i<data_num;i++)
//	{
//		TxBuffer[count++] = *(DataToSend+i);
//	}
//	if(!(USART1->CR1 & USART_CR1_TXEIE))
//	{
//		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
//	}
//
	
/*使用串口DMA方式发送，节约时间又稳定*/
	if(Flag_Uart_Send==0)
	{
		DMA_Cmd(DMA1_Channel4,DISABLE); 
		DMA1_Channel4->CMAR = (u32)DataToSend;
		Flag_Uart_Send=1;
	  DMA1_Channel4->CNDTR =data_num;
		DMA_Cmd(DMA1_Channel4,ENABLE); 
	}
}
