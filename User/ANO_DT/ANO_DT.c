/******************** (C) COPYRIGHT 2014 ANO Tech ********************************
  * 作者   ：移植的匿名通信协议
 * 文件名  ：ANO_DT.c
 * 描述    ：数据传输
**********************************************************************************/
#include "ANO_DT.h"
#include "board_config.h"

dt_flag_t f;					//需要发送数据的标志
u8 data_to_send[96];	//发送数据缓存
extern u16 Moto_duty[MOTOR_NUM];
float userdata1[13];
float userdata2[13];
float realtime;//*2.5ms
float realdata;
/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每10ms调用一次
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 userdata_cnt 	= 2;
	static u8 senser_cnt1 	= 13;
	static u8 senser_cnt2 	= 15;
	static u8 status_cnt 	= 9;
	static u8 rcdata_cnt 	= 23;
	static u8 motopwm_cnt	= 27;
	static u8 power_cnt		=	199;

	if((cnt % userdata_cnt) == (userdata_cnt-1))//还有个userdata2根据需要可以设置
	{
		f.send_userdata1=1;
	  f.send_userdata2=1;
	}
	if((cnt % senser_cnt1) == (senser_cnt1-1))//
		f.send_senser1=1;
		
	if((cnt % senser_cnt2) == (senser_cnt2-1))//
		f.send_senser2=1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-1))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		f.send_power = 1;		
	cnt++;
	////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_pid1)
	{
		f.send_pid1 = 0;
		ANO_DT_Send_PID(1,ctrl.pitch.core.kp * 100,ctrl.pitch.core.ki * 1000,ctrl.pitch.core.kd * 100,
											ctrl.roll.core.kp * 100,ctrl.roll.core.ki * 1000,ctrl.roll.core.kd * 100,
											ctrl.yaw.core.kp * 100,ctrl.yaw.core.ki * 1000,ctrl.yaw.core.kd * 100);
	}	
///////////////////////////////////////////////////////////////////////////////////////
  else if(f.send_pid2)
	{
		f.send_pid2 = 0;
		ANO_DT_Send_PID(2,ctrl.pitch.shell.kp * 100,ctrl.pitch.shell.ki * 1000,ctrl.pitch.shell.kd * 100,
											ctrl.roll.shell.kp * 100,ctrl.roll.shell.ki * 1000,ctrl.roll.shell.kd * 100,
											ctrl.yaw.shell.kp * 100,ctrl.yaw.shell.ki * 1000,ctrl.yaw.shell.kd * 100);
	}
///////////////////////////////////////////////////////////////////////////////////////
  else	if(f.send_pid3)
	{
		f.send_pid3 = 0;
		ANO_DT_Send_PID(3,ultra_wz_speed_pid.kp * 100,ultra_wz_speed_pid.ki * 1000,ultra_wz_speed_pid.kd * 100,
											ultra_pid.kp * 100,ultra_pid.ki * 1000,ultra_pid.kd * 100,
											baro_pid.kp * 100,baro_pid.ki * 1000,baro_pid.kd * 100);
	}
///////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_pid4)
	{
		f.send_pid4 = 0;
		ANO_DT_Send_PID(4,baro_wz_speed_pid.kp * 100,baro_wz_speed_pid.ki * 1000,baro_wz_speed_pid.kd * 100,0,0,0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}

/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(ADC_ConvertedValue*10/11,0);//电流电压,注意电压1K，10K分压了。
	}
/////////////////////////////////////////////////////////////////////////////////////	
  else 	if(f.send_motopwm)
	{
		f.send_motopwm = 0;
		ANO_DT_Send_MotoPWM(Moto_duty[0],Moto_duty[1],Moto_duty[2],Moto_duty[3],0,0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		ANO_DT_Send_RCData(RC_Pwm_In[2],RC_Pwm_In[3],RC_Pwm_In[0],RC_Pwm_In[1],RC_Pwm_In[4],RC_Pwm_In[5],RC_Pwm_In[6],0,0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(AngE.Roll,AngE.Pitch,AngE.Yaw,ultra_dis_lpf/10,flag.FlightMode,flag.ARMED);
	}	
/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_senser1)
	{
		f.send_senser1 = 0;
		ANO_DT_Send_Senser(sensor.acc.origin.x,sensor.acc.origin.y,sensor.acc.origin.z,
												sensor.gyro.origin.x,sensor.gyro.origin.y,sensor.gyro.origin.z,
												MAG[0],MAG[1],MAG[2],Pressure);	
	}		
	/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_senser2)
	{
		f.send_senser2 = 0;
    ANO_DT_Send_Senser2(baro_height,ultra_dis_lpf/10);//		
	}		
	/////////////////////////////////////////////////////////////////////////////////////
  else 	if(f.send_userdata1)
	{
		f.send_userdata1=0;
		ANO_DT_Send_userdata(1,userdata1[0],userdata1[1],userdata1[2],userdata1[3],userdata1[4],userdata1[5],userdata1[6],userdata1[7],userdata1[8]
		,userdata1[9],userdata1[10],userdata1[11]);
	}
  else if(f.send_userdata2)
	{
		f.send_userdata2=0;
		ANO_DT_Send_userdata(2,realtime,userdata2[0],userdata2[1],userdata2[2],userdata2[3],userdata2[4],userdata2[5],userdata2[6],userdata2[7],userdata2[8]
		,userdata2[9],userdata2[10]);
	}	
	/////////////////////////////////////////////////////////////////////////////////////
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void ANO_DT_Send_Data(u8 *dataToSend , u8 length)
{

 uint8 static numi,numj;

 // temp=NRF24L01_Txframes(dataToSend,length);//使用NRF发送
    Usart1_Send(dataToSend ,length);
		if(numi>=10)//	
		{
				if(numj>=1)
				{
					//Dis_Float(3,80,3,0);//没啥，就是数字跳动用于显示正在通信而已
					numj=0;
				}
				else 
        {
				  numj++;
					//Dis_Float(3,80,1,0);
				}
				numi=0;
		}
		else
			numi++;
}

static void ANO_DT_Send_Check(u8 head, u8 check_sum)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xEF;
	data_to_send[3]=2;
	data_to_send[4]=head;
	data_to_send[5]=check_sum;
		
	u8 sum = 0;
	for(u8 i=0;i<6;i++)
		sum += data_to_send[i];
	data_to_send[6]=sum;
	DMA_Cmd(DMA1_Channel4,DISABLE);//先停止其他DMA数据发送，避免混淆了check的重要帧
	delay_ms(10);//延时一点，以确保上一帧数据已经发完，串口链路是空的
	Flag_Uart_Send=0;//置0以确保可以下次启动DMA
	ANO_DT_Send_Data(data_to_send, 7);
	delay_ms(5);//延时避免其他数据混入	
}


/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能

void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	//功能字为01时
	if(*(data_buf+2)==0X01)
	{
		if(*(data_buf+4)==0X01)
			flag.calibratingA = 1;
		if(*(data_buf+4)==0X02)
     flag.calibratingG = 1;
		if(*(data_buf+4)==0X03)
		{
		flag.calibratingA = 1;
		flag.calibratingG = 1;
		}
		if(*(data_buf+4)==0X04)
		{
		flag.calibratingM = 1;		
		}
	}
	
	//功能字为02时	
	if(*(data_buf+2)==0X02)
	{
		if(*(data_buf+4)==0X01)
		{
			f.send_pid1 = 1;
			f.send_pid2 = 1;
			f.send_pid3 = 1;
			f.send_pid4 = 1;
			f.send_pid5 = 1;
			f.send_pid6 = 1;
		}
		if(*(data_buf+4)==0X02)
		{
			
		}
		if(*(data_buf+4)==0XA0)		//读取版本信息
		{
			f.send_version = 1;
		}
//		if(*(data_buf+4)==0XA1)		//恢复默认参数
//		{
//			Para_ResetToFactorySetup();
//		}
	}
	//功能字为0X10时	
	if(*(data_buf+2)==0X10)								//PID1
    {
        ctrl.pitch.core.kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl.pitch.core.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl.pitch.core.kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl.roll.core.kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl.roll.core.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl.roll.core.kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl.yaw.core.kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl.yaw.core.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl.yaw.core.kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
			  flag.ParamSave = 1;
    }
    if(*(data_buf+2)==0X11)								//PID2
    {
        ctrl.pitch.shell.kp 	= 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ctrl.pitch.shell.ki 	= 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ctrl.pitch.shell.kd 	= 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ctrl.roll.shell.kp 	= 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ctrl.roll.shell.ki 	= 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ctrl.roll.shell.kd 	= 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        ctrl.yaw.shell.kp	  = 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        ctrl.yaw.shell.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        ctrl.yaw.shell.kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
			  flag.ParamSave = 1;
    }
    if(*(data_buf+2)==0X12)								//PID3
    {	
        ultra_wz_speed_pid.kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        ultra_wz_speed_pid.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        ultra_wz_speed_pid.kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
        ultra_pid.kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
        ultra_pid.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
        ultra_pid.kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
        baro_pid.kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
        baro_pid.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
        baro_pid.kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
        ANO_DT_Send_Check(*(data_buf+2),sum);
        flag.ParamSave = 1;
    }
	if(*(data_buf+2)==0X13)								//PID4
	{
        baro_wz_speed_pid.kp  = 0.01*( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
        baro_wz_speed_pid.ki  = 0.001*( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
        baro_wz_speed_pid.kd  = 0.01*( (vs16)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ultra_pid.kp = 0.01*( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ultra_pid.ki = 0.001*( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ultra_pid.kd = 0.01*( (vs16)(*(data_buf+14)<<8)|*(data_buf+15) );
//        baro_pid.kp 	= 0.01*( (vs16)(*(data_buf+16)<<8)|*(data_buf+17) );
//        baro_pid.ki 	= 0.001*( (vs16)(*(data_buf+18)<<8)|*(data_buf+19) );
//        baro_pid.kd 	= 0.01*( (vs16)(*(data_buf+20)<<8)|*(data_buf+21) );
//        ANO_DT_Send_Check(*(data_buf+2),sum);		
				ANO_DT_Send_Check(*(data_buf+2),sum);
				flag.ParamSave = 1;
	}
	if(*(data_buf+2)==0X14)								//PID5
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0X15)								//PID6
	{
		ANO_DT_Send_Check(*(data_buf+2),sum);
	}
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2 = alt;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	
	data_to_send[_cnt++] = fly_model;
	
	data_to_send[_cnt++] = armed;
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	
	_temp = a_x;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Senser2(float ALT_BAR, float ALT_CSB)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp2;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x07;
	data_to_send[_cnt++]=0;
	
	_temp2 = ALT_BAR;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
	_temp = ALT_CSB;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(thr);
	data_to_send[_cnt++]=BYTE0(thr);
	data_to_send[_cnt++]=BYTE1(yaw);
	data_to_send[_cnt++]=BYTE0(yaw);
	data_to_send[_cnt++]=BYTE1(rol);
	data_to_send[_cnt++]=BYTE0(rol);
	data_to_send[_cnt++]=BYTE1(pit);
	data_to_send[_cnt++]=BYTE0(pit);
	data_to_send[_cnt++]=BYTE1(aux1);
	data_to_send[_cnt++]=BYTE0(aux1);
	data_to_send[_cnt++]=BYTE1(aux2);
	data_to_send[_cnt++]=BYTE0(aux2);
	data_to_send[_cnt++]=BYTE1(aux3);
	data_to_send[_cnt++]=BYTE0(aux3);
	data_to_send[_cnt++]=BYTE1(aux4);
	data_to_send[_cnt++]=BYTE0(aux4);
	data_to_send[_cnt++]=BYTE1(aux5);
	data_to_send[_cnt++]=BYTE0(aux5);
	data_to_send[_cnt++]=BYTE1(aux6);
	data_to_send[_cnt++]=BYTE0(aux6);

	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Power(u16 votage, u16 current)
{
	u8 _cnt=0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;
	
	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8)
{
	u8 _cnt=0;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x06;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(m_1);
	data_to_send[_cnt++]=BYTE0(m_1);
	data_to_send[_cnt++]=BYTE1(m_2);
	data_to_send[_cnt++]=BYTE0(m_2);
	data_to_send[_cnt++]=BYTE1(m_3);
	data_to_send[_cnt++]=BYTE0(m_3);
	data_to_send[_cnt++]=BYTE1(m_4);
	data_to_send[_cnt++]=BYTE0(m_4);
	data_to_send[_cnt++]=BYTE1(m_5);
	data_to_send[_cnt++]=BYTE0(m_5);
	data_to_send[_cnt++]=BYTE1(m_6);
	data_to_send[_cnt++]=BYTE0(m_6);
	data_to_send[_cnt++]=BYTE1(m_7);
	data_to_send[_cnt++]=BYTE0(m_7);
	data_to_send[_cnt++]=BYTE1(m_8);
	data_to_send[_cnt++]=BYTE0(m_8);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x10+group-1;
	data_to_send[_cnt++]=0;
	
	
	_temp = p1_p;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p1_d ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p2_d;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_p ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_i ;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = p3_d;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}



void ANO_DT_Send_userdata(u8 group,float data1,float data2,float data3,float data4,float data5,float data6,float data7,float data8,float data9,float data10,float data11,float data12)
{
	u8 _cnt=0;
	vs16 _temp;
	vs32 _temp1;//注意32位于16位的区别,这里强行将float转换为int32
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xF0+group;
	data_to_send[_cnt++]=0;
	
	
	_temp1 = data1;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
	_temp1 = data2;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data3;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data4;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data5;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data6;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data7;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data8;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data9;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data10;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data11;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);
		_temp1 = data12;
	data_to_send[_cnt++]=BYTE3(_temp1);
	data_to_send[_cnt++]=BYTE2(_temp1);	
	data_to_send[_cnt++]=BYTE1(_temp1);
	data_to_send[_cnt++]=BYTE0(_temp1);


	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
