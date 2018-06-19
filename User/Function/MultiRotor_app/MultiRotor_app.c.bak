/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MultiRotor_app.c
 * 描述     ：硬件初始化与OLED显示     
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "include.h"
#include "MultiRotor_app.h"
uint8 RXBUF[32];
//extern unsigned char HtoEs_OutPut_Buffer[63];
extern uint8 rxdata,rxdata1;
fp32 Battery_Voltage;
Flag_t flag;
u8 NRFRXOK,RXstate;
extern u16 Moto_duty[MOTOR_NUM];
extern u8 timetoconver;		
extern u8 GpsFlag;
extern float Longitude_val;
extern u8 Location_Sta;
extern GPSINFO GpsInfo;
extern u8 GpsBuffer[NMEA_COUNT_MAX];
u16 tempnum;
	
struct KALMAN_Data{
			double x_last;
			double p_last;
}Kalman_Data;

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Bootloader_Set
**功能 : BOOT相关设置
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Bootloader_Set(void)
{
 	u16 i;
	
	// 设置偏移量 
	SCB->VTOR = FLASH_BASE | FLASH_EXCURSION ; 
	
	i=0x0505;
	STMFLASH_Write(pro_FALG_ADD,&i,1);   
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Sensor_Init
**功能 : 传感器初始化
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void InitBoard(void)
{
	delay_init(72);
	Nvic_Init();
	
	/* 初始化USB设备 */
  bsp_InitUsb();//暂时先启用虚拟USB，不需要可以屏蔽
	 
	ADC1_Init();	
	OLED_Init();
	Draw_Logo();
  OLED_Print(35,6,"恒拓科技");
	I2C_INIT();
  LED_GPIO_Config();
	USART1_Config();
	TIM5_Config();
	PWM_OUT_Config();
	PWM_IN_Config();
	ECS_Calibrate();//电调油门行程校准
	//NRF24L01_Init();//使用了433数传
	Ultrasonic_Config();//US-100超声波初始化配置
 	LED_SHOW();
  LED_BLUE_SHOW();
 	FLASH_Unlock();
 	EE_Init();
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Sensor_Init
**功能 : 传感器初始化
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Sensor_Init(void)
{
	flag.MpuExist = InitMPU6050();
	flag.MagExist = Init_HMC5883L();
  flag.NrfExist = NRF24L01_Check();
  flag.MsExist = MS5611_init();
	OLED_Fill(0x00); //清屏
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : OLED_Display
**功能 : 屏幕显示初始化
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void OLED_Display(void)
{	
	if(flag.NrfExist==1)
	{
		OLED_P6x8Str(80,1,"NRF");
		NRF24L01_Mode(MODEL_TX2);//1=RX  2=TX默认应该都是接收模式 3=RX2 4=TX2
		NowMode=MODEL_TX2;		
	}
	OLED_P6x8Str(0,0,VERSION_OLED);
	OLED_P6x8Str(70,0,MULTIROTOR);
	OLED_P6x8Str(0,1,"YAW:");
	OLED_P6x8Str(0,2,"PIT:"); 
	OLED_P6x8Str(0,3,"ROL:");
	OLED_P6x8Str(0,4,"TIM:");
	OLED_P6x8Str(70,3,"ULT:");
	OLED_P6x8Str(56,4,"BAR:");
	OLED_P6x8Str(0,5,"YAW:");
	OLED_P6x8Str(0,6,"THR:");
	OLED_P6x8Str(56,5,"ROLL :");
	OLED_P6x8Str(56,6,"PITCH:"); 
	OLED_P6x8Str(0,7,"MOTO:"); 
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Screen_Update
**功能 : 屏幕数据更新
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Screen_Update(void)
{
	Dis_Float(1,28,IMU.Yaw+180,1);
	Dis_Float(2,28,-IMU.Pitch,1);
	Dis_Float(3,28,IMU.Roll,1);
}

unsigned int testT,testT_old;
unsigned int DISARMED_count,RC_count;

void loop(void)
{	
	if(flag.Loop_100Hz){
		flag.Loop_100Hz=0;
	#ifdef HT_HAWK 
		HT_GCS_Link();//使用恒拓地面站发送与接收数据	
	#endif 	
	#ifdef ANO 
		ANO_DT_Data_Exchange();//使用匿名地面站发送与接收数据	
	#endif 	
	
		Screen_Update();	
	}
		
	if(flag.Loop_20Hz){
			flag.Loop_20Hz=0;
	}

	if(flag.Loop_200Hz){
			flag.Loop_200Hz=0;
			usart_data_RX();
			UsbCmdPro();
			FailSafeLEDAlarm();			
	}
		
	if(flag.Loop_40Hz){
			flag.Loop_40Hz=0;
	}

	if(flag.Loop_10Hz)
		{
		  flag.Loop_10Hz=0;
		  //if(!flag.ARMED)//如果没解锁才显示，解锁后认为要起飞，反正都不用看了，不显示以节约时间
		  //{
			if (flag.ARMED) OLED_P6x8Str(70,1,"ARMED   ");
			else 
			{			
			  if(DISARMED_count==7) 
				  OLED_P6x8Str(70,1,"DISARMED");				
				else if(DISARMED_count==14)
				{
					OLED_P6x8Str(70,1,"        ");
				  DISARMED_count=0;		
				}			
				DISARMED_count++;	
			}
			
		  if(flag.calibratingA)
			{
				DisTIMER;
				Gyro_OFFSET();		
		    EnTIMER;  //开定时5时间片中断
			}	
			
//			if (flag.MagIssue) OLED_P6x8Str(70,1,"MG:XX");
//			else OLED_P6x8Str(70,1,"MG:OK");
//			if (flag.calibratingM==0) OLED_P6x8Str(105,1,"0");//这几个是校准磁场计时用来看的
//			if (flag.calibratingM==1) OLED_P6x8Str(105,1,"Z");
//			if (flag.calibratingM==2) OLED_P6x8Str(105,1,"P");
//			if (flag.calibratingM==3) OLED_P6x8Str(105,1,"R");
			
			if(RC_Data.YAW || RC_Data.THROTTLE || RC_Data.ROLL || RC_Data.PITCH){
					OLED_4num(4,5,RC_Data.YAW);
					OLED_4num(4,6,RC_Data.THROTTLE);
					OLED_4num(58,5,RC_Data.ROLL);
					OLED_4num(58,6,RC_Data.PITCH);
			}
			else 
			{			
			  if(RC_count==7) 
				{
					OLED_P6x8Str(26,5,"NO"); 
	        OLED_P6x8Str(26,6,"NO"); 
	        OLED_P6x8Str(94,5,"NO"); 
	        OLED_P6x8Str(94,6,"NO");						
				}	
				else if(RC_count==14)
				{
					OLED_P6x8Str(26,5,"  "); 
	        OLED_P6x8Str(26,6,"  "); 
	        OLED_P6x8Str(94,5,"  "); 
	        OLED_P6x8Str(94,6,"  ");
				  RC_count=0;		
				}			
				RC_count++;	
			}
			
			OLED_3num(5,7,Moto_duty[0]);
			OLED_3num(9,7,Moto_duty[1]);
			OLED_3num(13,7,Moto_duty[2]);
			OLED_3num(17,7,Moto_duty[3]);
			
			if(flag.FlightMode==MANUAL_High)
				OLED_P6x8Str(70,2,"Stabilize");
			else if(flag.FlightMode==ULTRASONIC_High)
				OLED_P6x8Str(70,2,"U-ALTHOLD");
			else if(flag.FlightMode==ATMOSPHERE_High)
				OLED_P6x8Str(70,2,"B-ALTHOLD");
			
			Dis_Float(4,30,(float)testtime/1000,1);
			Dis_Float(4,80,(float)Pressure/1000,3);
			Dis_Float(3,94,ultra_dis_lpf/1000,2);
			EE_SAVE_Attitude_PID();		
			//}
		}
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Time_slice
**功能 : 时间
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Time_slice(void)
{
  static u16 tick[6]={0,0,0,0,0,0};
	
	tick[0]++;tick[1]++;tick[2]++;tick[3]++;tick[4]++;tick[5]++;
	
	if(tick[0]>=2){
		tick[0] = 0;
		flag.Loop_200Hz = 1;
	}
  if(tick[1]>=4){
		tick[1] = 0;
		flag.Loop_100Hz = 1;
    timetoconver=1;
	}	
	  if(tick[2]>=10){
		tick[2] = 0;
		flag.Loop_40Hz = 1;
	}	
	  if(tick[3]>=15){
		tick[3] = 0;
		flag.Loop_27Hz = 1;
	}			
  if(tick[4] >= 40)	{
		tick[4] = 0;
		flag.Loop_10Hz = 1;
	}
	if(tick[5] >= 20)	{
	  tick[5] = 0;
	  flag.Loop_20Hz = 1;
  }
}

