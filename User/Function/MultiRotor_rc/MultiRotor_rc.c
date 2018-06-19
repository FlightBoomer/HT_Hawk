/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��MultiRotor_rc.c
 * ����     �������봦��ң��������      
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "MultiRotor_rc.h"
#include "include.h"

RC_GETDATA RC_Data;
rcReadRawData rcReadRawFunc = RC_Data_Refine;
extern u16 Moto_duty[MOTOR_NUM];

void RDAU(void)
{
	RC_directive();
	rcReadRawFunc();
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : RC_directive
**���� : ң��ָ��
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
u8  rcSticks,keySticks;
void RC_directive(void)
{
  u8 stTmp = 0,keyTemp=0,i;
	//static u8  rcSticks,keySticks;
	static u16  rcDelayCommand,keyDelayCommand;
  static u16 seltLockCommend;	
/**************************���´���ң�ؿ���ͨ������******************************/	
	for (i = 0; i < 4; i++) {
			stTmp >>= 2;
			if (RC_Data.rc_data[i] > RC_MINCHECK)
					stTmp |= 0x80;  // check for MIN
			if (RC_Data.rc_data[i] < RC_MAXCHECK)
					stTmp |= 0x40;  // check for MAX
	}

	if (stTmp == rcSticks) {
			if (rcDelayCommand < 600)
					rcDelayCommand++;
	} else
			rcDelayCommand = 0;
	rcSticks = stTmp;
	
	if (rcDelayCommand == ARMED_TIME) {
		if (flag.ARMED){
			 if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE)   //���� ����+��λ
				  flag.ARMED=0;
		}
		else{
					if ((rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE) && flag.FlightMode==MANUAL_High)    //���� ����+��λ ,�����ֶ�ģʽ 
					{
						 flag.ARMED=1;Pressure_groud=Pressure;//���������µ�ǰ����ѹֵ��Ϊ������ѹֵ��
					}
					if (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_HI)    //���ٶȽ��� ����+��
							{flag.calibratingA = 1;flag.calibratingG = 1;stTmp=0;rcSticks=0;}
					if ((rcSticks == THR_HI + YAW_LO + PIT_LO + ROL_HI) && flag.calibratingM_pre)//����+����ָ������� 
							flag.calibratingM = 1; 
					if (rcSticks == THR_HI + YAW_HI + PIT_LO + ROL_LO)//����+����    
							flag.calibratingM_pre = 1;
					else flag.calibratingM_pre = 0;	
    }
	}
	//����֮��һ��ʱ�����ű������  ���Զ�����
	if (flag.ARMED){
	   if (rcSticks == THR_LO + YAW_CE + PIT_CE + ROL_CE) {
		    if (seltLockCommend < AUTODISARMED_TIME)
					 seltLockCommend++;
				else 
					 flag.ARMED=0;
		 }
		 else 
        seltLockCommend = 0;			 
	}

/**************************���´���ң�ؿ�������******************************/
    for (i = 0; i < 4; i++) {
            keyTemp >>= 2;
            if (RC_Data.rc_data[i+4] > RC_MINCHECK)
                    keyTemp |= 0x80;  // check for MIN
            if (RC_Data.rc_data[i+4] < RC_MAXCHECK)
                    keyTemp |= 0x40;  // check for MAX
            }
    if (keyTemp == keySticks) {
            if (keyDelayCommand < 200)//250
                    keyDelayCommand++;
            else keyDelayCommand = 0;
    } else
            keyDelayCommand = 0;
    keySticks = keyTemp;

    if (keyDelayCommand==120) {
       // if (flag.ARMED) {
            if ((keySticks&0X03)==CH5_LO) {//CH5���ڵ�λʱ
                flag.FlightMode=MANUAL_High;
							  flag.HUDMode=STABILIZE_MODE;
							//keephigh=0;
            }
            if ((keySticks&0X03)==CH5_CE) {//CH5������λʱ
                flag.FlightMode=ULTRASONIC_High;//
							  flag.HUDMode=ALTHOLD_MODE;
							//keephigh=1;
            }
            if ((keySticks&0X03)==CH5_HI) {//CH5���ڸ�λʱ
              //gps_flag=1;
							  flag.FlightMode=ATMOSPHERE_High;//ULTRASONIC_High;ACC_High;// 
							  flag.HUDMode=ALTHOLD_MODE;
            }
            if ((keySticks&0X0C)==CH6_LO) {;//CH6���ڵ�λʱ
							 //flag.ControlMode=RC_MODE;
               flag.ARMED = 0;
            }
            if ((keySticks&0X0C)==CH6_CE) {;//CH6������λʱ
               flag.ARMED = 0;
            }
            if ((keySticks&0X0C)==CH6_HI) {;//CH6���ڸ�λʱ
							 //flag.ControlMode=PC_MODE;
               flag.ARMED = 1;              
            }
            if ((keySticks&0X30)==CH7_LO) {//CH7���ڵ�λʱ
            }
            if ((keySticks&0X30)==CH7_CE) {//CH7������λʱ
                //keephigh=2;
                //lockpressure=0;
            }
            if ((keySticks&0X30)==CH7_HI) {//CH7���ڸ�λʱ
               // keephigh=3;
               flag.ARMED = 0;
            }
            if ((keySticks&0XC0)==CH8_LO) {;//CH8���ڵ�λʱ
            }
            if ((keySticks&0XC0)==CH8_CE) {;//CH8������λʱ
            }
            if ((keySticks&0XC0)==CH8_HI) {;//CH8���ڸ�λʱ
            }
        //}
    }
    
}
/*====================================================================================================*/
/*====================================================================================================*
**���� : RcData_Refine
**���� : ����ң������
**���� : None
**��� : None
**��ע : ��
**====================================================================================================*/
/*====================================================================================================*/
void RC_Data_Refine(void)
{
  u8 chan,a;	

	u16 rcDataMax[8], rcDataMin[8];
	static int16_t rcDataCache[8][4], rcDataMean[8];
	static uint8_t rcValuesIndex = 0;

	rcValuesIndex++;
	for (chan = 0; chan < 8; chan++) {
		  //����ƽ��ֵ�˲���4��
		  if(RC_Pwm_In[chan]>2800 || RC_Pwm_In[chan]<800)  RC_Pwm_In[chan] = RC_Pwm_In_his[chan];
			rcDataCache[chan][rcValuesIndex % 4] = RC_Pwm_In[chan] ;		
		  RC_Pwm_In_his[chan] = RC_Pwm_In[chan];
			
			rcDataMean[chan] = 0;
		  rcDataMax[chan]  = 0;
		  rcDataMin[chan]  = 25000;
		
			for (a = 0; a < 4; a++) {
				  // ��¼���������ֵ && ��Сֵ
				  if(rcDataCache[chan][a] > rcDataMax[chan])  rcDataMax[chan] = rcDataCache[chan][a];     
					if(rcDataCache[chan][a] < rcDataMin[chan])	rcDataMin[chan] = rcDataCache[chan][a]; 
				  // ���
					rcDataMean[chan] += rcDataCache[chan][a];  
      }
			// �޳������� ���ֵ && ��Сֵ 
			rcDataMean[chan] = (rcDataMean[chan] - (rcDataMax[chan] + rcDataMin[chan])) / 2;
	} 

	 RC_Data.ROLL     = RC_Data.rc_data[0] = rcDataMean[0];
	 RC_Data.PITCH    = RC_Data.rc_data[1] = rcDataMean[1];
	 RC_Data.THROTTLE = RC_Data.rc_data[3] = rcDataMean[2];
	 RC_Data.YAW      = RC_Data.rc_data[2] = rcDataMean[3];
	 RC_Data.rc_data[4] = rcDataMean[4];
	 RC_Data.rc_data[5] = rcDataMean[5];
	 RC_Data.rc_data[6] = rcDataMean[6];
    
    if (RC_Data.THROTTLE > 1000) 
        RC_Data.THROTTLE = (RC_Data.THROTTLE - 1000) * 0.6f + 1000;     // �����޷�����Ϊ���Ի��ǳ��ᣬ�������Ź��߲��׿���
}

/*====================================================================================================*/
/*====================================================================================================**
**���� : ECS_Calibrate
**���� : ��������г�����
**���� : None
**��� : None
**��ע :
		//             �����г�����               //
	  //     ������������������            ������������������     //
	  //    |    |    |          |         |    //
	  //    |    |    |          |    |    |    //
	  //    |         |          |    |    |    //
	  //    |         |          |         |    //
	  //     ������������������            ������������������     //
	  //   �����Ƶ����         �Ҳ�ҡ�����м�  //	
**====================================================================================================*/
/*====================================================================================================*/
void ECS_Calibrate(void)
{
	static vs8 ECS_Calibrate_FLAG=0;
	static vs16 time=2000;
	while(time--)
	  RDAU();
  while(RC_Data.THROTTLE >= 1850 || ECS_Calibrate_FLAG)
  {   
		LED_ALLON();//��ɫLED����
		OLED_Print(35,6,"���У׼");
		ECS_Calibrate_FLAG=1;//�����г̱�־λ��1
    RDAU();	
			
		#ifdef QUADROTOR 
     Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = RC_Data.THROTTLE-1000;
		#elif defined HEXACOPTER
      Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = Moto_duty[4] = Moto_duty[5] = RC_Data.THROTTLE-1000;
		#endif 
	  moto_PwmRflash(&Moto_duty[0]);//������ˢ�£�ֱ��дPWM����Ĵ���
  }
}
