/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��MultiRotor_control.c
 * ����     ��������������
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/

#include "include.h"
#include "MultiRotor_control.h"

struct _ctrl ctrl;
struct _target Target;

u16 Moto_duty[MOTOR_NUM];
s16 Moto[MOTOR_NUM];
u16 *motor_array = Moto_duty;
float Thr_Weight;
float thr_value;//����ģʽ�µ�����ֵ
u8 Thr_Low;
float Thr_Weight;
int date_throttle;
extern EulerAngle IMU;
/*====================================================================================================*/
/*====================================================================================================*
**���� : Calculate_target
**���� : ����Ŀ����
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	Target.Pitch = (1500-RC_Data.PITCH)/(20 + 7*RC_Data.SENSITIVITY);//RC_Data.SENSITIVITYû�б���ֵ
	Target.Roll = (RC_Data.ROLL-1500)/(20 + 7*RC_Data.SENSITIVITY);

  //Ŀ�꺽����ơ������Ŵ�����С���ֵʱ����Ϊ�û�ϣ����ɡ���ô��ʱ�ĺ�����ΪĿ�꺽��
   if(RC_Data.THROTTLE > RC_MINCHECK ) {
      if(flag.LockYaw != 1){  
				 flag.LockYaw = 1;
	       Target.Yaw = IMU.Yaw; //����ǰ�ĺ�����ΪĿ�꺽��
      }
   }
   else {
		 flag.LockYaw = 0;	
		 Target.Yaw = IMU.Yaw;
	 } 
	//�������е�����һ���������ô����ֲ���ʱ����С����
	if((RC_Data.YAW > 1530)||(RC_Data.YAW < 1470)){
		ftemp = 1500 - RC_Data.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.1f; 
		
		//ת[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
		
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : CONTROL(struct _target Goal) 
**���� : ����PID����
**���� : Goal
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//*****************�⻷(�ǶȻ�)PID**************************//
		//�������///////////////
	  deviation_pitch = Goal.Pitch - IMU.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);

		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//��������//////////////
		deviation_roll = Goal.Roll - IMU.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//�������////////////
    if((Goal.Yaw - IMU.Yaw)>180 || (Goal.Yaw - IMU.Yaw)<-180){
       if(Goal.Yaw>0 && IMU.Yaw<0)  deviation_yaw= (-180 - IMU.Yaw) +(Goal.Yaw - 180);
       if(Goal.Yaw<0 && IMU.Yaw>0)  deviation_yaw= (180 - IMU.Yaw) +(Goal.Yaw + 180);
    }
    else  deviation_yaw = Goal.Yaw - IMU.Yaw;
		
	  ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp * deviation_yaw;
    ctrl.ctrlRate = 0; 
	}
	ctrl.ctrlRate ++;
  Attitude_RatePID();
	Thr_Ctrl(TT);// ���ſ���
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Attitude_RatePID
**���� : �����ʿ���PID
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
  fp32 E_pitch,E_roll,E_yaw;
	
	// ����ƫ��  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	// ����
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// �����޷�
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// ΢��
	ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (sensor.gyro.histor.y - sensor.gyro.averag.y)*33 / 4;
	ctrl.roll.core.kd_out  = ctrl.roll.core.kd  * (sensor.gyro.histor.x - sensor.gyro.averag.x)*33 / 4;
	ctrl.yaw.core.kd_out   = ctrl.yaw.core.kd   * (sensor.gyro.histor.z - sensor.gyro.averag.z)*33 / 4;	
	
	sensor.gyro.histor.y = sensor.gyro.averag.y;
	sensor.gyro.histor.x = sensor.gyro.averag.x; 
  sensor.gyro.histor.z = sensor.gyro.averag.z;	
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.kp_out + ctrl.pitch.core.ki_out + ctrl.pitch.core.kd_out;
	ctrl.roll.core.pid_out  = ctrl.roll.core.kp_out  + ctrl.roll.core.ki_out  + ctrl.roll.core.kd_out;
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.kp_out   + ctrl.yaw.core.kd_out;
	
	ctrl.pitch.core.pid_out = ctrl.pitch.core.pid_out*0.8 + ctrl.pitch.shell.pid_out/2;
	ctrl.roll.core.pid_out  = ctrl.roll.core.pid_out *0.8 + ctrl.roll.shell.pid_out/2; 
	ctrl.yaw.core.pid_out   = ctrl.yaw.core.pid_out;

}
/*====================================================================================================*/
/*====================================================================================================*
**���� : Motor_Conter(void)
**���� : �������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Motor_Conter(void)
{
	s16 pitch,roll,yaw;
	
		pitch = ctrl.pitch.core.pid_out;
		roll  = ctrl.roll.core.pid_out;    
		yaw   = -ctrl.yaw.core.pid_out;
	
  	if(flag.FlightMode==ULTRASONIC_High || flag.FlightMode==AUTO_High || flag.FlightMode==ACC_High  || flag.FlightMode==ATMOSPHERE_High){
			Moto[0] = thr_value - pitch - roll + yaw;
			Moto[1] = thr_value - pitch + roll - yaw;
			Moto[2] = thr_value + pitch + roll + yaw;
			Moto[3] = thr_value + pitch - roll - yaw;
    }
    else	if(RC_Data.THROTTLE > RC_MINCHECK) {
		  date_throttle	= (RC_Data.THROTTLE-MINRCVALUE)/cos(IMU.Roll/RtA)/cos(IMU.Pitch/RtA);
		
		#ifdef QUADROTOR 
			Moto[0] = date_throttle - pitch - roll + yaw + IDLING;
			Moto[1] = date_throttle - pitch + roll - yaw + IDLING;
			Moto[2] = date_throttle + pitch + roll + yaw + IDLING;
			Moto[3] = date_throttle + pitch - roll - yaw + IDLING;
		#elif defined HEXACOPTER
			Moto[0] = date_throttle - pitch + 0.5*roll - yaw + IDLING;
			Moto[1] = date_throttle         +     roll + yaw + IDLING;
			Moto[2] = date_throttle + pitch + 0.5*roll - yaw + IDLING;
			Moto[3] = date_throttle + pitch - 0.5*roll + yaw + IDLING;	
			Moto[4] = date_throttle         -     roll - yaw + IDLING;
			Moto[5] = date_throttle - pitch - 0.5*roll + yaw + IDLING;	
		#endif 	
		}
		else
		{	
			array_assign(&Moto[0],IDLING,MOTOR_NUM);//������200
			Reset_Integral();//�ڻ�pidȫ�������0		
		}
		
		if(flag.ARMED)
		{	
			#ifdef QUADROTOR 
					Moto_duty[0]=Moto[0];
					Moto_duty[1]=Moto[1];
					Moto_duty[2]=Moto[2];
					Moto_duty[3]=Moto[3];			
			
			#elif defined HEXACOPTER
					Moto_duty[0]=Moto[0];
					Moto_duty[1]=Moto[1];
					Moto_duty[2]=Moto[2];
					Moto_duty[3]=Moto[3];	
					Moto_duty[4]=Moto[4];
					Moto_duty[5]=Moto[5];	
			
			#endif 	
			
			moto_PwmRflash(&Moto_duty[0]);//������ˢ�£�ֱ��дPWM����Ĵ���	
		}	
		else 
		{
			 array_assign(&Moto_duty[0],0,MOTOR_NUM);//������0
		   moto_STOP();//ǿ�����1000	
		}				
}

/*====================================================================================================*/
/*====================================================================================================*
**���� : Reset_Integral
**���� : ��������
**���� : None
**ݔ�� : None
**��ע : None
**====================================================================================================*/
/*====================================================================================================*/
void Reset_Integral(void)
{
	ctrl.pitch.shell.increment = 0;
	ctrl.roll.shell.increment= 0;	
  ctrl.pitch.core.increment = 0;		
  ctrl.roll.core.increment = 0;		
	ctrl.yaw.core.increment = 0;
}


void Thr_Ctrl(float T)
{
///////////////////////////////////////////////////////////////////////////		
	static float thr;
	static float Thr_tmp;
	thr = RC_Data.THROTTLE-1110; //����ֵthr 0 ~ 1000
	Thr_tmp += 10 *3.14f *T *(thr/250.0f - Thr_tmp); //��ͨ�˲�
	Thr_Weight = LIMIT(Thr_tmp,0,1);    	//��߶ദ�������ݻ��õ����ֵ
	
///////////////////////////////////////////////////////////////////////////////	

	if( thr < 100 )
	{
		Thr_Low = 1;
	}
	else
	{
		Thr_Low = 0;
	}
	
	#if(CTRL_HEIGHT)

	Height_Ctrl(T,thr);

	thr_value = Thr_Weight *height_ctrl_out;   //ʵ��ʹ��ֵ

	#else
	thr_value = thr;   //ʵ��ʹ��ֵ

	#endif

	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);//�����������Ϊ800����200��ظ���̬����
}
