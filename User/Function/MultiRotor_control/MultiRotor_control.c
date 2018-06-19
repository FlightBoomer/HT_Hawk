/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MultiRotor_control.c
 * 描述     ：电机控制与输出
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/

#include "include.h"
#include "MultiRotor_control.h"

struct _ctrl ctrl;
struct _target Target;

u16 Moto_duty[MOTOR_NUM];
s16 Moto[MOTOR_NUM];
u16 *motor_array = Moto_duty;
float Thr_Weight;
float thr_value;//定高模式下的油门值
u8 Thr_Low;
float Thr_Weight;
int date_throttle;
extern EulerAngle IMU;
/*====================================================================================================*/
/*====================================================================================================*
**函数 : Calculate_target
**功能 : 计算目标量
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Calculate_Target(void) 
{
	int16_t ftemp=0;
	Target.Pitch = (1500-RC_Data.PITCH)/(20 + 7*RC_Data.SENSITIVITY);//RC_Data.SENSITIVITY没有被赋值
	Target.Roll = (RC_Data.ROLL-1500)/(20 + 7*RC_Data.SENSITIVITY);

  //目标航向控制。当油门大于最小检查值时，认为用户希望起飞。那么此时的航向做为目标航向
   if(RC_Data.THROTTLE > RC_MINCHECK ) {
      if(flag.LockYaw != 1){  
				 flag.LockYaw = 1;
	       Target.Yaw = IMU.Yaw; //将当前的航向做为目标航向
      }
   }
   else {
		 flag.LockYaw = 0;	
		 Target.Yaw = IMU.Yaw;
	 } 
	//航向在中点设置一个死区，好处是手操作时忽略小动作
	if((RC_Data.YAW > 1530)||(RC_Data.YAW < 1470)){
		ftemp = 1500 - RC_Data.YAW; 
	  Target.Yaw += (ftemp / 200.0f)*0.1f; 
		
		//转[-180.0,+180.0]
	  if(Target.Yaw >180.0f) Target.Yaw -= 360.0f;	
	  else if(Target.Yaw <-180.0f)Target.Yaw += 360.0f;
	}
		
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : CONTROL(struct _target Goal) 
**功能 : 串级PID控制
**输入 : Goal
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void CONTROL(struct _target Goal)   
{
	float  deviation_pitch,deviation_roll,deviation_yaw;
	
	if(ctrl.ctrlRate >= 2)
	{////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//*****************外环(角度环)PID**************************//
		//横滚计算///////////////
	  deviation_pitch = Goal.Pitch - IMU.Pitch;
		ctrl.pitch.shell.increment += deviation_pitch;
		
		//limit for the max increment
		ctrl.pitch.shell.increment = data_limit(ctrl.pitch.shell.increment,ctrl.pitch.shell.increment_max,-ctrl.pitch.shell.increment_max);

		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * deviation_pitch + ctrl.pitch.shell.ki * ctrl.pitch.shell.increment;
		
		//俯仰计算//////////////
		deviation_roll = Goal.Roll - IMU.Roll;
		ctrl.roll.shell.increment += deviation_roll;
		
		//limit for the max increment
		ctrl.roll.shell.increment = data_limit(ctrl.roll.shell.increment,ctrl.roll.shell.increment_max,-ctrl.roll.shell.increment_max);

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * deviation_roll + ctrl.roll.shell.ki * ctrl.roll.shell.increment;
		
		//航向计算////////////
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
	Thr_Ctrl(TT);// 油门控制
	Motor_Conter();
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Attitude_RatePID
**功能 : 角速率控制PID
**输入 : None
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void Attitude_RatePID(void)
{
  fp32 E_pitch,E_roll,E_yaw;
	
	// 计算偏差  
	E_pitch = ctrl.pitch.shell.pid_out - sensor.gyro.averag.y;
	E_roll  = ctrl.roll.shell.pid_out  - sensor.gyro.averag.x;
	E_yaw   = ctrl.yaw.shell.pid_out   - sensor.gyro.averag.z;
	
	// 积分
	ctrl.pitch.core.increment += E_pitch;
	ctrl.roll.core.increment  += E_roll;
	ctrl.yaw.core.increment   += E_yaw;
	
	// 积分限幅
	ctrl.pitch.core.increment = data_limit(ctrl.pitch.core.increment,20,-20);
	ctrl.roll.core.increment  = data_limit(ctrl.roll.core.increment,20,-20);		
	ctrl.yaw.core.increment   = data_limit(ctrl.yaw.core.increment,20,-20);
	
	ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * E_pitch;
	ctrl.roll.core.kp_out  = ctrl.roll.core.kp  * E_roll;
	ctrl.yaw.core.kp_out   = ctrl.yaw.core.kp   * E_yaw;
	
	ctrl.pitch.core.ki_out = ctrl.pitch.core.ki * ctrl.pitch.core.increment;
  ctrl.roll.core.ki_out  = ctrl.roll.core.ki  * ctrl.roll.core.increment;
	ctrl.yaw.core.ki_out   = ctrl.yaw.core.ki   * ctrl.yaw.core.increment;
	
	// 微分
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
**函数 : Motor_Conter(void)
**功能 : 电机控制
**输入 : None
**輸出 : None
**备注 : None
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
			array_assign(&Moto[0],IDLING,MOTOR_NUM);//马达输出200
			Reset_Integral();//内环pid全部输出置0		
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
			
			moto_PwmRflash(&Moto_duty[0]);//马达输出刷新，直接写PWM输出寄存器	
		}	
		else 
		{
			 array_assign(&Moto_duty[0],0,MOTOR_NUM);//马达输出0
		   moto_STOP();//强制输出1000	
		}				
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : Reset_Integral
**功能 : 积分清零
**输入 : None
**輸出 : None
**备注 : None
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
	thr = RC_Data.THROTTLE-1110; //油门值thr 0 ~ 1000
	Thr_tmp += 10 *3.14f *T *(thr/250.0f - Thr_tmp); //低通滤波
	Thr_Weight = LIMIT(Thr_tmp,0,1);    	//后边多处分离数据会用到这个值
	
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

	thr_value = Thr_Weight *height_ctrl_out;   //实际使用值

	#else
	thr_value = thr;   //实际使用值

	#endif

	thr_value = LIMIT(thr_value,0,10 *MAX_THR *MAX_PWM/100);//限制油门最大为800，留200余地给姿态控制
}
