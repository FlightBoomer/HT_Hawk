/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：height_ctrl.c
 * 描述     ：高度控制       
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "height_ctrl.h"
#include "include.h"
#include "mymath.h"

extern float Thr_Weight;
extern Gravity V;//重力分量

/******************高度控制变量********************/
float height_ctrl_out;
float wz_acc;
float keepheight_thr;
/*-------------------------------------------------*/

/******************超声波变量********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // 积分幅度
u8 lock=0;
extern s8 ultra_start_f;
extern float US100_Alt_delta;
float exp_height_speed,exp_height;
float ultra_speed,ultra_speed_acc;
float ultra_dis_lpf;
float ultra_ctrl_out;
_st_height_pid_v ultra_ctrl;
_st_height_pid ultra_pid;
/*-------------------------------------------------*/

/******************速度环变量********************/
float wz_speed;
float wz_acc_mms2;
float tempacc_lpf;
u8 lock_spd_crl=0;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;
_st_height_pid ultra_wz_speed_pid;
_st_height_pid baro_wz_speed_pid;
/*-------------------------------------------------*/

/******************气压计变量********************/
float baro_height,baro_height_old;
#define BARO_SPEED 		 300    // mm/s
#define BARO_INT        300    // 积分幅度

u8 lock_BARO=0;//定高锁定当前以及清零用
#define BARO_SPEED_NUM 10
float baro_speed_arr[BARO_SPEED_NUM + 1];

#define ACC_SPEED_NUM 50
float acc_speed_arr[ACC_SPEED_NUM + 1];

float baro_dis_lpf,baro_dis_kalman;
float baro_ctrl_out;
_st_height_pid_v baro_ctrl;
_st_height_pid baro_pid;

u16 baro_cnt[2];
u16 acc_cnt[2];
float baro_speed,baro_speed_lpf;
float baro_height;
float Pressure_groud;
/*-------------------------------------------------*/

void Height_Ctrl(float T,float thr)
{	
	static u8 height_ctrl_start_f;
	static float thr_lpf;
	static float height_thr;
	static float wz_acc_temp,wz_acc1;
	static float hc_speed_i,wz_speed_0;
	static float h_speed;
	
	switch( height_ctrl_start_f )
	{
		case 0:
	
				if( sensor.acc.averag.z > 7000 )//注意这里，指水平静态下的Z值要比这个大，表示水平垂直
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh(thr_lpf);//一旦切到控高模式，立即保存当前的距离值，油门值、相关清零。
				if((flag.FlightMode==ULTRASONIC_High && lock==1 &&lock_spd_crl==0)|| (flag.FlightMode==ATMOSPHERE_High && lock_BARO==1 &&lock_spd_crl==0))
				{
						keepheight_thr=thr_lpf;
						wz_speed_0=0;
						hc_speed_i=0;
						hc_speed_i=0;
						wz_speed=0;
						wz_speed_pid_v.err_old=0;
						wz_speed_pid_v.err=0;
						wz_speed_pid_v.err_i=0;
						lock=2;
						lock_BARO=2;
						lock_spd_crl=1;
				}
				else if(flag.FlightMode==MANUAL_High)
				{
						lock=0;
						lock_BARO=0;
						lock_spd_crl=0;//切回手动模式后置零，以使下次进入定高模式是能顺利置零速度内环	
				}
			
				height_thr = LIMIT( thr , 0, 700 );
				thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//对油门值低通滤波修正
				userdata1[0]=	thr_lpf;//调试用
				/*下面的低通滤波用于测试对比效果，最终没有选用*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//加速度的静态零点是读取的EEPROM数据，但是每次飞行都可能不一样，能每次起飞前校准一次最好，可以单独校准Z轴的，自行设计程序
				wz_acc_temp = V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;//
				Moving_Average( wz_acc_temp,acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,&wz_acc1 );
				tempacc_lpf= (float)(wz_acc1/8192.0f) *9800;//9800 *T;由于是+-4G共8G，65535/8g=8192 g，加速度，mms2毫米每平方秒
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//简单消除下噪声
				userdata1[2]=tempacc_lpf;
				wz_speed_0 += tempacc_lpf *T;//加速度计积分成速度
								
				if( ultra_start_f == 1 )////不管是啥模式，只要更新了超声波数据就进行运算
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra_start_f=2;
				}
				
				if(baro_start_f==1)//不管是啥模式，只要更新了气压数据就进行运算
				{ 
					 Baro_dataporcess(8.0f*TT);			 //8.0f*TT这么长时间更新一次气压计数据
					 userdata2[10]=baro_height;//Baro_Height_Source
					 baro_dis_delta=baro_height-baro_dis_old;
					 baro_dis_old=baro_height;//气压计速度可以继续优化，这里比较粗糙			 
		       //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //单位mm/s
					 userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed这里乘以系数以削减该值		 
					 baro_start_f=2;
				}	
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed=ultra_speed;
					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//超声波垂直速度互补滤波
				}				
				else if(flag.FlightMode==ATMOSPHERE_High)
				{
					 h_speed=baro_speed_lpf;
					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//气压计垂直速度互补滤波，系数可调
				}	

				userdata1[3] =h_speed;//h_speed是高度环传到速度环的实测高度方向速度【但可能是错误的，气压计速度不可靠】
				userdata1[4]=wz_speed_0;//调试用
				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//速度偏差积分，乘以了0.4系数
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//积分限幅	
				userdata1[5] =hc_speed_i;//这个没显示
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1实测速度修正加速度算的速度
				userdata1[6] = wz_speed=wz_speed_0 + hc_speed_i;//经过修正的速度+经过限幅的增量式速度积分

			  if( flag.FlightMode == ATMOSPHERE_High)
				{
						if(baro_start_f==2)//说明有新数据且被上面移动均值滤波使用过了
						{
							baro_start_f = -1;
							Baro_Ctrl(8.0f*TT,thr_lpf);
						}	
						height_speed_ctrl(T,keepheight_thr,baro_ctrl_out,baro_speed_lpf);							
				}

				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T,keepheight_thr,ultra_ctrl_out,ultra_speed);//系数原来是0.4
							
						if( ultra_start_f == 2 )//超声波数据更新了且被运算了
						{				
								Ultra_Ctrl(15.0f*TT,thr_lpf);//realtime是周期为 TT 秒的，#define TT 0.0025//控制周期2.5ms
								ultra_start_f = -1;
						}
				}
		
	      /*******************************************控制高度输出********************************************/
				if(flag.FlightMode==ULTRASONIC_High || flag.FlightMode==ATMOSPHERE_High || flag.FlightMode==ACC_High)//注意这里，模式
				{		
					  height_ctrl_out = wz_speed_pid_v.pid_out;
				}
				else
				{
					  height_ctrl_out = thr;
				}
				userdata2[0]=exp_height;
			
				break; 	
				default: break;
	} 
}

void WZ_Speed_PID_Init()
{
		ultra_wz_speed_pid.kp = 0.25;// //超声波定高的速度环PID，会被EEPROM里的重新赋值的
		ultra_wz_speed_pid.ki = 0.08; //0.12
		ultra_wz_speed_pid.kd = 8;
		baro_wz_speed_pid.kp = 0.1;// 气压定高的速度环PID，会被EEPROM里的重新赋值的
		baro_wz_speed_pid.ki = 0.06; 
		baro_wz_speed_pid.kd = 8;//1.4*10，在计算式乘以了10，已删除
		wz_speed_pid.kp = 0.1;//速度环默认PID，真正用不到被重新赋值
		wz_speed_pid.ki = 0.08;
		wz_speed_pid.kd = 8;
}

void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed)
{
    userdata1[1] =	thr_keepheight;
		//wz_speed被用于参与超声波高度D运算
		userdata1[7]=wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );//
		userdata1[8]=wz_speed_pid_v.err_d = wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		wz_speed_pid_v.err_i += wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//期望速度与实际速度误差积分
		wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *200,Thr_Weight *200);
		userdata1[9]=wz_speed_pid_v.err_i;
		wz_speed_pid_v.pid_out = thr_keepheight + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);//积分原来没有乘wz_speed_pid.kp
		userdata1[10]=wz_speed_pid_v.pid_out-thr_keepheight;
		wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}

void Baro_PID_Init()
{
		baro_pid.kp = 0.15;//气压定高的高度位置环PID，会被EEPROM里的重新赋值的
		baro_pid.ki = 0;
		baro_pid.kd = 1.5;	//2.5
}

void Baro_dataporcess(float T)
{
		float baro_dis_tmp;	
		
		baro_dis_tmp = Moving_Median(2,5,baro_height);//对超声波测量的距离进行移动中位值滤波

		if( baro_dis_tmp < Baro_MAX_Height )
		{	
			if( ABS(baro_dis_tmp - baro_dis_lpf) < 100 )
			{	
				 baro_dis_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *(baro_dis_tmp - baro_dis_lpf) ;
			}
			else if( ABS(baro_dis_tmp - baro_dis_lpf) < 500 )
			{
				 baro_dis_lpf += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) *(baro_dis_tmp- baro_dis_lpf) ;
			}
			else
			{
				 baro_dis_lpf += ( 1 / ( 1 + 1 / ( 0.5f *3.14f *T ) ) ) *(baro_dis_tmp- baro_dis_lpf) ;
			}
		}
		userdata2[2]=baro_dis_lpf;//调试用
		userdata2[3]+= ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) *(Baro_Height_Source- userdata2[3]) ;//调试用	
		userdata2[4]=baro_speed;//调试用
}

void Baro_Ctrl(float T,float thr)
{
		exp_height_speed = BARO_SPEED *my_deathzoom_2(thr - 500,100)/400.0f; //20这里具体根据自己起飞油门定，油门控制高度+-ULTRA_SPEEDmm / s
		exp_height_speed = LIMIT(exp_height_speed ,-BARO_SPEED,BARO_SPEED);
		
		if( exp_height > Baro_MAX_Height )//限定高度，可以根据实际修改
		{
			if( exp_height_speed > 0 )
			{
				exp_height_speed = 0;
			}
		}
		else if( exp_height < Baro_MIN_Height )
		{
			if( exp_height_speed < 0 )
			{
				exp_height_speed = 0;
			}
		}
		
		exp_height += exp_height_speed *T;//累积期望高度，因为期望速度可能改变了			
		baro_ctrl.err = baro_pid.kp*(exp_height - baro_dis_lpf);//baro_dis_lpfmm 对高度误差乘以Kp
		userdata2[5]=baro_ctrl.err;
		baro_ctrl.err_i += baro_pid.ki *baro_ctrl.err *T;//对高度误差积分
		userdata2[6]=baro_ctrl.err_i = LIMIT(baro_ctrl.err_i,-Thr_Weight *BARO_INT,Thr_Weight *BARO_INT);//积分限幅，融合了油门权重
		//对于D，融合了加速度计运算得到的距离	
		userdata2[7]=0.4f *(baro_ctrl.err - baro_ctrl.err_old);
		baro_ctrl.err_d = baro_pid.kd *( 0.6f *(-wz_speed*T) + 0.4f *(baro_ctrl.err - baro_ctrl.err_old) );
		userdata2[8]=baro_ctrl.err_d;
		baro_ctrl.pid_out = baro_ctrl.err + baro_ctrl.err_i + baro_ctrl.err_d;
		baro_ctrl.pid_out = LIMIT(baro_ctrl.pid_out,-300,300);	
		userdata2[9]=baro_ctrl_out = baro_ctrl.pid_out;	
		baro_ctrl.err_old = baro_ctrl.err;
}

void Ultra_PID_Init()
{
		ultra_pid.kp = 0.5;//超声波定高的高度位置环PID，会被EEPROM里的重新赋值的
		ultra_pid.ki = 0;
		ultra_pid.kd = 2.5;	//2.5
}

void Ultra_dataporcess(float T)
{
		float ultra_sp_tmp,ultra_dis_tmp;	
		ultra_dis_tmp = Moving_Median(1,5,US100_Alt);//对超声波测量的距离进行移动中位值滤波
		userdata2[1]=ultra_dis_tmp;
		if( ultra_dis_tmp < Ultrasonic_MAX_Height )
		{	
			if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 100 )
			{	
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 4.0f *3.14f *T ) ) ) *(ultra_dis_tmp - ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 200 )
			{			
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 2.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else if( ABS(ultra_dis_tmp - ultra_dis_lpf) < 400 )
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 1.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
			else
			{
				ultra_dis_lpf += ( 1 / ( 1 + 1 / ( 0.2f *3.14f *T ) ) ) *(ultra_dis_tmp- ultra_dis_lpf) ;
			}
		}
		
		//注意超声波测量的 距离的时效性问题，避免反复被计算
		ultra_sp_tmp = Moving_Median(0,5,US100_Alt_delta/T); //对超声波测距出来两次之间距离差计算的速度中值滤波 ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//小于3米,注意这里，万一超出了呢？
		{
			if( ABS(ultra_sp_tmp) < 100 )//运动速度小于100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed会传递给速度环，作为当前速度的反馈,原来是4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//系数越大作用越小，原来是1
		}
}

void Ultra_Ctrl(float T,float thr)
{
		exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,100)/300.0f; //20这里具体根据自己起飞油门定，油门控制高度+-ULTRA_SPEEDmm / s
		exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
		
		if( exp_height > Ultrasonic_MAX_Height )//超出超声波高度稳定范围则不执行
		{
			if( exp_height_speed > 0 )
			{
				exp_height_speed = 0;
			}
		}
		else if( exp_height < Ultrasonic_MIN_Height )
		{
			if( exp_height_speed < 0 )
			{
				exp_height_speed = 0;
			}
		}
		
		exp_height += exp_height_speed *T;//累积期望高度，因为期望速度可能改变了

		ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm 对高度误差乘以Kp
		userdata2[5]=ultra_ctrl.err;
		ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;//对高度误差积分
		userdata2[6]=ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);//积分限幅，融合了油门权重
					//对于D，融合了加速度计运算得到的距离	
		userdata2[7]=0.4f *(ultra_ctrl.err_old - ultra_ctrl.err);
		ultra_ctrl.err_d = ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(ultra_ctrl.err - ultra_ctrl.err_old) );
		userdata2[8]=ultra_ctrl.err_d;
		ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;	
		ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
		
		if(flag.Special_Mode==ACC_High)ultra_ctrl.pid_out=0;//加入这代码用于测试加速度计定高效果,但仍然有速度反馈	
		userdata2[9]=ultra_ctrl_out = ultra_ctrl.pid_out;
		ultra_ctrl.err_old = ultra_ctrl.err;
}

void LockForKeepHigh(float THROTTLE)
{
    if (flag.FlightMode==ULTRASONIC_High && lock==0)
		{
			  wz_speed_pid.kp=ultra_wz_speed_pid.kp;//变PID
		    wz_speed_pid.ki=ultra_wz_speed_pid.ki;
		    wz_speed_pid.kd=ultra_wz_speed_pid.kd;
        exp_height=ultra_dis_lpf;//这个超声波高度值经过滤波处理
			  wz_acc=0;
			  wz_speed_pid_v.err_i=0;
				ultra_speed=0;//若不清零，则且超声波时原来累积的速度会错误传递给当前，造成掉高
			  ultra_ctrl.err_i=0;
				ultra_ctrl.err_old=0;
			  lock=1;
    }
		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ATMOSPHERE_High)
		{lock=0;}
						
		if (flag.FlightMode==ATMOSPHERE_High && lock_BARO==0)
		{
			  wz_speed_pid.kp=baro_wz_speed_pid.kp;//变PID
		    wz_speed_pid.ki=baro_wz_speed_pid.ki;
		    wz_speed_pid.kd=baro_wz_speed_pid.kd;
        exp_height=baro_dis_lpf;
			  wz_acc=0;
				baro_speed=0;
				baro_speed_lpf=0;
			  baro_ctrl.err_i=0;
				baro_ctrl.err_old=0;				
			  lock_BARO=1;				
    }
		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ULTRASONIC_High)
		{lock_BARO=0;}					
}
