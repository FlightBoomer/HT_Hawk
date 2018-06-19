/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��height_ctrl.c
 * ����     ���߶ȿ���       
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
#include "height_ctrl.h"
#include "include.h"
#include "mymath.h"

extern float Thr_Weight;
extern Gravity V;//��������

/******************�߶ȿ��Ʊ���********************/
float height_ctrl_out;
float wz_acc;
float keepheight_thr;
/*-------------------------------------------------*/

/******************����������********************/
#define ULTRA_SPEED 		 300    // mm/s
#define ULTRA_INT        300    // ���ַ���
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

/******************�ٶȻ�����********************/
float wz_speed;
float wz_acc_mms2;
float tempacc_lpf;
u8 lock_spd_crl=0;
_st_height_pid_v wz_speed_pid_v;
_st_height_pid wz_speed_pid;
_st_height_pid ultra_wz_speed_pid;
_st_height_pid baro_wz_speed_pid;
/*-------------------------------------------------*/

/******************��ѹ�Ʊ���********************/
float baro_height,baro_height_old;
#define BARO_SPEED 		 300    // mm/s
#define BARO_INT        300    // ���ַ���

u8 lock_BARO=0;//����������ǰ�Լ�������
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
	
				if( sensor.acc.averag.z > 7000 )//ע�����ָˮƽ��̬�µ�ZֵҪ������󣬱�ʾˮƽ��ֱ
				{
					  height_ctrl_start_f = 1;
				}
				break;
		
		case 1:
			
				LockForKeepHigh(thr_lpf);//һ���е��ظ�ģʽ���������浱ǰ�ľ���ֵ������ֵ��������㡣
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
						lock_spd_crl=0;//�л��ֶ�ģʽ�����㣬��ʹ�´ν��붨��ģʽ����˳�������ٶ��ڻ�	
				}
			
				height_thr = LIMIT( thr , 0, 700 );
				thr_lpf += ( 1 / ( 1 + 1 / ( 2.0f *3.14f *T ) ) ) *( height_thr - thr_lpf );//������ֵ��ͨ�˲�����
				userdata1[0]=	thr_lpf;//������
				/*����ĵ�ͨ�˲����ڲ��ԶԱ�Ч��������û��ѡ��*/				
		    //wz_acc += ( 1 / ( 1 + 1 / ( 8 *3.14f *T ) ) ) *my_deathzoom( (V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y - wz_acc),100 );//
		    //wz_acc_mms2 = (float)(wz_acc/8192.0f) *9800 ;
				//���ٶȵľ�̬����Ƕ�ȡ��EEPROM���ݣ�����ÿ�η��ж����ܲ�һ������ÿ�����ǰУ׼һ����ã����Ե���У׼Z��ģ�������Ƴ���
				wz_acc_temp = V.z *(sensor.acc.averag.z- sensor.acc.quiet.z) + V.x *sensor.acc.averag.x + V.y *sensor.acc.averag.y;//
				Moving_Average( wz_acc_temp,acc_speed_arr,ACC_SPEED_NUM,acc_cnt ,&wz_acc1 );
				tempacc_lpf= (float)(wz_acc1/8192.0f) *9800;//9800 *T;������+-4G��8G��65535/8g=8192 g�����ٶȣ�mms2����ÿƽ����
				if(abs(tempacc_lpf)<50)tempacc_lpf=0;//������������
				userdata1[2]=tempacc_lpf;
				wz_speed_0 += tempacc_lpf *T;//���ٶȼƻ��ֳ��ٶ�
								
				if( ultra_start_f == 1 )////������ɶģʽ��ֻҪ�����˳��������ݾͽ�������
				{	
					 Ultra_dataporcess(15.0f*TT);
					 ultra_start_f=2;
				}
				
				if(baro_start_f==1)//������ɶģʽ��ֻҪ��������ѹ���ݾͽ�������
				{ 
					 Baro_dataporcess(8.0f*TT);			 //8.0f*TT��ô��ʱ�����һ����ѹ������
					 userdata2[10]=baro_height;//Baro_Height_Source
					 baro_dis_delta=baro_height-baro_dis_old;
					 baro_dis_old=baro_height;//��ѹ���ٶȿ��Լ����Ż�������Ƚϴֲ�			 
		       //Moving_Average( (float)( baro_dis_delta/(8.0f*TT)),baro_speed_arr,BARO_SPEED_NUM,baro_cnt ,&baro_speed ); //��λmm/s
					 userdata1[11]=baro_speed_lpf=0.4* baro_dis_delta/(8.0f*TT);//baro_speed�������ϵ����������ֵ		 
					 baro_start_f=2;
				}	
		
				if(flag.FlightMode==ULTRASONIC_High)
				{
					 h_speed=ultra_speed;
					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//��������ֱ�ٶȻ����˲�
				}				
				else if(flag.FlightMode==ATMOSPHERE_High)
				{
					 h_speed=baro_speed_lpf;
					 wz_speed_0 = 0.99	*wz_speed_0 + 0.01*h_speed;//��ѹ�ƴ�ֱ�ٶȻ����˲���ϵ���ɵ�
				}	

				userdata1[3] =h_speed;//h_speed�Ǹ߶Ȼ������ٶȻ���ʵ��߶ȷ����ٶȡ��������Ǵ���ģ���ѹ���ٶȲ��ɿ���
				userdata1[4]=wz_speed_0;//������
				hc_speed_i += 0.4f *T *( h_speed - wz_speed );//�ٶ�ƫ����֣�������0.4ϵ��
				hc_speed_i = LIMIT( hc_speed_i, -500, 500 );//�����޷�	
				userdata1[5] =hc_speed_i;//���û��ʾ
				wz_speed_0 += ( 1 / ( 1 + 1 / ( 0.1f *3.14f *T ) ) ) *( h_speed - wz_speed_0  ) ;//0.1ʵ���ٶ��������ٶ�����ٶ�
				userdata1[6] = wz_speed=wz_speed_0 + hc_speed_i;//�����������ٶ�+�����޷�������ʽ�ٶȻ���

			  if( flag.FlightMode == ATMOSPHERE_High)
				{
						if(baro_start_f==2)//˵�����������ұ������ƶ���ֵ�˲�ʹ�ù���
						{
							baro_start_f = -1;
							Baro_Ctrl(8.0f*TT,thr_lpf);
						}	
						height_speed_ctrl(T,keepheight_thr,baro_ctrl_out,baro_speed_lpf);							
				}

				if( flag.FlightMode == ULTRASONIC_High)
				{
						height_speed_ctrl(T,keepheight_thr,ultra_ctrl_out,ultra_speed);//ϵ��ԭ����0.4
							
						if( ultra_start_f == 2 )//���������ݸ������ұ�������
						{				
								Ultra_Ctrl(15.0f*TT,thr_lpf);//realtime������Ϊ TT ��ģ�#define TT 0.0025//��������2.5ms
								ultra_start_f = -1;
						}
				}
		
	      /*******************************************���Ƹ߶����********************************************/
				if(flag.FlightMode==ULTRASONIC_High || flag.FlightMode==ATMOSPHERE_High || flag.FlightMode==ACC_High)//ע�����ģʽ
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
		ultra_wz_speed_pid.kp = 0.25;// //���������ߵ��ٶȻ�PID���ᱻEEPROM������¸�ֵ��
		ultra_wz_speed_pid.ki = 0.08; //0.12
		ultra_wz_speed_pid.kd = 8;
		baro_wz_speed_pid.kp = 0.1;// ��ѹ���ߵ��ٶȻ�PID���ᱻEEPROM������¸�ֵ��
		baro_wz_speed_pid.ki = 0.06; 
		baro_wz_speed_pid.kd = 8;//1.4*10���ڼ���ʽ������10����ɾ��
		wz_speed_pid.kp = 0.1;//�ٶȻ�Ĭ��PID�������ò��������¸�ֵ
		wz_speed_pid.ki = 0.08;
		wz_speed_pid.kd = 8;
}

void height_speed_ctrl(float T,float thr_keepheight,float exp_z_speed,float h_speed)
{
    userdata1[1] =	thr_keepheight;
		//wz_speed�����ڲ��볬�����߶�D����
		userdata1[7]=wz_speed_pid_v.err = wz_speed_pid.kp *( exp_z_speed - wz_speed );//
		userdata1[8]=wz_speed_pid_v.err_d = wz_speed_pid.kd * (-tempacc_lpf) *T;//(wz_speed_pid_v.err - wz_speed_pid_v.err_old);
		//wz_speed_pid_v.err_i += wz_speed_pid.ki *wz_speed_pid_v.err *T;
		wz_speed_pid_v.err_i += wz_speed_pid.ki *( exp_z_speed - h_speed ) *T;//�����ٶ���ʵ���ٶ�������
		wz_speed_pid_v.err_i = LIMIT(wz_speed_pid_v.err_i,-Thr_Weight *200,Thr_Weight *200);
		userdata1[9]=wz_speed_pid_v.err_i;
		wz_speed_pid_v.pid_out = thr_keepheight + Thr_Weight *LIMIT((wz_speed_pid.kp *exp_z_speed + wz_speed_pid_v.err + wz_speed_pid_v.err_d + wz_speed_pid_v.err_i),-300,300);//����ԭ��û�г�wz_speed_pid.kp
		userdata1[10]=wz_speed_pid_v.pid_out-thr_keepheight;
		wz_speed_pid_v.err_old = wz_speed_pid_v.err; 
}

void Baro_PID_Init()
{
		baro_pid.kp = 0.15;//��ѹ���ߵĸ߶�λ�û�PID���ᱻEEPROM������¸�ֵ��
		baro_pid.ki = 0;
		baro_pid.kd = 1.5;	//2.5
}

void Baro_dataporcess(float T)
{
		float baro_dis_tmp;	
		
		baro_dis_tmp = Moving_Median(2,5,baro_height);//�Գ����������ľ�������ƶ���λֵ�˲�

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
		userdata2[2]=baro_dis_lpf;//������
		userdata2[3]+= ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) *(Baro_Height_Source- userdata2[3]) ;//������	
		userdata2[4]=baro_speed;//������
}

void Baro_Ctrl(float T,float thr)
{
		exp_height_speed = BARO_SPEED *my_deathzoom_2(thr - 500,100)/400.0f; //20�����������Լ�������Ŷ������ſ��Ƹ߶�+-ULTRA_SPEEDmm / s
		exp_height_speed = LIMIT(exp_height_speed ,-BARO_SPEED,BARO_SPEED);
		
		if( exp_height > Baro_MAX_Height )//�޶��߶ȣ����Ը���ʵ���޸�
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
		
		exp_height += exp_height_speed *T;//�ۻ������߶ȣ���Ϊ�����ٶȿ��ܸı���			
		baro_ctrl.err = baro_pid.kp*(exp_height - baro_dis_lpf);//baro_dis_lpfmm �Ը߶�������Kp
		userdata2[5]=baro_ctrl.err;
		baro_ctrl.err_i += baro_pid.ki *baro_ctrl.err *T;//�Ը߶�������
		userdata2[6]=baro_ctrl.err_i = LIMIT(baro_ctrl.err_i,-Thr_Weight *BARO_INT,Thr_Weight *BARO_INT);//�����޷����ں�������Ȩ��
		//����D���ں��˼��ٶȼ�����õ��ľ���	
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
		ultra_pid.kp = 0.5;//���������ߵĸ߶�λ�û�PID���ᱻEEPROM������¸�ֵ��
		ultra_pid.ki = 0;
		ultra_pid.kd = 2.5;	//2.5
}

void Ultra_dataporcess(float T)
{
		float ultra_sp_tmp,ultra_dis_tmp;	
		ultra_dis_tmp = Moving_Median(1,5,US100_Alt);//�Գ����������ľ�������ƶ���λֵ�˲�
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
		
		//ע�ⳬ���������� �����ʱЧ�����⣬���ⷴ��������
		ultra_sp_tmp = Moving_Median(0,5,US100_Alt_delta/T); //�Գ���������������֮�����������ٶ���ֵ�˲� ultra_delta/T;

		if( ultra_dis_tmp < Ultrasonic_MAX_Height )//С��3��,ע�������һ�������أ�
		{
			if( ABS(ultra_sp_tmp) < 100 )//�˶��ٶ�С��100mm/s
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 4 *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ultra_speed�ᴫ�ݸ��ٶȻ�����Ϊ��ǰ�ٶȵķ���,ԭ����4
			else
			{
				ultra_speed += ( 1 / ( 1 + 1 / ( 1.0f *3.14f *T ) ) ) * ( (float)(ultra_sp_tmp) - ultra_speed );
			}//ϵ��Խ������ԽС��ԭ����1
		}
}

void Ultra_Ctrl(float T,float thr)
{
		exp_height_speed = ULTRA_SPEED *my_deathzoom_2(thr - 500,100)/300.0f; //20�����������Լ�������Ŷ������ſ��Ƹ߶�+-ULTRA_SPEEDmm / s
		exp_height_speed = LIMIT(exp_height_speed ,-ULTRA_SPEED,ULTRA_SPEED);
		
		if( exp_height > Ultrasonic_MAX_Height )//�����������߶��ȶ���Χ��ִ��
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
		
		exp_height += exp_height_speed *T;//�ۻ������߶ȣ���Ϊ�����ٶȿ��ܸı���

		ultra_ctrl.err = ( ultra_pid.kp*(exp_height - ultra_dis_lpf) );//mm �Ը߶�������Kp
		userdata2[5]=ultra_ctrl.err;
		ultra_ctrl.err_i += ultra_pid.ki *ultra_ctrl.err *T;//�Ը߶�������
		userdata2[6]=ultra_ctrl.err_i = LIMIT(ultra_ctrl.err_i,-Thr_Weight *ULTRA_INT,Thr_Weight *ULTRA_INT);//�����޷����ں�������Ȩ��
					//����D���ں��˼��ٶȼ�����õ��ľ���	
		userdata2[7]=0.4f *(ultra_ctrl.err_old - ultra_ctrl.err);
		ultra_ctrl.err_d = ultra_pid.kd *( 0.5f *(-wz_speed*T) + 0.5f *(ultra_ctrl.err - ultra_ctrl.err_old) );
		userdata2[8]=ultra_ctrl.err_d;
		ultra_ctrl.pid_out = ultra_ctrl.err + ultra_ctrl.err_i + ultra_ctrl.err_d;	
		ultra_ctrl.pid_out = LIMIT(ultra_ctrl.pid_out,-500,500);
		
		if(flag.Special_Mode==ACC_High)ultra_ctrl.pid_out=0;//������������ڲ��Լ��ٶȼƶ���Ч��,����Ȼ���ٶȷ���	
		userdata2[9]=ultra_ctrl_out = ultra_ctrl.pid_out;
		ultra_ctrl.err_old = ultra_ctrl.err;
}

void LockForKeepHigh(float THROTTLE)
{
    if (flag.FlightMode==ULTRASONIC_High && lock==0)
		{
			  wz_speed_pid.kp=ultra_wz_speed_pid.kp;//��PID
		    wz_speed_pid.ki=ultra_wz_speed_pid.ki;
		    wz_speed_pid.kd=ultra_wz_speed_pid.kd;
        exp_height=ultra_dis_lpf;//����������߶�ֵ�����˲�����
			  wz_acc=0;
			  wz_speed_pid_v.err_i=0;
				ultra_speed=0;//�������㣬���ҳ�����ʱԭ���ۻ����ٶȻ���󴫵ݸ���ǰ����ɵ���
			  ultra_ctrl.err_i=0;
				ultra_ctrl.err_old=0;
			  lock=1;
    }
		else if(flag.FlightMode==MANUAL_High || flag.FlightMode==ATMOSPHERE_High)
		{lock=0;}
						
		if (flag.FlightMode==ATMOSPHERE_High && lock_BARO==0)
		{
			  wz_speed_pid.kp=baro_wz_speed_pid.kp;//��PID
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
