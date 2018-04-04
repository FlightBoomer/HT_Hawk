/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：MultiRotor_ahrs.c
 * 描述     ：姿态解算    
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
#include "board_config.h"
#include "MultiRotor_ahrs.h"
#include "MPU6050.h"

#define KpDef 0.8f
#define KiDef 0.0005f
#define SampleRateHalf 0.00125f  //0.001

#define  IIR_ORDER     4      //使用IIR滤波器的阶数
double b_IIR[IIR_ORDER+1] ={0.0008f, 0.0032f, 0.0048f, 0.0032f, 0.0008f};  //系数b
double a_IIR[IIR_ORDER+1] ={1.0000f, -3.0176f, 3.5072f, -1.8476f, 0.3708f};//系数a
double InPut_IIR[3][IIR_ORDER+1] = {0};
double OutPut_IIR[3][IIR_ORDER+1] = {0};
fp32 yawangle;
u16 testtime;


// /*	
// 	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
// 	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
// */
// #define KALMAN_Q        0.02
// #define KALMAN_R        8.0000

Quaternion NumQ = {1, 0, 0, 0};
EulerAngle AngE = {0},IMU = {0};


int16_t MAG[3];
Gravity V;//重力分量


void AHRS_getValues(void)
{
	static float x,y,z;
	
	MPU6050_Dataanl();
	
	HMC5883lRead(MAG);//260us
	
	// 加速度计IIR滤波
	sensor.acc.averag.x = IIR_I_Filter(sensor.acc.origin.x, InPut_IIR[0], OutPut_IIR[0], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.y = IIR_I_Filter(sensor.acc.origin.y, InPut_IIR[1], OutPut_IIR[1], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	sensor.acc.averag.z = IIR_I_Filter(sensor.acc.origin.z, InPut_IIR[2], OutPut_IIR[2], b_IIR, IIR_ORDER+1, a_IIR, IIR_ORDER+1);
	
	// 陀螺仪一阶低通滤波
 	sensor.gyro.averag.x = LPF_1st(x,sensor.gyro.radian.x * Gyro_G,0.386f);	x = sensor.gyro.averag.x;
 	sensor.gyro.averag.y = LPF_1st(y,sensor.gyro.radian.y * Gyro_G,0.386f);	y = sensor.gyro.averag.y;
 	sensor.gyro.averag.z = LPF_1st(z,sensor.gyro.radian.z * Gyro_G ,0.386f);	z = sensor.gyro.averag.z;//
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_Update
**功能 : AHRS
**输入 : None
**输出 : None
**使用 : AHRS_Update();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_GetQ( Quaternion *pNumQ )
{
  fp32 ErrX, ErrY, ErrZ;
  fp32 AccX, AccY, AccZ;
  fp32 GyrX, GyrY, GyrZ;
	fp32 Normalize;
  static fp32 exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;

	
	// 加速度归一化
	Normalize = Q_rsqrt(squa(sensor.acc.averag.x)+ squa(sensor.acc.averag.y) +squa(sensor.acc.averag.z));
	AccX = sensor.acc.averag.x*Normalize;
  AccY = sensor.acc.averag.y*Normalize;
  AccZ = sensor.acc.averag.z*Normalize;

	// 提取重力分量
	V = Quaternion_vectorGravity(&NumQ);
	
	// 向量差乘
 	ErrX = (AccY*V.z - AccZ*V.y);
  ErrY = (AccZ*V.x - AccX*V.z);
  ErrZ = (AccX*V.y - AccY*V.x);
 	
 	exInt = exInt + ErrX * KiDef;
  eyInt = eyInt + ErrY * KiDef;
  ezInt = ezInt + ErrZ * KiDef;

  GyrX = Rad(sensor.gyro.averag.x) + KpDef * VariableParameter(ErrX) * ErrX  +  exInt;
  GyrY = Rad(sensor.gyro.averag.y) + KpDef * VariableParameter(ErrY) * ErrY  +  eyInt;
	GyrZ = Rad(sensor.gyro.averag.z) + KpDef * VariableParameter(ErrZ) * ErrZ  +  ezInt;
	
	
	// 一阶龙格库塔法, 更新四元数
	Quaternion_RungeKutta(&NumQ, GyrX, GyrY, GyrZ, SampleRateHalf);
	
	// 四元数归一化
	Quaternion_Normalize(&NumQ);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : AHRS_Geteuler
**功能 : AHRS获取欧拉角
**输入 : None
**输出 : None
**使用 : AHRS_Geteuler();
**====================================================================================================*/
/*====================================================================================================*/
void AHRS_Geteuler(void)
{
	fp32 sin_pitch,sin_roll,cos_roll,cos_pitch;
	
	AHRS_getValues();
	
	// 获取四元数
  AHRS_GetQ(&NumQ);
	
  // 四元数转欧拉角
	Quaternion_ToAngE(&NumQ, &AngE);
	
  // 计算欧拉角的三角函数值
  sin_roll  = sin(AngE.Roll);
  sin_pitch = sin(AngE.Pitch);
  cos_roll  = cos(AngE.Roll);
  cos_pitch = cos(AngE.Pitch);
	
	//  地磁不存在或地磁数据不正常则停用地磁数据
  //flag.MagIssue=0;//地磁存在问题，待调试解决
  //flag.MagExist=0;

	if(!flag.MagIssue && flag.MagExist){//40US
		// 地磁倾角补偿
		fp32 hx = MAG[0]*cos_pitch + MAG[1]*sin_pitch*sin_roll - MAG[2]*cos_roll*sin_pitch; 
		fp32 hy = MAG[1]*cos_roll + MAG[2]*sin_roll;
		
		// 利用地磁解算航向角
		fp32 mag_yaw = -Degree(atan2((fp64)hy,(fp64)hx));
		 yawangle=mag_yaw;
		// 陀螺仪积分解算航向角
//		AngE.Yaw += Degree(sensor.gyro.averag.z * Gyro_Gr * 2 * SampleRateHalf);//重大bug，已经是角度，这里不能再乘以Gyro_Gr了
		AngE.Yaw += sensor.gyro.averag.z * 2  * SampleRateHalf;
		// 地磁解算的航向角与陀螺仪积分解算的航向角进行互补融合 
		if((mag_yaw>90 && IMU.Yaw<-90) || (mag_yaw<-90 && IMU.Yaw>90)) 
			IMU.Yaw = -IMU.Yaw * 0.988f + mag_yaw * 0.012f;
		else 
			IMU.Yaw = IMU.Yaw * 0.988f + mag_yaw * 0.012f;
	}
	else 
		IMU.Yaw = Degree(AngE.Yaw);

  IMU.Roll = Degree(AngE.Roll);  // roll
	IMU.Pitch = Degree(AngE.Pitch); // pitch
}
