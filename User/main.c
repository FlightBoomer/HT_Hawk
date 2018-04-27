/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * 文件名   ：main.c
 * 描述     ：系统初始化         
 * 实验平台 ：HT-Hawk开源飞控
 * 库版本   ：ST3.5.0
 * 作者     ：Hentotech Team 
 * 官网     ：http://www.hentotech.com 
 * 论坛     ：http://bbs.hentotech.com
 * 商城     ：http://hentotech.taobao.com   
*********************************************************************************/
/*********************************************************************************
1、超声波模块接线方法（建议超声波模块定高高度不超过2.5M）
             VCC---5V
             ECHO--CH7（PB0）
             TRIG--CH8（PB1）
						 GND---GND
						 
2、遥控器CH5通道控制飞行模式（OLED显示屏会实时显示飞行模式）  
             自稳模式--------Stabilize
						 超声波定高模式--U-ALTHOLD
						 气压计定高模式--B-ALTHOLD
						 
3、只能在自稳模式下解锁飞行器

4、电调油门行程统一校准步骤
             1)连接好电调、飞控与接收机，此时不要给飞行器上电；
             2)打开遥控器，将遥控器油门摇杆推到最大；
						 3)插上飞行器电源，此时会听到电机“滴滴”的声音，并且OLED显示屏会显示“电调校准”；
						 4)此时立即将遥控器油门摇杆推到最小，听到“滴滴--滴”的音乐声；
						 5)推动油门摇杆电机会转动，代表电调校准完毕，需要重启飞控进入正常模式；
   
*********************************************************************************/
#include "board_config.h"

#include <AttitudeEKF.h>

struct attitude_estimator_ekf_params {
	float r[3];
	float q[4];
	float moment_inertia_J[9];
	int32_t use_moment_inertia;
	float roll_off;
	float pitch_off;
	float yaw_off;
	float mag_decl;
	int acc_comp;
};

/**********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 无
***********************************************************************************************************/
int main(void)
{
		// Bootloader相关配置
		Bootloader_Set();
		
		// 硬件初始化
		InitBoard();	

		// 传感器初始化
		Sensor_Init();
		
		// 加载参数
		paramLoad();
	
        // EKF2测试
        float dt = 0.005f;
    /* state vector x has the following entries [ax,ay,az||mx,my,mz||wox,woy,woz||wx,wy,wz]' */
        float z_k[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 9.81f, 0.2f, -0.2f, 0.2f};					/**< Measurement vector */
        float x_aposteriori_k[12];		/**< states */
        float P_aposteriori_k[144] = {100.f, 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                         0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                         0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                         0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,   0,   0,
                         0,   0,   0,   0,  100.f,  0,   0,   0,   0,   0,   0,   0,
                         0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,   0,
                         0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,   0,
                         0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,   0,
                         0,   0,   0,   0,   0,   0,   0,   0, 100.f,   0,   0,   0,
                         0,   0,   0,   0,   0,   0,   0,   0,  0.0f, 100.0f,   0,   0,
                         0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   100.0f,   0,
                         0,   0,   0,   0,   0,   0,   0,   0,  0.0f,   0,   0,   100.0f,
                        }; /**< init: diagonal matrix with big values */

        float x_aposteriori[12];
        float P_aposteriori[144];

        /* output euler angles */
        float euler[3] = {0.0f, 0.0f, 0.0f};

        float Rot_matrix[9] = {1.f,  0,  0,
                      0,  1.f,  0,
                      0,  0,  1.f
                     };		/**< init: identity matrix */

        float debugOutput[4] = { 0.0f };
        
        uint8_t update_vect[3] = {0, 0, 0};

        struct attitude_estimator_ekf_params ekf_params;
                    ekf_params.q[0] = 1e-4f;
            ekf_params.q[1] = 0.08f;
            ekf_params.q[2] = 0.009f;
            ekf_params.q[3] = 0.005f;
            ekf_params.r[0] = 0.0008f;
            ekf_params.r[1] = 10000.0f;
            ekf_params.r[2] = 100.0f;
            ekf_params.use_moment_inertia = 0;
        
        /* Initialize filter */
        AttitudeEKF_initialize();
        
	
		// 开定时5时间片中断
		EnTIMER;
	
		while(1)
		{
			loop();
            
            
            /*
            		perf_begin(ekf_loop_perf);

					// Calculate data time difference in seconds 
					dt = (raw.timestamp - last_measurement) / 1000000.0f;
					last_measurement = raw.timestamp;
					uint8_t update_vect[3] = {0, 0, 0};

					// Fill in gyro measurements
					if (sensor_last_timestamp[0] != raw.gyro_timestamp[0]) {
						update_vect[0] = 1;
						// sensor_update_hz[0] = 1e6f / (raw.timestamp - sensor_last_timestamp[0]);
						sensor_last_timestamp[0] = raw.gyro_timestamp[0];
					}

					z_k[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
					z_k[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
					z_k[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];

					// update accelerometer measurements 
					if (sensor_last_timestamp[1] != raw.accelerometer_timestamp[0]) {
						update_vect[1] = 1;
						// sensor_update_hz[1] = 1e6f / (raw.timestamp - sensor_last_timestamp[1]);
						sensor_last_timestamp[1] = raw.accelerometer_timestamp[0];
					}
            */
           
           	z_k[0] =  raw.gyro_rad_s[0] - gyro_offsets[0];
			z_k[1] =  raw.gyro_rad_s[1] - gyro_offsets[1];
			z_k[2] =  raw.gyro_rad_s[2] - gyro_offsets[2];
            z_k[3] = raw.accelerometer_m_s2[0] - acc(0);
			z_k[4] = raw.accelerometer_m_s2[1] - acc(1);
			z_k[5] = raw.accelerometer_m_s2[2] - acc(2);
			z_k[6] = raw.magnetometer_ga[0];
			z_k[7] = raw.magnetometer_ga[1];
			z_k[8] = raw.magnetometer_ga[2];            
            
            /* Call the estimator */
			AttitudeEKF(false, // approx_prediction
					(unsigned char)ekf_params.use_moment_inertia,
					update_vect,
					dt,
					z_k,
					ekf_params.q[0], // q_rotSpeed,
					ekf_params.q[1], // q_rotAcc
					ekf_params.q[2], // q_acc
					ekf_params.q[3], // q_mag
					ekf_params.r[0], // r_gyro
					ekf_params.r[1], // r_accel
					ekf_params.r[2], // r_mag
					ekf_params.moment_inertia_J,
					x_aposteriori,
					P_aposteriori,
					Rot_matrix,
					euler,
					debugOutput);

		}			
}
