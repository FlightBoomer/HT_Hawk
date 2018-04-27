/******************* (C) COPYRIGHT 2016 Hentotech Team ***************************
 * �ļ���   ��main.c
 * ����     ��ϵͳ��ʼ��         
 * ʵ��ƽ̨ ��HT-Hawk��Դ�ɿ�
 * ��汾   ��ST3.5.0
 * ����     ��Hentotech Team 
 * ����     ��http://www.hentotech.com 
 * ��̳     ��http://bbs.hentotech.com
 * �̳�     ��http://hentotech.taobao.com   
*********************************************************************************/
/*********************************************************************************
1��������ģ����߷��������鳬����ģ�鶨�߸߶Ȳ�����2.5M��
             VCC---5V
             ECHO--CH7��PB0��
             TRIG--CH8��PB1��
						 GND---GND
						 
2��ң����CH5ͨ�����Ʒ���ģʽ��OLED��ʾ����ʵʱ��ʾ����ģʽ��  
             ����ģʽ--------Stabilize
						 ����������ģʽ--U-ALTHOLD
						 ��ѹ�ƶ���ģʽ--B-ALTHOLD
						 
3��ֻ��������ģʽ�½���������

4����������г�ͳһУ׼����
             1)���Ӻõ�����ɿ�����ջ�����ʱ��Ҫ���������ϵ磻
             2)��ң��������ң��������ҡ���Ƶ����
						 3)���Ϸ�������Դ����ʱ������������εΡ�������������OLED��ʾ������ʾ�����У׼����
						 4)��ʱ������ң��������ҡ���Ƶ���С���������ε�--�Ρ�����������
						 5)�ƶ�����ҡ�˵����ת����������У׼��ϣ���Ҫ�����ɿؽ�������ģʽ��
   
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
*	�� �� ��: main
*	����˵��: c�������
*	��    �Σ���
*	�� �� ֵ: ��
***********************************************************************************************************/
int main(void)
{
		// Bootloader�������
		Bootloader_Set();
		
		// Ӳ����ʼ��
		InitBoard();	

		// ��������ʼ��
		Sensor_Init();
		
		// ���ز���
		paramLoad();
	
        // EKF2����
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
        
	
		// ����ʱ5ʱ��Ƭ�ж�
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
