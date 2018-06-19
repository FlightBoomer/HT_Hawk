#include "Multirotor_ahrs.h"

float acc[3]  = { 0. };
float gyro[3] = { 0. };
float mag[3]  = { 0. };
float X[7]    = { 1.,  0.,  0.,  0.,  0.,  0.,  0., };
float P[49]   = {  1e6, 0.,  0.,  0.,  0.,  0.,  0., 
                   0.,  1e6, 0.,  0.,  0.,  0.,  0.,
                   0.,  0.,  1e6, 0.,  0.,  0.,  0.,
                   0.,  0.,  0.,  1e6, 0.,  0.,  0.,
                   0.,  0.,  0.,  0.,  1e6, 0.,  0.,
                   0.,  0.,  0.,  0.,  0.,  1e6, 0.,
                   0.,  0.,  0.,  0.,  0.,  0.,  1e6,  };

float Q[49]   = {  1e-3, 0.,   0.,   0.,   0.,   0.,   0., 
                   0.,   1e-3, 0.,   0.,   0.,   0.,   0.,
                   0.,   0.,   1e-3, 0.,   0.,   0.,   0.,
                   0.,   0.,   0.,   1e-3, 0.,   0.,   0.,
                   0.,   0.,   0.,   0.,   1e-6, 0.,   0.,
                   0.,   0.,   0.,   0.,   0.,   1e-6, 0.,
                   0.,   0.,   0.,   0.,   0.,   0.,   1e-6,  };

float R[36]   = {  3e-1, 0.,   0.,   0.,   0.,   0., 
                   0.,   3e-1, 0.,   0.,   0.,   0.,
                   0.,   0.,   3e-1, 0.,   0.,   0.,
                   0.,   0.,   0.,   2e-1, 0.,   0., 
                   0.,   0.,   0.,   0.,   2e-1, 0.,
                   0.,   0.,   0.,   0.,   0.,   2e-1,  };

float roll_qkf = 0., pitch_qkf = 0., yaw_qkf = 0.;
float dt_qkf = 0.01;            // 10ms
                   
                   
void update_IMU(void);
void refine_Result(void);
                   
void AHRS_get_Euler_QKF(void) {
    
    AHRS_getValues();
    update_IMU();
    
    qkf_run(X, P, acc, gyro, mag, dt_qkf, Q, R);
    
    refine_Result();
    
    IMU.Roll  = roll_qkf;
    IMU.Pitch = pitch_qkf;
    IMU.Yaw   = yaw_qkf;    
}

void AHRS_init(void) {
    
    float q0, q1, q2, q3;
    float bwx, bwy, bwz;
    float wx,  wy,  wz;
    
    const float dt = dt_qkf;
    
    qkf_initialize();
    
    delay_ms(50);
    
    // Ԥװ��ֵ
    AHRS_getValues();
    update_IMU();
    
    q0 = 1; q1 = 0; q2 = 0; q3 = 0;
    wx = gyro[0]; wy = gyro[0]; wz = gyro[0];
    bwx = 0;      bwy = 0;      bwz = 0;
    X[0] =  q0 - (q1*(wx-bwx)*dt)/2 - (q2*(wy - bwy)*dt)/2 - (q3*(wz - bwz)*dt)/2;
    X[1] =  q1 + (q0*(wx-bwx)*dt)/2 - (q3*(wy - bwy)*dt)/2 + (q2*(wz - bwz)*dt)/2;
    X[2] =  q2 + (q3*(wx-bwx)*dt)/2 + (q0*(wy - bwy)*dt)/2 - (q1*(wz - bwz)*dt)/2;
    X[3] =  q3 - (q2*(wx-bwx)*dt)/2 + (q1*(wy - bwy)*dt)/2 + (q0*(wz - bwz)*dt)/2;
    X[4] =                                                                    bwx;
    X[5] =                                                                    bwy;
    X[6] =                                                                    bwz;
}


void update_IMU(void) {

    const float d2r = PI / 180.0f;
    const float _4g = 4 * 9.8;
    
    float norm_temp = 0;
    
/// ���ٶ�
	acc[0] = sensor.acc.averag.x / 32768. * _4g;        // acc[0] = -sensor.acc.averag.x / 32768. * _4g;
	acc[1] = sensor.acc.averag.y / 32768. * _4g;        // acc[1] = -sensor.acc.averag.y / 32768. * _4g;
	acc[2] = sensor.acc.averag.z / 32768. * _4g;        // acc[2] = -sensor.acc.averag.z / 32768. * _4g;
	// ���ٶȹ�һ��
    norm_temp = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
    acc[0] /= norm_temp;
    acc[1] /= norm_temp;
    acc[2] /= norm_temp;

// ������    
 	gyro[0] = sensor.gyro.averag.x * d2r;
 	gyro[1] = sensor.gyro.averag.y * d2r;
 	gyro[2] = sensor.gyro.averag.z * d2r;
    
// �ش�
	mag[0] = MAG[0];
	mag[1] = MAG[1];
	mag[2] = MAG[2];
	// ���ٶȹ�һ��
    norm_temp = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
    mag[0] /= norm_temp;
    mag[1] /= norm_temp;
    mag[2] /= norm_temp;
}

void refine_Result(void) {
    
    const float r2d = 180. / PI;

    float q_data[4] = { 0 };
    int   q_size[2] = { 0 };
    
    q_data[0] = X[0];
    q_data[1] = X[1];
    q_data[2] = X[2];
    q_data[3] = X[3];
    
    my_quat2att(q_data, q_size, &roll_qkf, &pitch_qkf, &yaw_qkf);
    roll_qkf  *= r2d;
    pitch_qkf *= r2d;
    yaw_qkf   *= r2d;
    
// �������������ֻ�ܸ���������ж���û���ɵķ���
    roll_qkf += 180.0f;
    while (roll_qkf >= 180.0f)
        roll_qkf -= 360.0f; 
//    yaw_qkf = -yaw_qkf;
    
// �����¼�: �ɿ�ͨ����ع����Ǽ��ٶ�Ҫȡ���ŵ�
    // ����Ƕ��������Ը����ٶ�����ӷ��ţ���ʹ������Ĵ���
    // �����������ע�͵Ĵ���
    pitch_qkf = -pitch_qkf;             // ����ͷ������бpitchΪ��
    
// ƫ��������
    while (yaw_qkf >= 180.0f)
        yaw_qkf -= 360.0f;
    while (yaw_qkf <= -180.0f)
        yaw_qkf += 360.0f;
// �����ͷָ������ȴ��ʾ-180����ִ������2�С������뱣֤��ͷָ������ʱ�Ƕ�Ϊ0,���ң���˳ʱ��תΪ��ֵ    
    yaw_qkf += 180.0f;
    while (yaw_qkf >= 180.0f)
        yaw_qkf -= 360.0f;
    
    yaw_qkf = -yaw_qkf;
}
