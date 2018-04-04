#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


typedef struct {
	      u8 MpuExist;      // MPU����
	      u8 MagExist;      // MAG����
	      u8 NrfExist;      // NRF����
	      u8 MagIssue;      // MAG������
          u8 MsExist;
        u8 ARMED;         // �������
	      u8 LockYaw;       // ��������       
        u8 calibratingA;  // ���ٶȲɼ�
	      u8 calibratingM;  // �����Ʋɼ�
	      u8 calibratingM_pre; //������Ԥ�ɼ�
	      u8 calibratingG;
	      u8 ParamSave;     // ���������־
	
	      u8 Loop_200Hz;
	      u8 Loop_100Hz;
				u8 Loop_40Hz;
				u8 Loop_27Hz;
				u8 Loop_20Hz;
	      u8 Loop_10Hz;
        u8 fortest;//������ʱ��
				u8 FlightMode;
				u8 HUDMode;
				u8 ControlMode;
				u8 Special_Mode;
         }Flag_t;

extern Flag_t flag;
extern u16 testtime;
extern u8 NRFRXOK,RXstate;
extern uint8 RXBUF[32];
extern  u16 tempnum;
extern double KalmanFilter(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);		 
void loop(void);
void Bootloader_Set(void);
void InitBoard(void);
void Sensor_Init(void);
void OLED_Display(void);
void Screen_Update(void);
void Time_slice(void);
#endif /* __MultiRotor_app_H */



