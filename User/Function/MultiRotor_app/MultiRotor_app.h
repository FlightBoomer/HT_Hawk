#ifndef _MultiRotor_app_H_
#define _MultiRotor_app_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


typedef struct {
	      u8 MpuExist;      // MPU存在
	      u8 MagExist;      // MAG存在
	      u8 NrfExist;      // NRF存在
	      u8 MagIssue;      // MAG有问题
          u8 MsExist;
        u8 ARMED;         // 电机解锁
	      u8 LockYaw;       // 航向锁定       
        u8 calibratingA;  // 加速度采集
	      u8 calibratingM;  // 磁力计采集
	      u8 calibratingM_pre; //磁力计预采集
	      u8 calibratingG;
	      u8 ParamSave;     // 参数保存标志
	
	      u8 Loop_200Hz;
	      u8 Loop_100Hz;
				u8 Loop_40Hz;
				u8 Loop_27Hz;
				u8 Loop_20Hz;
	      u8 Loop_10Hz;
        u8 fortest;//加来临时用
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



