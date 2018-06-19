#ifndef _Algorithm_sqLite_H_
#define _Algorithm_sqLite_H_
/* Includes ------------------------------------------------------------------*/
#include "include.h"


//存储外环和内环PID调控
#define EE_PID_CORE_PITCH_P	9
#define EE_PID_CORE_PITCH_I	10
#define EE_PID_CORE_PITCH_D	11
#define EE_PID_CORE_ROLL_P	12
#define EE_PID_CORE_ROLL_I	13
#define EE_PID_CORE_ROLL_D	14
#define EE_PID_CORE_YAW_P	  15
#define EE_PID_CORE_YAW_I	  16
#define EE_PID_CORE_YAW_D	  17

#define EE_PID_ULTRA_SPEED_Z_P	  20
#define EE_PID_ULTRA_SPEED_Z_I	  21
#define EE_PID_ULTRA_SPEED_Z_D	  22
#define EE_PID_ULTRA_P	  23
#define EE_PID_ULTRA_I	  24
#define EE_PID_ULTRA_D	  25
#define EE_PID_BARO_P	  26
#define EE_PID_BARO_I	  27
#define EE_PID_BARO_D	  28
#define EE_PID_BARO_SPEED_Z_P	  29
#define EE_PID_BARO_SPEED_Z_I	  30
#define EE_PID_BARO_SPEED_Z_D	  31

extern uint16_t VirtAddVarTab[NumbOfVar];

void EE_SAVE_ACC_OFFSET(void);
void EE_SAVE_MAG_OFFSET(void);
void EE_SAVE_Attitude_PID(void);
void paramLoad(void);
void Data_Parser(u16 *rxBuffer);
#endif /* __Algorithm_sqLite_H */



