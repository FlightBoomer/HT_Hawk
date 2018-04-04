#ifndef _MultiRotor_rc_H_
#define _MultiRotor_rc_H_
#include "stm32f10x.h"
typedef struct {
	      int16_t rc_data[8];
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
				int16_t SENSITIVITY;
				int16_t KEEPHIGH;}RC_GETDATA;

/*********** RC alias *****************/
enum {
    ROLL = 0,
    PITCH,
    YAW,
    THROTTLE,
};

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))


#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))
				
				
#define CH5_LO (1 << (2 * 0))
#define CH5_CE (3 << (2 * 0))
#define CH5_HI (2 << (2 * 0))
#define CH6_LO (1 << (2 * 1))
#define CH6_CE (3 << (2 * 1))
#define CH6_HI (2 << (2 * 1))
#define CH7_LO (1 << (2 * 2))
#define CH7_CE (3 << (2 * 2))
#define CH7_HI (2 << (2 * 2))
#define CH8_LO (1 << (2 * 3))
#define CH8_CE (3 << (2 * 3))
#define CH8_HI (2 << (2 * 3))
				
				
extern  RC_GETDATA RC_Data;
extern  u8 accCorrect_flag;
extern  u8 turn_flag; 
extern u8 keephigh;
extern u8 lockpressrue;
void RC_Analy(void);
void RC_directive(void);
void RDAU(void);
void RC_Data_Refine(void);
void ECS_Calibrate(void);
#endif
