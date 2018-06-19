#ifndef __Ultrasonic_H_
#define __Ultrasonic_H_

#include "include.h"


extern float US100_Alt;
extern float US100_Alt_V;

extern float g_HightControl;
extern float US100_Alt;
extern float US100_Alt_delta,US100_Alt_old;
void Ultrasonic_Config(void);
void Ultrasonic_Pulsing(void);

#endif // __Ultrasonic_H__
