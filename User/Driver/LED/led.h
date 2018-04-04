#ifndef __LED_H
#define	__LED_H

#include "stm32f10x.h"


/* the macro definition to trigger the led on or off 
 * 1 - off
 - 0 - on
 */
#define ON  0
#define OFF 1

#define GPIO_LED	     GPIOD
#define LED_R          GPIO_Pin_0
#define LED_G          GPIO_Pin_1
#define LED_B          GPIO_Pin_2
#define RCC_GPIO_LED	 RCC_APB2Periph_GPIOD

#define GPIOE_LED	     GPIOE
#define LED_BLUE_1     GPIO_Pin_12
#define LED_BLUE_2     GPIO_Pin_10
#define LED_BLUE_3     GPIO_Pin_14
#define LED_BLUE_4     GPIO_Pin_15
				
#define Ledr_off       GPIO_SetBits(GPIO_LED, LED_R)
#define Ledr_on        GPIO_ResetBits(GPIO_LED, LED_R)

#define Ledg_off       GPIO_SetBits(GPIO_LED, LED_G)
#define Ledg_on        GPIO_ResetBits(GPIO_LED, LED_G)

#define Ledb_off       GPIO_SetBits(GPIO_LED, LED_B)
#define Ledb_on        GPIO_ResetBits(GPIO_LED, LED_B)
			
#define LED_ALLON()		 GPIO_ResetBits(GPIO_LED, LED_R | LED_G | LED_B)			
#define LED_ALLOFF()	 GPIO_SetBits(GPIO_LED, LED_R | LED_G | LED_B)				
					
#define LED_NUM 3
#define LR	    0x01
#define LG      0x02
#define LB      0x04


#define LED_BLUE1_OFF    GPIO_SetBits(GPIOE_LED, LED_BLUE_1)
#define LED_BLUE1_ON     GPIO_ResetBits(GPIOE_LED, LED_BLUE_1)

#define LED_BLUE2_OFF    GPIO_SetBits(GPIOE_LED, LED_BLUE_2)
#define LED_BLUE2_ON     GPIO_ResetBits(GPIOE_LED, LED_BLUE_2)

#define LED_BLUE3_OFF    GPIO_SetBits(GPIOB, LED_BLUE_3)
#define LED_BLUE3_ON     GPIO_ResetBits(GPIOB, LED_BLUE_3)

#define LED_BLUE4_OFF    GPIO_SetBits(GPIOE_LED, LED_BLUE_4)
#define LED_BLUE4_ON     GPIO_ResetBits(GPIOE_LED, LED_BLUE_4)

enum {
    Ht_ARMED = 0,  
    Ht_DISARMED,   
    Ht_CALIBRATA,  
	  Ht_CALIBRATG,  
    Ht_CALIBRATM_X,
    Ht_CALIBRATM_Y,
	  Ht_CALIBRATM_Z,
};


typedef union{
	uint8_t byte;
	struct 
	{
			uint8_t R	:1;
		  uint8_t G	:1;
			uint8_t B	:1;
	}bits;
}LEDBuf_t;

		
typedef struct Led
{
  u8 event;
  u8 state;
  u16 cnt;
}led_Fsm;					
	
extern led_Fsm LED;


void LED_GPIO_Config(void);
void LED_SHOW(void);
void LED_BLUE_SHOW(void);
void LED_Running(int tim);
void LED_Sailing(int rate);
void FailSafeLEDAlarm(void);



#endif /* __LED_H */
