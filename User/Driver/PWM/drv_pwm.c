/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */
#include "include.h"
#include "drv_pwm.h"



typedef struct {
    volatile uint16_t *ccr;
    volatile uint16_t *cr1;
    volatile uint16_t *cnt;
    uint16_t period;

    // for input only
    uint8_t channel;  //通道
    uint8_t state;    //状态
    uint16_t rise;    //上升
    uint16_t fall;    //下降
    uint16_t capture; //捕获
} pwmPortData_t;

enum {
    TYPE_IP = 0x10,
    TYPE_IW = 0x20,
    TYPE_M = 0x40,
    TYPE_S = 0x80
};

typedef void (*pwmWriteFuncPtr)(uint8_t index, uint16_t value);  // function pointer used to write motors

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
static pwmPortData_t *motors[MAX_MOTORS];
static pwmWriteFuncPtr pwmWritePtr = NULL;
static uint8_t numMotors = 0;  //电机数量
static uint8_t numInputs = 0;  //输入通道数量
static uint8_t pwmFilter = 0;  
static bool syncPWM = FALSE;
static uint16_t failsafeThreshold = 985;
// external vars (ugh)
extern int16_t failsafeCnt;



static const uint8_t multiPPM[] = {
    PWM1 | TYPE_IP,     // PPM input
    PWM9 | TYPE_M,      // Swap to servo if needed
    PWM10 | TYPE_M,     // Swap to servo if needed
    PWM11 | TYPE_M,
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,
    PWM5 | TYPE_M,      // Swap to servo if needed
    PWM6 | TYPE_M,      // Swap to servo if needed
    PWM7 | TYPE_M,      // Swap to servo if needed
    PWM8 | TYPE_M,      // Swap to servo if needed
    0xFF
};

static const uint8_t multiPWM[] = {
    PWM1 | TYPE_IW,     // input #1
    PWM2 | TYPE_IW,
    PWM3 | TYPE_IW,
    PWM4 | TYPE_IW,
    PWM5 | TYPE_IW,
    PWM6 | TYPE_IW,
    PWM7 | TYPE_IW,
    PWM8 | TYPE_IW,     // input #8
    PWM9 | TYPE_M,      // motor #1 or servo #1 (swap to servo if needed)
    PWM10 | TYPE_M,     // motor #2 or servo #2 (swap to servo if needed)
    PWM11 | TYPE_M,     // motor #1 or #3
    PWM12 | TYPE_M,
    PWM13 | TYPE_M,
    PWM14 | TYPE_M,     // motor #4 or #6
    0xFF
};

static const uint8_t *const hardwareMaps[] = {
    multiPWM,
    multiPPM,
};

#define PWM_TIMER_MHZ 1
#define PWM_TIMER_8_MHZ 8


/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmOCConfig
**功能 : PWM输出配置
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    uint16_t tim_oc_preload;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = value;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

    if (syncPWM)
        tim_oc_preload = TIM_OCPreload_Disable;
    else
        tim_oc_preload = TIM_OCPreload_Enable;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, tim_oc_preload);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, tim_oc_preload);
            break;
    }
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmICConfig
**功能 : PWM输入配置
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = pwmFilter;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmGPIOConfig
**功能 : PWM所需IO口配置
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef cfg;

    cfg.GPIO_Pin = pin;
    cfg.GPIO_Mode = mode;
    cfg.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(gpio, &cfg);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmOutConfig
**功能 : 
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static pwmPortData_t *pwmOutConfig(uint8_t port, uint8_t mhz, uint16_t period, uint16_t value)
{
    pwmPortData_t *p = &pwmPorts[port];
    configTimeBase(timerHardware[port].tim, period, mhz);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, GPIO_Mode_AF_PP);
    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, value);
    // Needed only on TIM1
    if (timerHardware[port].outputEnable)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);
    TIM_Cmd(timerHardware[port].tim, ENABLE);

    p->cr1 = &timerHardware[port].tim->CR1;
    p->cnt = &timerHardware[port].tim->CNT;

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware[port].tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware[port].tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware[port].tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware[port].tim->CCR4;
            break;
    }
    p->period = period;
    return p;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmInConfig
**功能 : 
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static pwmPortData_t *pwmInConfig(uint8_t port, timerCCCallbackPtr callback, uint8_t channel)
{
    pwmPortData_t *p = &pwmPorts[port];
    const timerHardware_t *timerHardwarePtr = &(timerHardware[port]);

    p->channel = channel;

    pwmGPIOConfig(timerHardwarePtr->gpio, timerHardwarePtr->pin, GPIO_Mode_IPD);
    pwmICConfig(timerHardwarePtr->tim, timerHardwarePtr->channel, TIM_ICPolarity_Rising);

    timerConfigure(timerHardwarePtr, 0xFFFF, PWM_TIMER_MHZ);
    configureTimerCaptureCompareInterrupt(timerHardwarePtr, port, callback);

    return p;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : 
**功能 : 
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void failsafeCheck(uint8_t channel, uint16_t pulse)
{
    static uint8_t goodPulses;

    if (channel < 4 && pulse > failsafeThreshold)
        goodPulses |= (1 << channel);       // if signal is valid - mark channel as OK
    if (goodPulses == 0x0F) {               // If first four chanells have good pulses, clear FailSafe counter
        goodPulses = 0;
        if (failsafeCnt > 20)
            failsafeCnt -= 20;
        else
            failsafeCnt = 0;
    }
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : ppmCallback
**功能 : 采集PPM
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void ppmCallback(uint8_t port, uint16_t capture)
{
    (void)port;
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        chan = 0;
    } else {
        if (diff > PULSE_MIN && diff < PULSE_MAX && chan < MAX_INPUTS) {   // 750 to 2250 ms is our 'valid' channel range
            captures[chan] = diff;
            failsafeCheck(chan, diff);
        }
        chan++;
    }
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmCallback
**功能 : PWM采集
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void pwmCallback(uint8_t port, uint16_t capture)
{
    if (pwmPorts[port].state == 0) {
        pwmPorts[port].rise = capture;
        pwmPorts[port].state = 1;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Falling);
    } else {
        pwmPorts[port].fall = capture;
        // compute capture
        pwmPorts[port].capture = pwmPorts[port].fall - pwmPorts[port].rise;
        if (pwmPorts[port].capture > PULSE_MIN && pwmPorts[port].capture < PULSE_MAX) { // valid pulse width
            captures[pwmPorts[port].channel] = pwmPorts[port].capture;
            failsafeCheck(pwmPorts[port].channel, pwmPorts[port].capture);
        }
        // switch state
        pwmPorts[port].state = 0;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);
    }
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmWriteStandard
**功能 : 写入标准的PWM
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
static void pwmWriteStandard(uint8_t index, uint16_t value)
{
    *motors[index]->ccr = value;
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmInit
**功能 : PWM初始化
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
bool pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint8_t *setup;
    uint16_t period;

    // to avoid importing cfg/mcfg
    failsafeThreshold = init->failsafeThreshold;
    // pwm filtering on input
    pwmFilter = init->pwmFilter;

    syncPWM = init->syncPWM;

    if (init->usePPM)
        i++; // next index is for PPM

    setup = hardwareMaps[i];

    for (i = 0; i < MAX_PORTS; i++) {
        uint8_t port = setup[i] & 0x0F;
			  uint8_t mask = setup[i] & 0xF0;

        if (setup[i] == 0xFF) // terminator
            break;

        // hacks to allow current functionality
        if ((mask & (TYPE_IP | TYPE_IW)) && !init->enableInput)
            mask = 0;


        if (mask & TYPE_IP) {
            pwmInConfig(port, ppmCallback, 0);
            numInputs = 8;
        } else if (mask & TYPE_IW) {
            pwmInConfig(port, pwmCallback, numInputs);
            numInputs++;
        } else if (mask & TYPE_M) {
            uint32_t hz, mhz;

            if (init->motorPwmRate > 500 || init->fastPWM)
                mhz = PWM_TIMER_8_MHZ;
            else
                mhz = PWM_TIMER_MHZ;

            hz = mhz * 1000000;

            if (init->syncPWM)
                period = 8000 * mhz; // 8ms period in syncPWM mode, cycletime should be smaller than this
            else if (init->fastPWM)
                period = hz / 4000;
            else
                period = hz / init->motorPwmRate;

            motors[numMotors++] = pwmOutConfig(port, mhz, period, init->idlePulse);
        } 
    }
		
    // determine motor writer function
    pwmWritePtr = pwmWriteStandard;
		
    return FALSE;
}
/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmWriteMotor
**功能 : PWM写入电机
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
void pwmWriteMotor(uint8_t index, uint16_t value)
{
    if (index < numMotors)
        pwmWritePtr(index, value);
}

/*====================================================================================================*/
/*====================================================================================================*
**函数 : pwmRead
**功能 : 读取PWM
**输入 : 
**輸出 : None
**备注 : None
**====================================================================================================*/
/*====================================================================================================*/
uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
