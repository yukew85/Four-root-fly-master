#ifndef __PPM_H_
#define __PPM_H_

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define TIM_COUNTER 36000
#define TIM_PSC 2
#define PPM_GPIO_BASE       GPIOB
#define PPM_GPIO_PIN        GPIO_PIN_7 

enum 
{
    RC_CH1 = 0,
    RC_CH2,
    RC_CH3,
    RC_CH4,
    RC_CH5,
    RC_CH6,
    RC_CH7,
    RC_CH8, 
    
    RC_CH_NUM
};



enum PPM_OUTPUT_CH_STATE 
{
    CH_DOWN_STATE,
    CH_UP_STATE,
};

extern int Target;
extern int TIM2_Cnt;
extern void (*TIM_IT_CallBack)(uint8_t* p);
void ppm_output(uint8_t *ppm);
void ppm_delay(uint16_t time);
void ppm_output_ZKHD(uint8_t *ppm);
void ppm_init(void);

#endif
