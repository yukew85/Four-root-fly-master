#include "ppm.h"


/* 20ms */
#define MS20 (72000 * 20)
/* 0.5ms */
#define MS05  (36000)

static uint64_t total_value = 0;
static uint8_t ch_idx = 0;

static enum PPM_OUTPUT_CH_STATE state = CH_DOWN_STATE;

int Target;
int TIM2_Cnt;
void (*TIM_IT_CallBack)(uint8_t* p);

/*
*	- Continuously send 8 PWMs through GPIO_PB1 pin.
*	- each pwm pulse's low level width is 0.5ms
*	  the high level width shouldn't less than
*	  0.5ms, and greater than 1.5ms.
*	- the frequence of PPM output signal is 50Hz,
*	  so the period is 20ms.
*/
void ppm_output(uint8_t *ppm)
{
    uint32_t ch_val = 0;
    
    /* for CH1 ~ CH8 and the last one low level interval */
    if (CH_DOWN_STATE == state) 
    {
		/* next systick interrupt after 0.5ms */
//        systick_init(MS05);
        total_value += MS05;
        state = CH_UP_STATE;
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
//        GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    } else 
    {
        /* for channedl CH1 ~ CH8 high level interval */
        if (ch_idx < RC_CH_NUM) //是否小于通道总数
        {
            if ( ppm[ch_idx] < (MS05) )
            {
                ch_val = MS05;
            }
            else 
            {
                ch_val = (ppm[ch_idx] << 1) - MS05;
            }
            
//            systick_init(ch_val);
            total_value += ch_val;
            ch_idx++;
        } else 
        {
            /* the last long high level interval */
//            systick_init(MS20 - total_value);
            total_value = 0;
            ch_idx = 0;
        }
        state = CH_DOWN_STATE;
        
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
//        GPIO_SetBits(GPIOB, GPIO_Pin_1);
    }     
}

//单位us
void ppm_delay(uint16_t time)
{
    TIM2->ARR = time - 1;
    TIM2->CR1 |= 1 << 0;
}

int delay_total = 0;
const uint16_t delay_H = 500;
void ppm_output_ZKHD(uint8_t *ppm)
{
    uint32_t ch_val = 0;
    volatile static uint16_t p = 0;
    if (CH_DOWN_STATE == state) 
    {
        //拉高
        HAL_GPIO_WritePin(PPM_GPIO_BASE, PPM_GPIO_PIN, GPIO_PIN_SET);

        //设定延时500us
        ppm_delay(delay_H);
        delay_total += delay_H;
        state = CH_UP_STATE;
        
    }else if(CH_UP_STATE == state)
    {
        HAL_GPIO_WritePin(PPM_GPIO_BASE, PPM_GPIO_PIN, GPIO_PIN_RESET);
        
        state = CH_DOWN_STATE;
        
        if (ch_idx < RC_CH_NUM) //是否小于通道总数
        {
            p = ppm[4 + ch_idx * 2] + (ppm[5 + ch_idx * 2] << 8);
            
            if( ( ppm[4 + ch_idx * 2] + (ppm[5 + ch_idx * 2] << 8) ) < delay_H )
            {
                ch_val = delay_H;
            }else
            {
                ch_val = ( ppm[4 + ch_idx * 2] + (ppm[5 + ch_idx * 2] << 8) ) - delay_H;
            }
            
            ppm_delay(ch_val);
            
            delay_total += ch_val;
            ch_idx++;
        }else
        {
            ppm_delay(20000 - delay_total);
            ch_idx = 0;
            delay_total = 0;
        }
    }
}

void ppm_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    
    GPIO_InitStruct.Pin = PPM_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PPM_GPIO_BASE, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    
    RCC->APB1ENR |= 1 << 0;
    TIM_IT_CallBack = ppm_output_ZKHD;
    
    //72分频  1M
    TIM2->PSC = 72 - 1;
    TIM2->DIER = 1<<0;//|1<<6|1<<7;
    //TIM2->DIER |= 1<<1;  
    
    state = CH_DOWN_STATE;
    HAL_GPIO_WritePin(PPM_GPIO_BASE, PPM_GPIO_PIN, GPIO_PIN_RESET);
}
