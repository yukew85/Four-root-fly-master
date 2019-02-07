/**
  ******************************************************************************
  * @file    led.c 
  * @brief   本源码中实现了通过led灯标志位的改变从而改变灯的状态
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  北京中科浩电科技有限公司   www.bj-zkhd.com  
  * @date    2017-11-5
  * @Update
  *
  * @historyVersion        @author        @date
  *    
  *    
  ******************************************************************************
  * @attention
  *
  * 本程所提供的源代吗均为本程序作者编写，仅供公司内部使用。使用本程序的用户必须明白，
  * 我们不能保证所提供的源码的准确性、安全性和完整性，因此我们将不负责承担任何直接、间 
  * 接因使用这些源码对自己和他人造成任何形式的损失或伤害。任何人在使用本代码时，请注明
  * 作者和出处。
  * <h2><center>版权所有(C) 2017 北京中科浩电科技有限公司</center></h2>
  ******************************************************************************
  */  

#include "led.h"

extern uint8_t led_RF_state = 0;
extern uint8_t led_STU_state  = 0;
extern uint8_t led_red_state   = 0;
extern uint16_t Batt_Flag; 

void led_state_updata(void)
{
	if (Batt_Flag)
	{
		LED_STU_ms++;
	
	    if (LED_STU_ms == LED_Period)
		{
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		    LED_STU_ms = 0;
		}
	}else if(led_STU_state)
	{
		LED_STU_ms++;

		if(LED_STU_ms == LED_STU_Period)
		{
			led_STU_state = 0;
			LED_STU_Toggle;
			LED_STU_ms = 0;
		}
	}else if(led_RF_state)
	{
		LED_RF_ms++;

		if(LED_RF_ms == LED_RF_Period)
		{
			led_RF_state = 0;
			LED_RF_Toggle;
			LED_RF_ms = 0;			
		}
	}else if(led_red_state)
	{
		LED_RED_ms++;
	
		if(LED_RED_ms == LED_RED_Period)
		{
			led_red_state = 0;
			LED_RED_ms = 0;
		}else
		{
			LED_POWER_ON;
		}
	}
}

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/
