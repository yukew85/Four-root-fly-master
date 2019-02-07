/**
  ******************************************************************************
  * @file    key.c 
  * @brief   本源码中实现了F150遥控器所有按键值得获取
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
  * <h2><center>    版权所有(C) 2017 北京中科浩电科技有限公司    </center></h2>
  ******************************************************************************
  */  

#include "key.h"

//按键处理函数
//返回按键值
//0，没有任何按键按下
//1，SW1按下
//2，SW2按下
//3，SW3按下 
//4，SW4按下
//5, SW5按下
//注意此函数有响应优先级,(SW1>SW2) > (SW3>SW4>SW5)!!

extern uint8_t sw1_up      = 1;
extern uint8_t sw2_up      = 1;
extern uint8_t sw3_up      = 1;
extern uint8_t Arm_up      = 1;
extern uint8_t CaliFlag_up = 1;

extern uint8_t is_sw2_pressed(void)
{
	// GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
	
	/*
		typedef enum
		{
		  GPIO_PIN_RESET = 0U,
		  GPIO_PIN_SET
		}GPIO_PinState;
	*/
	if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(UNLOCK_GPIO_Port,UNLOCK_Pin))
		return 1;
	
	return 0;
}

uint8_t SW_Scan(void)
{	 
	static uint8_t s_swUp = 1;//按键按松开标志
		
	if( (s_swUp == 1) && (SW1 != Arm_up) )
	{
		HAL_Delay(40);//去抖动 

		s_swUp = 0;

		if(SW1 != Arm_up)
		{
			Arm_up = SW1;
			return SW1_PRES;		
		}
	}else if(SW2 != CaliFlag_up)
    {
        CaliFlag_up = SW2;
        return SW5_PRES;
    }else if (SW3 != sw1_up)
	{
		sw1_up = SW3;
		return SW3_PRES;		
	}else if (SW4 != sw2_up)
	{
	    sw2_up = SW4;
	    return SW4_PRES;		
	}else if (SW5 != sw3_up)
	{
		sw3_up = SW5;
		return SW5_PRES;
	}else if((SW1 == 1) && (SW2 == 1))
	{
		s_swUp = 1; 
	}
					
	return 0;// 无按键按下
}

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/
