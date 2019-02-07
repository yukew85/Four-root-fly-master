/**
  ******************************************************************************
  * @file    key.h 
  * @brief   本源码中定义了基于F150遥控器KEY按键使用参数以及函数声明。 
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

#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define SW1     HAL_GPIO_ReadPin(UNLOCK_GPIO_Port,UNLOCK_Pin)
#define SW2     HAL_GPIO_ReadPin(SW3_B_GPIO_Port,SW3_B_Pin)
#define SW4     HAL_GPIO_ReadPin(SW3_A_GPIO_Port,SW3_A_Pin)

#define SW3     HAL_GPIO_ReadPin(SW2_A_GPIO_Port,SW2_A_Pin)
#define SW5     HAL_GPIO_ReadPin(SW5_GPIO_Port,SW5_Pin)

#define SW1_PRES       1	  //SW1按下
#define SW2_PRES	   2	  //SW2按下
#define SW3_PRES	   3	  //SW3按下
#define SW4_PRES       4	  //SW4按下
#define SW5_PRES	   5	  //SW5按下



uint8_t SW_Scan(void);
extern uint8_t is_sw2_pressed(void);


#endif

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/  
