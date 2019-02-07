/**
  ******************************************************************************
  * @file    beep.h 
  * @brief   本源码中定义了基于F150遥控器的BEEP蜂鸣器使用参数以及函数声明。 
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
  
#ifndef __BEEP_H
#define __BEEP_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "sys.h"

#define BEEP_OFF     HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)  
#define BEEP_ON	     HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)

#define BEEP_Period     50

static uint8_t BEEP_ms = 0;

void beep_state_updata(void);

#endif

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/
