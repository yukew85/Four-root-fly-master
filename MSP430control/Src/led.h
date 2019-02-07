/**
  ******************************************************************************
  * @file    led.h 
  * @brief   ��Դ���ж����˻���F150ң������LED�Ƶ�ʹ�ò����Լ������������� 
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  �����пƺƵ�Ƽ����޹�˾   www.bj-zkhd.com  
  * @date    2017-11-5
  * @Update  
  * 
  * @historyVersion        @author        @date
  *    
  *    
  ******************************************************************************
  * @attention
  *
  * �������ṩ��Դ�����Ϊ���������߱�д��������˾�ڲ�ʹ�á�ʹ�ñ�������û��������ף�
  * ���ǲ��ܱ�֤���ṩ��Դ���׼ȷ�ԡ���ȫ�Ժ������ԣ�������ǽ�������е��κ�ֱ�ӡ��� 
  * ����ʹ����ЩԴ����Լ�����������κ���ʽ����ʧ���˺����κ�����ʹ�ñ�����ʱ����ע��
  * ���ߺͳ�����
  * <h2><center>��Ȩ����(C) 2017 �����пƺƵ�Ƽ����޹�˾</center></h2>
  ******************************************************************************
  */  
  
#ifndef  __LED_H
#define  __LED_H

#include "main.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "sys.h"

#define LED_STU_ON     HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET)
#define LED_STU_OFF    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET)
#define LED_STU_Toggle HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin)

#define LED_RF_ON      HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET)  
#define LED_RF_OFF     HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET)
#define LED_RF_Toggle  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin)

#define LED_POWER_ON      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)  
#define LED_POWER_OFF     HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

#define LED_STU_Period      50
#define LED_RF_Period       100
#define LED_RED_Period      20
#define LED_Period          1000

static uint8_t LED_STU_ms  = 0;
static uint8_t LED_RF_ms = 0;
static uint8_t LED_RED_ms   = 0;

void led_state_updata(void);

#endif

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/
