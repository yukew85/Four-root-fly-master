/**
  ******************************************************************************
  * @file    beep.h 
  * @brief   ��Դ���ж����˻���F150ң������BEEP������ʹ�ò����Լ����������� 
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

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/
