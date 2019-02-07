/**
  ******************************************************************************
  * @file    key.h 
  * @brief   ��Դ���ж����˻���F150ң����KEY����ʹ�ò����Լ����������� 
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

#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define SW1     HAL_GPIO_ReadPin(UNLOCK_GPIO_Port,UNLOCK_Pin)
#define SW2     HAL_GPIO_ReadPin(SW3_B_GPIO_Port,SW3_B_Pin)
#define SW4     HAL_GPIO_ReadPin(SW3_A_GPIO_Port,SW3_A_Pin)

#define SW3     HAL_GPIO_ReadPin(SW2_A_GPIO_Port,SW2_A_Pin)
#define SW5     HAL_GPIO_ReadPin(SW5_GPIO_Port,SW5_Pin)

#define SW1_PRES       1	  //SW1����
#define SW2_PRES	   2	  //SW2����
#define SW3_PRES	   3	  //SW3����
#define SW4_PRES       4	  //SW4����
#define SW5_PRES	   5	  //SW5����



uint8_t SW_Scan(void);
extern uint8_t is_sw2_pressed(void);


#endif

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/  
