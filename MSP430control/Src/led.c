/**
  ******************************************************************************
  * @file    led.c 
  * @brief   ��Դ����ʵ����ͨ��led�Ʊ�־λ�ĸı�Ӷ��ı�Ƶ�״̬
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

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/
