/**
  ******************************************************************************
  * @file    key.c 
  * @brief   ��Դ����ʵ����F150ң�������а���ֵ�û�ȡ
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
  * <h2><center>    ��Ȩ����(C) 2017 �����пƺƵ�Ƽ����޹�˾    </center></h2>
  ******************************************************************************
  */  

#include "key.h"

//����������
//���ذ���ֵ
//0��û���κΰ�������
//1��SW1����
//2��SW2����
//3��SW3���� 
//4��SW4����
//5, SW5����
//ע��˺�������Ӧ���ȼ�,(SW1>SW2) > (SW3>SW4>SW5)!!

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
	static uint8_t s_swUp = 1;//�������ɿ���־
		
	if( (s_swUp == 1) && (SW1 != Arm_up) )
	{
		HAL_Delay(40);//ȥ���� 

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
					
	return 0;// �ް�������
}

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/
