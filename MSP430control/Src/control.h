/**
  ******************************************************************************
  * @file    control.h 
  * @brief   ��Դ���ж����˻���F150ң�������Ƴ���Դ��ʹ�ò����Լ����������� 
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
  
#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f1xx_hal.h"

#define SamplingFrequency  (10)        //����Ƶ��
#define ADCRecordTime      (10)        //��¼����

// ��̬����
typedef enum
{
    No = 0,
    Yes,
}YesNo_t;

typedef struct
{
	uint16_t Min;
	uint16_t Mid;
	uint16_t Max;
}JoystickCal_Target_t;

static const JoystickCal_Target_t JoystickCal_Target = 
{
	.Min = 1000,
	.Mid = 1500,
	.Max = 2000
};

typedef enum
{
    Yaw = 0,
    Throttle,
    Roll,
    Pitch,
	Left,
	Right,
	Batt,
    Remote_CH_Num,
}RemoteItem_t;

//У׼����
typedef struct
{
	uint16_t Max;
	uint16_t Mid;
	uint16_t Min;
}RecordADCExtremumVal_t;

typedef struct
{
	uint16_t IsSaved;             // 0xAAAA: Saved.   0x5555: Not Saved.
	
	RecordADCExtremumVal_t rol;
	RecordADCExtremumVal_t pit;
	RecordADCExtremumVal_t thr;
	RecordADCExtremumVal_t yaw;
	uint16_t crc;
}RC_Para_t;

typedef struct
{
	uint16_t Filter_Yaw;
	uint16_t Filter_Thr;
	uint16_t Filter_Rol;
	uint16_t Filter_Pit;
	uint16_t Filter_Left;
	uint16_t Filter_Right;
	uint16_t Filter_Batt;
}RC_Raw_t;

//��ɿص�ͨ��Э��
typedef struct
{
	int16_t Start;        //��ʼλ 0X5555
	int16_t ContrlBit;    //У��λ ox01
	int16_t THROTTLE;	  
	int16_t PITCH;
	int16_t ROLL;
    int16_t YAW;
    int16_t SW_TWO;       //�����������
	int16_t SW_THREE;	  //�Ҳ���������
    int16_t LEFT;		  //��ದ��
	int16_t RIGHT;		  //�Ҳದ��
	int16_t CaliFlag;	  //У׼����
	int16_t Arm_State;	  //��������
    int16_t Stop;         //ֹͣλ 0xAAAA
}send_Data_t;

typedef union
{
	send_Data_t sendData;
	uint8_t RC_Sendbuff[sizeof(send_Data_t)];
}send_Data_u;

extern uint32_t USER_FLASH_ADDR;       // �û� Flash ���� ��ʼ��ַ

extern uint16_t adcBuf[7];
extern send_Data_u RC_sendData;

extern uint8_t beep_state;
extern uint8_t led_RF_state;
extern uint8_t led_STU_state;
extern uint8_t led_POWER_state;

extern uint8_t sw1_up;
extern uint8_t sw2_up;
extern uint8_t sw3_up;

extern uint8_t Arm_up;
extern uint8_t CaliFlag_up;

void get_ADC_value(void);
static uint16_t Joystick_Raw_To_Traget(uint16_t Raw, const RecordADCExtremumVal_t* pRaw, const JoystickCal_Target_t* pTarget);
static uint16_t Get_Linear_Equation(uint16_t x, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void SW_control(void);
void LoadRCdata(void);

void KeyRCCalibration(void);                                             //ң������У��
extern uint8_t RC_Parc_CRC(RC_Para_t* pPara);                            //У��ң��������

#endif

/******************* (C) ��Ȩ���� 2017 �����пƺƵ�Ƽ����޹�˾ *******************/
