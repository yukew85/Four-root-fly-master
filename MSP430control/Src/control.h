/**
  ******************************************************************************
  * @file    control.h 
  * @brief   本源码中定义了基于F150遥控器控制程序源码使用参数以及函数声明。 
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
  
#ifndef __CONTROL_H
#define __CONTROL_H

#include "stm32f1xx_hal.h"

#define SamplingFrequency  (10)        //采样频率
#define ADCRecordTime      (10)        //记录次数

// 二态数据
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

//校准数据
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

//与飞控的通信协议
typedef struct
{
	int16_t Start;        //起始位 0X5555
	int16_t ContrlBit;    //校验位 ox01
	int16_t THROTTLE;	  
	int16_t PITCH;
	int16_t ROLL;
    int16_t YAW;
    int16_t SW_TWO;       //左侧两档开关
	int16_t SW_THREE;	  //右侧三档开关
    int16_t LEFT;		  //左侧拨盘
	int16_t RIGHT;		  //右侧拨盘
	int16_t CaliFlag;	  //校准按键
	int16_t Arm_State;	  //启动按键
    int16_t Stop;         //停止位 0xAAAA
}send_Data_t;

typedef union
{
	send_Data_t sendData;
	uint8_t RC_Sendbuff[sizeof(send_Data_t)];
}send_Data_u;

extern uint32_t USER_FLASH_ADDR;       // 用户 Flash 数据 起始地址

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

void KeyRCCalibration(void);                                             //遥控器自校验
extern uint8_t RC_Parc_CRC(RC_Para_t* pPara);                            //校验遥控器数据

#endif

/******************* (C) 版权所有 2017 北京中科浩电科技有限公司 *******************/
