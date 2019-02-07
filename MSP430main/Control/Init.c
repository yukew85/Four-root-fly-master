/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：Init.c
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
硬件初始化驱动使用时只需要调用Hadrware_Init即可


*/
//外部文件引用
#include "include.h"
#include "gcs.h"
#include "led.h"
#include "spl06.h"
#include "communication.h"
#include "led.h"
#include "delay.h"
#include "uart.h"
#include "motor.h"
#include "timer.h"

//宏定义区
#define HARDWARE_CHECK_LED    LED_STATUS_ON;LED_POWER_ON;delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;delay_ms(100);\
                              LED_STATUS_ON;LED_POWER_ON;delay_ms(100);\
                              LED_STATUS_OFF;LED_POWER_OFF;delay_ms(100);

#define HARDWARE_CHECK        g_MPUManager.Check && \
                              g_SPL06Manager.Check && \
                              g_NRFManager.Check

//Extern引用


//私有函数区
void PID_Init(void); 

//私有变量区
uint32_t SysTick_count; //系统时间计数
Queue_t USB_Send_Queue;

/******************************************************************************
  * 函数名称：Hadrware_Init
  * 函数描述：初始化所有硬件和参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void Hardware_Init(void)
{    
    System_Clock_Init();
    I2C_Init();
    Motor_Init();
    LEDInit();                      //LED闪灯初始化
    MPU6050Init();                  //g_MPUManager初始化
    SPL06_Init();                   //SPL06初始化
    NRF_Radio_Init();  
    if(HARDWARE_CHECK)              //硬件检测
    {
        g_LedManager.emLEDPower = PowerOn;
    }
    
    gcs_init();                     //地面站通信初始化
    PID_Init();                     //PID参数初始化
    USART_Init(USCI_A_UART_CLOCKSOURCE_ACLK, 115200);
    Timer_Init();
}

/******************************************************************************
  * 函数名称：PID_Init
  * 函数描述：初始化所有PID参数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null
  *    
  *
******************************************************************************/
void PID_Init(void)
{
    PIDGroup[emPID_Roll_Spd].kp    = 2.0f;
    PIDGroup[emPID_Pitch_Spd].kp   = 2.0f;
    PIDGroup[emPID_Yaw_Spd].kp     = 7.0f;
    
    PIDGroup[emPID_Roll_Spd].ki    = 0.05f;
    PIDGroup[emPID_Pitch_Spd].ki   = 0.05f;
    PIDGroup[emPID_Yaw_Spd].ki     = 0.0f;    
    
    PIDGroup[emPID_Roll_Spd].kd    = 0.15f;
    PIDGroup[emPID_Pitch_Spd].kd   = 0.15f;
    PIDGroup[emPID_Yaw_Spd].kd     = 0.2f;
    
    PIDGroup[emPID_Pitch_Pos].kp   = 3.5f;
    PIDGroup[emPID_Roll_Pos].kp    = 3.5f;
    PIDGroup[emPID_Yaw_Pos].kp     = 6.0f;

    //不建议修改以下参数
    PIDGroup[emPID_Height_Spd].kp  = 3.5f; //0.5f
    PIDGroup[emPID_Height_Spd].kd  = 0.5f;
    PIDGroup[emPID_Height_Spd].ki  = 0.00f;
    
    PIDGroup[emPID_Height_Pos].kp  = 2.0f;//0.32f  
    PIDGroup[emPID_Height_Pos].desired = 80;
    PIDGroup[emPID_Height_Pos].OutLimitHigh = 50;
    PIDGroup[emPID_Height_Pos].OutLimitLow = -50;

    //初始化UAV相关信息
    g_UAVinfo.UAV_Mode = Altitude_Hold;
}


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
