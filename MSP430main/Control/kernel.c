/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：kernel.c
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
核心运行函数，本文件用以轮训主要功能函数

*/
//外部文件引用
#include "include.h"
#include "communication.h"
#include "gcs.h"
#include "speed_estimator.h"
#include "led.h"
#include "UART.h"

//宏定义区
#define RATE_166HZ (166)
#define RATE_333HZ (333)

//Extern定义区

//私有函数区
void Update(void);
void UpdateUSBQueue(void);

//私有变量区
bool Thread_3MS;
bool Thread_6MS;
bool Thread_10MS;
bool Thread_20MS;

/******************************************************************************
  * 函数名称：TIMER0_B0_ISR
  * 函数描述：TIMER0 B0 中断服务函数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：1ms运行一次
  *    
  *
******************************************************************************/
// Timer B0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMER0_B0_ISR(void)
#elif defined(__GNUC__)
void __attribute__((interrupt(TIMER0_B0_VECTOR))) TIMER0_B0_ISR(void)
#else
#error Compiler not supported!
#endif
{
    static uint32_t Cnt = 0;

    Cnt++;
    if (Cnt % 3 == 0) //3ms
    {
        Thread_3MS = true;
    }

    if (Cnt % 6 == 0)
    {
        Thread_6MS = true;
    }

    if (Cnt % 20 == 0)
    {
        Thread_20MS = true;
    }
    
    if (Cnt % 10 == 0)
    {
        Thread_10MS = true;
    }
    
    PollingUSART();
    PollingLED();
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);
}

/******************************************************************************
  * 函数名称：PollingKernel
  * 函数描述：主运行函数
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：此函数任务由B0中断服务函数开启
  *    
  *
******************************************************************************/
void PollingKernel()
{

    if (Thread_3MS)
    {
        Thread_3MS = false;
        
        GetMPU6050Data();        
        FlightPidControl(0.003f); 
        switch (g_UAVinfo.UAV_Mode)
        {
        case Altitude_Hold:
            ControlAlt(0.003f);
            break;
        default:
            break;
        }

        MotorControl();
    }

    if (Thread_6MS)
    {
        Thread_6MS = false;

        GetAngle(&g_MPUManager, &g_Attitude, 0.00626f);
        UpdateAltSpeed(0.00626f);
    }
    
    if (Thread_10MS)
    {
        Thread_10MS = false;
        
        PollingGCS();
    }

    if (Thread_20MS)
    {
        Thread_20MS = false;
      
        PollingNRF(); 
        UpdateAltInfo(0.02f);
    }
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/

