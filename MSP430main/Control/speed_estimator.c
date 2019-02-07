/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：speesd_estimator.c
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


*/
//外部文件引用
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "gcs.h"
#include "myMath.h"


//宏定义区
#define VELOCITY_LIMIT        (130.f)    /*速度限幅 单位cm/s*/

//Extern引用


//私有函数区
float applyDeadbandf(float value, float deadband);

//私有变量区
float Err = 0.f;            /*位移误差*/
float wBaro = 0.35f;            /*气压校正权重*/
float HeightLPF = 0.f;    /*融合高度，低通*/
float rangeLpf = 0.f;
float accZLpf = 0.f;            /*Z轴加速度低通*/

/******************************************************************************
  * 函数名称：UpdateAltSpeed
  * 函数描述：Z轴速度估算
  * 输    入：float dt：单位运行时间
  * 输    出：void
  * 返    回：void 
  * 备    注：null  
  *
  *
******************************************************************************/
void UpdateAltSpeed(float dt)
{
    float ewdt = 0;
    float weight = wBaro;
    
    HeightInfo.Alt = HeightInfo.Alt;    
    HeightLPF += (HeightInfo.Alt - HeightLPF) * 0.1f;    
    
    bool isKeyFlightLand = (g_UAVinfo.UAV_Mode == Altitude_Hold);    
    
    float accZRemovalDead = applyDeadbandf(HeightInfo.Z_Acc, 4);/*去除死区的Z轴加速度*/
    accZLpf += (accZRemovalDead - accZLpf) * 0.1f;        /*低通*/
    
    if(isKeyFlightLand == true)
    {
        accZLpf = LIMIT(accZLpf, -1000.f, 1000.f);
    }
    else
    {
        HeightInfo.Z_Acc = accZRemovalDead;
    }

    HeightInfo.Z_Acc = accZRemovalDead;
    HeightInfo.Z_Acc -= 0.02f * Err * weight * weight * dt;    /*补偿加速度*/
    
    HeightInfo.Z_Postion += HeightInfo.Z_Speed * dt + HeightInfo.Z_Acc * dt * dt / 2.0f;
    HeightInfo.Z_Speed += HeightInfo.Z_Acc * dt;
    
    Err = HeightInfo.Alt - HeightInfo.Z_Postion;        
    
    ewdt = Err * weight * dt;
    HeightInfo.Z_Postion += ewdt;
    HeightInfo.Z_Speed += weight * ewdt;
    
    if(isKeyFlightLand == true)        
    {
        HeightInfo.Z_Speed = LIMIT(HeightInfo.Z_Speed, -VELOCITY_LIMIT, VELOCITY_LIMIT);    
    }
}

/******************************************************************************
  * 函数名称：ResetAltSpeed
  * 函数描述：重置速度数据
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null  
  *
  *
******************************************************************************/
void ResetAltSpeed(void)
{    
    accZLpf = 0.f;
    HeightInfo.Alt  = 0.f;
    HeightLPF = 0.f;
    HeightInfo.Z_Speed = 0.f;
    HeightInfo.Z_Postion = HeightInfo.Alt;
}

/******************************************************************************
  * 函数名称：applyDeadbandf
  * 函数描述：适应死区范围
  * 输    入：
  * value：数据
  * deadband：死区范围
  * 输    出：void
  * 返    回：void 
  * 备    注：null  
  *
  *
******************************************************************************/
float applyDeadbandf(float value, float deadband)
{
    if (ABS(value) < deadband) {
        value = 0;
    } else if (value > 0) {
        value -= deadband;
    } else if (value < 0) {
        value += deadband;
    }
    return value;
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
