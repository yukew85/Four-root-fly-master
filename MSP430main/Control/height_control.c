/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：height_control.c
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
高度控制驱动


*/
//外部文件引用
#include "height_control.h"
#include "control.h"
#include "SPL06.h"
#include "imu.h"
#include "mpu6050.h"
#include "fmuConfig.h"
#include "myMath.h"
#include "math.h"
#include "Remote.h"
#include "pid.h"

//宏定义区
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 600

//Extern引用
extern SPL06Manager_t g_SPL06Manager;


//私有函数区
float constrain_float(float amt, float low, float high);
float ctrl_get_boosted_throttle(float throttle_in);
void UpdateAlt(void);

//私有变量区
HeightInfo_t HeightInfo;
float dt2 = 0;
bool Acc_Enable_Flag = false;

/******************************************************************************
  * 函数名称：HeightInit
  * 函数描述：高度函数初始化
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void HeightInit()
{
    HeightInfo.Alt = 0;
    HeightInfo.Z_Speed = 0;
}

/******************************************************************************
  * 函数名称：UpdateAltInfo
  * 函数描述：更新高度信息
  * 输    入：float dt:单位运行时间
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void UpdateAltInfo(float dt)
{  
    //更新高度值
    UpdateAlt();
}

/******************************************************************************
  * 函数名称：UpdateAlt
  * 函数描述：更新气压计检测的高度值
  * 输    入：void
  * 输    出：void
  * 返    回：void
  * 备    注：null    
  *    
  *
******************************************************************************/
void UpdateAlt()
{
    UpdateSPL06Info();
    HeightInfo.Alt = g_SPL06Manager.fRelative_Alt * 100;   //cm
}

/******************************************************************************
  * 函数名称：ControlAlt
  * 函数描述：高度控制函数
  * 输    入：
  * float dt:单位运行时间
  *
  * 输    出：void
  * 返    回：void
  * 备    注：null
  *    
  *    
******************************************************************************/
void ControlAlt(float dt)
{
    /*更新测量值*/
    PIDGroup[emPID_Height_Pos].measured = HeightInfo.Alt;
    PIDGroup[emPID_Height_Spd].measured = HeightInfo.Z_Speed;

    ClacCascadePID(&PIDGroup[emPID_Height_Spd],&PIDGroup[emPID_Height_Pos], dt);      //X轴
    
    //输出要给一个基准油门
    HeightInfo.Thr = (uint16_t)(PIDGroup[emPID_Height_Spd].out + THROTTLE_BASE);
}

/******************************************************************************
  * 函数名称：ctrl_get_boosted_throttle
  * 函数描述：油门倾角补偿值
  * 输    入：
  * float throttle_in：油门输入值
  * 
  * 输    出：void
  * 返    回：油门输出值
  * 备    注：null
  *    
  *
******************************************************************************/
float ctrl_get_boosted_throttle(float throttle_in)
{
    float cos_tilt = cosine(g_Attitude.roll)*sine(g_Attitude.pitch);
    float inverted_factor = constrain_float(2.0f * cos_tilt, 0.0f, 1.0f);
    float boost_factor = 1.0f / constrain_float(cos_tilt, 0.5f, 1.0f);

    float throttle_out = throttle_in * inverted_factor * boost_factor;
    
    return throttle_out;
}

/******************************************************************************
  * 函数名称：constrain_float
  * 函数描述：数据限幅函数
  * 输    入：
  * float amt：需要限幅的数据
  * float low：限幅最小值
  * float high：限幅最大值
  * 输    出：void
  * 返    回：限幅后的值
  * 备    注：null
  *    
  *
******************************************************************************/
float constrain_float(float amt, float low, float high)
{
    if (isnan(amt)) 
   {
        return (low + high) * 0.5f;
    }
    return ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)));
}

/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
