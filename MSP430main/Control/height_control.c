/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�height_control.c
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�    
  * �޸�˵����
  * 
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
�߶ȿ�������


*/
//�ⲿ�ļ�����
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

//�궨����
#define IN_RANGE(value, min, max)        ( (value)>(min) && (value)<(max) )
#define THROTTLE_BASE 600

//Extern����
extern SPL06Manager_t g_SPL06Manager;


//˽�к�����
float constrain_float(float amt, float low, float high);
float ctrl_get_boosted_throttle(float throttle_in);
void UpdateAlt(void);

//˽�б�����
HeightInfo_t HeightInfo;
float dt2 = 0;
bool Acc_Enable_Flag = false;

/******************************************************************************
  * �������ƣ�HeightInit
  * �����������߶Ⱥ�����ʼ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void HeightInit()
{
    HeightInfo.Alt = 0;
    HeightInfo.Z_Speed = 0;
}

/******************************************************************************
  * �������ƣ�UpdateAltInfo
  * �������������¸߶���Ϣ
  * ��    �룺float dt:��λ����ʱ��
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void UpdateAltInfo(float dt)
{  
    //���¸߶�ֵ
    UpdateAlt();
}

/******************************************************************************
  * �������ƣ�UpdateAlt
  * ����������������ѹ�Ƽ��ĸ߶�ֵ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void UpdateAlt()
{
    UpdateSPL06Info();
    HeightInfo.Alt = g_SPL06Manager.fRelative_Alt * 100;   //cm
}

/******************************************************************************
  * �������ƣ�ControlAlt
  * �����������߶ȿ��ƺ���
  * ��    �룺
  * float dt:��λ����ʱ��
  *
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *    
******************************************************************************/
void ControlAlt(float dt)
{
    /*���²���ֵ*/
    PIDGroup[emPID_Height_Pos].measured = HeightInfo.Alt;
    PIDGroup[emPID_Height_Spd].measured = HeightInfo.Z_Speed;

    ClacCascadePID(&PIDGroup[emPID_Height_Spd],&PIDGroup[emPID_Height_Pos], dt);      //X��
    
    //���Ҫ��һ����׼����
    HeightInfo.Thr = (uint16_t)(PIDGroup[emPID_Height_Spd].out + THROTTLE_BASE);
}

/******************************************************************************
  * �������ƣ�ctrl_get_boosted_throttle
  * ����������������ǲ���ֵ
  * ��    �룺
  * float throttle_in����������ֵ
  * 
  * ��    ����void
  * ��    �أ��������ֵ
  * ��    ע��null
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
  * �������ƣ�constrain_float
  * ���������������޷�����
  * ��    �룺
  * float amt����Ҫ�޷�������
  * float low���޷���Сֵ
  * float high���޷����ֵ
  * ��    ����void
  * ��    �أ��޷����ֵ
  * ��    ע��null
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
