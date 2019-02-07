/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�speesd_estimator.c
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


*/
//�ⲿ�ļ�����
#include "speed_estimator.h"
#include "height_control.h"
#include "SPL06.h"
#include "control.h"
#include "gcs.h"
#include "myMath.h"


//�궨����
#define VELOCITY_LIMIT        (130.f)    /*�ٶ��޷� ��λcm/s*/

//Extern����


//˽�к�����
float applyDeadbandf(float value, float deadband);

//˽�б�����
float Err = 0.f;            /*λ�����*/
float wBaro = 0.35f;            /*��ѹУ��Ȩ��*/
float HeightLPF = 0.f;    /*�ںϸ߶ȣ���ͨ*/
float rangeLpf = 0.f;
float accZLpf = 0.f;            /*Z����ٶȵ�ͨ*/

/******************************************************************************
  * �������ƣ�UpdateAltSpeed
  * ����������Z���ٶȹ���
  * ��    �룺float dt����λ����ʱ��
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null  
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
    
    float accZRemovalDead = applyDeadbandf(HeightInfo.Z_Acc, 4);/*ȥ��������Z����ٶ�*/
    accZLpf += (accZRemovalDead - accZLpf) * 0.1f;        /*��ͨ*/
    
    if(isKeyFlightLand == true)
    {
        accZLpf = LIMIT(accZLpf, -1000.f, 1000.f);
    }
    else
    {
        HeightInfo.Z_Acc = accZRemovalDead;
    }

    HeightInfo.Z_Acc = accZRemovalDead;
    HeightInfo.Z_Acc -= 0.02f * Err * weight * weight * dt;    /*�������ٶ�*/
    
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
  * �������ƣ�ResetAltSpeed
  * ���������������ٶ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null  
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
  * �������ƣ�applyDeadbandf
  * ������������Ӧ������Χ
  * ��    �룺
  * value������
  * deadband��������Χ
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null  
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

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
