/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�remote.c
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
ң��������������÷�ʽ���£�
1.AnalyRC������NRF���������������ã����Է���ң�������͹��������ݣ�����Ҫ�������ã�
2.RCReceiveHandle�������Դ���ң�������մ�������

*/
//�ⲿ�ļ�����
#include "nrf24l01.h"
#include "control.h"
#include <math.h>
#include "myMath.h"
#include "LED.h"
#include "Remote.h"
#include "gcs.h"
#include "communication.h"
#include "pid.h"
#include <string.h>
#include "SPL06.h"

//�궨����
extern Queue_t USB_Send_Queue;


//Extern����



//˽�к�����
void RemoteUnlock(void);    
void RCReceiveHandle(void);


//˽�б�����
uint8_t RC_rxData[32];
Remote_t Remote;

 
/******************************************************************************
  * �������ƣ�AnalyRC
  * ��������������ң��������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void AnalyRC(void)
{
    uint8_t CheckSum = 0;
    OldRemote_t OldRemote;
    
    deQueue(&NRF_Mannager.qRx, RC_rxData,&CheckSum);
    memcpy((uint8_t*)&OldRemote, RC_rxData, sizeof(OldRemote_t));   
    if(RC_rxData[0]==0xAA && RC_rxData[1]==0xAF)
    {
        g_LedManager.emLEDStatus = StatusFlash;
        
        Remote.roll =  ((uint16_t)RC_rxData[4]<<8)  | RC_rxData[5];  //ͨ��1�����
        Remote.pitch = ((uint16_t)RC_rxData[6]<<8)  | RC_rxData[7];  //ͨ��2������
        Remote.thr =   ((uint16_t)RC_rxData[8]<<8)  | RC_rxData[9];   //ͨ��3������
        Remote.yaw =   ((uint16_t)RC_rxData[10]<<8) | RC_rxData[11];   //ͨ��4��ƫ��
        Remote.AUX1 =  ((uint16_t)RC_rxData[12]<<8) | RC_rxData[13];   //ͨ��5  ���Ͻǰ�������ͨ��5,Ҳ����AUX1�����߹���  
        Remote.AUX2 =  ((uint16_t)RC_rxData[14]<<8) | RC_rxData[15];   //ͨ��6  ���Ͻǰ�������ͨ��6,Ҳ����AUX2��һ����ɹ���  
        Remote.AUX3 =  ((uint16_t)RC_rxData[16]<<8) | RC_rxData[17];   //ͨ��7  û���ᵽ���ܵİ��������ڱ��� 
        Remote.AUX4 =  ((uint16_t)RC_rxData[18]<<8) | RC_rxData[19];   //ͨ��8  
        Remote.AUX5 =  ((uint16_t)RC_rxData[20]<<8) | RC_rxData[21];   //ͨ��9  
        Remote.AUX6 =  ((uint16_t)RC_rxData[22]<<8) | RC_rxData[23];   //ͨ��10 
        Remote.AUX7 =  ((uint16_t)RC_rxData[24]<<8) | RC_rxData[25];   //ͨ��11 
        
        RCReceiveHandle();
    }
    
    if((OldRemote.Start == 0xAAAA) && (OldRemote.Stop == 0x5555))
    {
        g_LedManager.emLEDStatus = StatusFlash;
        Remote.roll = OldRemote.ROLL;
        Remote.pitch = OldRemote.PITCH;
        Remote.thr = OldRemote.THROTTLE;
        Remote.yaw =  OldRemote.YAW;
        Remote.AUX1 =  OldRemote.SW_THREE;
        Remote.AUX2 =  OldRemote.SW_TWO;
        Remote.AUX6 = OldRemote.ContrlBit;
        RCReceiveHandle();
    }
}
 
/******************************************************************************
  * �������ƣ�RemoteUnlock
  * ����������ң������������
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע��null    
  *    
  *
******************************************************************************/
void RemoteUnlock(void)
{
    volatile static uint8_t status = WAITING_1;

    if(Remote.thr < 1150 &&Remote.yaw < 1200)                         //����ң�����½�����
    {
        ResetAlt();
        status = EXIT_255;
    }
    
    switch(status)
    {
        case WAITING_1://�ȴ�����
            //���������࣬�������->�������->������� ����LED�Ʋ����� ����ɽ���            
            if(Remote.thr < 1100)  //��һ��        
            {             
                 status = WAITING_2;                 
            }        
            break;
        case WAITING_2://�ڶ��� 
            if(Remote.thr > 1800)          
            {        
                static uint8_t cnt = 0;
                 cnt++;        
                if(cnt > 5) //��������豣��200ms���ϣ���ֹң�ؿ�����ʼ��δ��ɵĴ�������
                {    
                    cnt = 0;
                    status = WAITING_3;
                }
            }            
            break;
        case WAITING_3:
            if(Remote.thr < 1100)  //������        
            {             
                status = WAITING_4;                 
            }
            break;            
        case WAITING_4:    //����ǰ׼��                   
            g_FMUflg.unlock = 1;   //������־λ

            status = PROCESS_31;   //�������
            break;
        case PROCESS_31:    //�������״̬
            if(!g_FMUflg.unlock)                           //�������������ֱ�������ɿ�
            {
                status = EXIT_255;                
            }

            break;
        case EXIT_255: //��������                           
            g_FMUflg.unlock = 0;           //����
            status = WAITING_1;     //���صȴ�����
            break;
        default:
            status = EXIT_255;
            break;
    }
}

/******************************************************************************
  * �������ƣ�RCReceiveHandle
  * ����������ң�������ݽ��մ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void RCReceiveHandle()
{
    const float roll_pitch_ratio = 0.04f;  //ң�ؿ�����̬����
      
    PIDGroup[emPID_Pitch_Pos].desired = ((Remote.pitch * 1.0f)-1500)*roll_pitch_ratio;	    //��ң��ֵ��Ϊ���нǶȵ�����ֵ  
    PIDGroup[emPID_Roll_Pos].desired = ((Remote.roll * 1.0f)-1500)*roll_pitch_ratio;	
    
    if(Remote.yaw > 1800 )
    {    //����Ϊң�ؿ���ƫ���� +-�Ŵ����� 0.75�������ƫ���ǵ���ת��                            
        PIDGroup[emPID_Yaw_Pos].desired -= 2;    
    }
    else if(Remote.yaw <1200)
    {
        PIDGroup[emPID_Yaw_Pos].desired += 2;
    }
    
    switch(Remote.AUX1)
    {
        case 1000:
            g_UAVinfo.UAV_Mode = Stabilize_Mode;
            break;
        case 1500:
            g_UAVinfo.UAV_Mode = Altitude_Hold;

            if(Remote.thr > 1800)
            {
                PIDGroup[emPID_Height_Pos].desired += 1  ;
            }else if(Remote.thr < 1200)
            {
                PIDGroup[emPID_Height_Pos].desired -= 1;
            }
            break;
        case 2000:

            break;
        default:
            break;
    }
    
    switch(Remote.AUX2)
    {
        case 2000:
            
            break;
        case 1000:
            g_FMUflg.unlock = 0;
            break;
    }
    
    if(Remote.AUX6)
    {
        if(g_UAVinfo.UAV_Mode == Altitude_Hold)
        {
            g_FMUflg.unlock = 1;
        }
    }
    
    RemoteUnlock();
}

/******************************************************************************
  * �������ƣ�UpdateFMUToRemote
  * �������������·ɿ����ݵ�ң����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void 
  * ��    ע���˺����������У�ͨ��ң���������ݴ�����λ��
  *    
  *
******************************************************************************/
void UpdateFMUToRemote()
{
    uint8_t Buff[QUEUE_DATA_MAXLENGTH];
    uint8_t length;

    if(deQueue(&USB_Send_Queue, Buff,&length))
    {
        NRF_Mannager.Hardware_Mannager->send_buff(Buff);
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
