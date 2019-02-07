/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�mpu6050.c
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
MPU6050������ʹ�÷������£�
1.����MPU6050Init�������鿴��ǰMPU6050�Ƿ��ʼ���ɹ���
2.�̶����ڵ���GetMPU6050Data���Ի�ȡ���������ݣ�

PS�����������ݴ����g_MPUManager��


*/
//�ⲿ�ļ�����
#include "include.h"
#include "mpu6050.h"
#include "filter.h"
#include <string.h>
#include "LED.h"
#include "myMath.h"
#include "kalman.h"
#include "i2c.h"


//�궨����
#define SMPLRT_DIV          0x19    //�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define CONFIGL             0x1A    //��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define GYRO_CONFIG         0x1B    //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define ACCEL_CONFIG        0x1C    //���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define ACCEL_ADDRESS       0x3B
#define ACCEL_XOUT_H        0x3B
#define ACCEL_XOUT_L        0x3C
#define ACCEL_YOUT_H        0x3D
#define ACCEL_YOUT_L        0x3E
#define ACCEL_ZOUT_H        0x3F
#define ACCEL_ZOUT_L        0x40
#define TEMP_OUT_H          0x41
#define TEMP_OUT_L          0x42
#define GYRO_XOUT_H         0x43
#define GYRO_ADDRESS        0x43
#define GYRO_XOUT_L         0x44    
#define GYRO_YOUT_H         0x45
#define GYRO_YOUT_L         0x46
#define GYRO_ZOUT_H         0x47
#define GYRO_ZOUT_L         0x48
#define PWR_MGMT_1          0x6B    //��Դ��������ֵ��0x00(��������)
#define WHO_AM_I            0x75    //IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define MPU6050_PRODUCT_ID  0x68
#define MPU6052C_PRODUCT_ID 0x72
#define MPU6050_ADDRESS     0xD0    //0x68


//Extern����



//˽�к�����
void Acc_Read(uint8_t *ptr);
void Gyro_Read(uint8_t *ptr);


//˽�б�����
MPU6050Manager_t g_MPUManager;   //g_MPUManagerԭʼ����
int16_t *pMpu = (int16_t *)&g_MPUManager;
/******************************************************************************
  * �������ƣ�MPU6050Init
  * ����������g_MPUManager�ĳ�ʼ��
  * ��    �룺void
  * ��    ����g_MPUManager��ʼ�����   
              0:��ʼ���ɹ�
              1:��ʼ��ʧ��
  * ��    �أ� 
  * ��    ע��    
  *    
  *
******************************************************************************/
bool MPU6050Init(void) //��ʼ��
{   
    uint8_t check = 0;
    
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,    0x80);   //��λ
    delay_ms(30);
    I2C_Write_Byte(MPU6050_ADDRESS, SMPLRT_DIV,   0x02);   //�����ǲ����ʣ�0x00(333Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, PWR_MGMT_1,   0x03);   //�����豸ʱ��Դ��������Z��
    I2C_Write_Byte(MPU6050_ADDRESS, CONFIGL,      0x03);   //��ͨ�˲�Ƶ�ʣ�0x03(42Hz)
    I2C_Write_Byte(MPU6050_ADDRESS, GYRO_CONFIG,  0x18);   //+-2000deg/s
    I2C_Write_Byte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //+-4G

    check = I2C_Read_Byte(MPU6050_ADDRESS, 0x75);  //�ж�g_MPUManager��ַ

    if(check != MPU6050_PRODUCT_ID) //�����ַ����ȷ
    {
        g_MPUManager.Check = false;
        return false;
    }
    else
    {
        GetMPU6050Offset(); //����У׼����
        g_MPUManager.Check = true;
        return true;
    }
}

/******************************************************************************
  * �������ƣ�GetMPU6050Data
  * ������������ȡ�����Ǻͼ��ٶȼƵ����ݲ����˲�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void GetMPU6050Data(void) 
{
    uint8_t buffer[12];
    const float factor = 0.15f;  //�˲�����  
    static float tBuff[3] = {0};
    static EKF_Filter_t s_EKF[3] = {{0.02, 0, 0, 0, 0.001, 0.543},
                                    {0.02, 0, 0, 0, 0.001, 0.543},
                                    {0.02, 0, 0, 0, 0.001, 0.543}};    

    Acc_Read(buffer);
    Gyro_Read(buffer);

    for(int i = 0; i < 6; i++)
    {
        pMpu[i] = (((int16_t)buffer[i << 1] << 8) | buffer[(i << 1) + 1])
                    - g_MPUManager.Offset[i];        

        //�˴��Լ��ٶ���һά�������˲�
        if(i < 3)
        {
            KalmanFilter(&s_EKF[i],(float)pMpu[i]);  //һά������
            pMpu[i] = (int16_t)s_EKF[i].out;
        }

        //�˴��Խ��ٶ���һ���ͨ�˲�
        if(i > 2)
        {    
            uint8_t k = i - 3;
            pMpu[i] = (int16_t)(tBuff[k] * (1 - factor) + pMpu[i] * factor);         
            tBuff[k] = tBuff[k] * (1 - factor) + pMpu[i] * factor;   
        }
    }
}

/******************************************************************************
  * �������ƣ�GetMPU6050Offset
  * ������������ȡg_MPUManager��̬�´�����ƫ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void GetMPU6050Offset(void) //У׼
{
    int32_t buffer[6] = {0};
    int16_t i = 0;  
    uint8_t k = 30;
    const int8_t MAX_GYRO_QUIET = 5;
    const int8_t MIN_GYRO_QUIET = -5;    
    
    int16_t LastGyro[3] = {0};  /*wait for calm down*/
    int16_t ErrorGyro[3] = {0};        /*set offset initial to zero*/
    
    memset(g_MPUManager.Offset, 0, 12);
    g_MPUManager.Offset[2] = 8192;   //�����ֲ������趨���ٶȱ궨ֵ 

    while(k--)  //�жϷɿ��Ƿ��ھ�ֹ״̬
    {
        do
        {
            delay_ms(10);
            GetMPU6050Data();
            
            for(i = 0; i < 3; i++)
            {
                ErrorGyro[i] = pMpu[i + 3] - LastGyro[i];
                LastGyro[i] = pMpu[i + 3];    
            }
        }while ((ErrorGyro[0] > MAX_GYRO_QUIET) 
             || (ErrorGyro[0] < MIN_GYRO_QUIET)
             || (ErrorGyro[1] > MAX_GYRO_QUIET) 
             || (ErrorGyro[1] < MIN_GYRO_QUIET)
             || (ErrorGyro[2] > MAX_GYRO_QUIET)
             || (ErrorGyro[2] < MIN_GYRO_QUIET));
    }

    for(i = 0; i < 356; i++)  //ȡ��100����356���ƽ��ֵ��ΪУ׼ֵ
    {        
        GetMPU6050Data();
        
        if(100 <= i)
        {
            for(int k = 0; k < 6; k++)
            {
                buffer[k] += pMpu[k];
            }
        }
    }

    for(i = 0; i < 6; i++)  //����У׼ֵ
    {
        g_MPUManager.Offset[i] = buffer[i] >>8;
    }
}

/******************************************************************************
  * �������ƣ�Acc_Read
  * ������������ȡ���ٶ�ֵ
  * ��    �룺
  * uint8_t *ptr��д���ַ
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void Acc_Read(uint8_t *ptr)
{
    for(int i=0;i<6;i++)
    {
        ptr[i] = I2C_Read_Byte(0xd0,0x3B+i);
    }
}

/******************************************************************************
  * �������ƣ�Gyro_Read
  * ������������ȡ������ֵ
  * ��    �룺
  * uint8_t *ptr��д���ַ
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void Gyro_Read(uint8_t *ptr)
{
    for(int i=0;i<6;i++)
    {
        ptr[i+6] = I2C_Read_Byte(0xd0,0x43+i);
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
