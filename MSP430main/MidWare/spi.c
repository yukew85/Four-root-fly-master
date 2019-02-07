/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�spi.c
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
����SPI_init������ʼ��SPI����
����SPI_RW������SPI�����ж�ȡ����

*/

#include "spi.h"

#define SPI_MISO_PIN    GPIO_PIN2
#define SPI_MOSI_PIN    GPIO_PIN1
#define SPI_SLK_PIN     GPIO_PIN3

#define SPI_MISO        (P4IN & SPI_MISO_PIN)
#define SPI_MOSI_H      P4OUT |= SPI_MOSI_PIN
#define SPI_MOSI_L      P4OUT &= ~SPI_MOSI_PIN
#define SPI_SLK_H       P4OUT |= SPI_SLK_PIN
#define SPI_SLK_L       P4OUT &= ~SPI_SLK_PIN

/******************************************************************************
  * �������ƣ�SPI_init
  * ������������ʼ��SPI����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void SPI_init()
{
    GPIO_setAsOutputPin(GPIO_PORT_P4, SPI_MOSI_PIN);
    GPIO_setAsInputPin(GPIO_PORT_P4, SPI_MISO_PIN);
    GPIO_setAsOutputPin(GPIO_PORT_P4, SPI_SLK_PIN);
}

/******************************************************************************
  * �������ƣ�SPI_RW
  * ����������SPI���߶�д
  * ��    �룺uint8_t txdata:��Ҫ���͵�����
  * ��    ����void
  * ��    �أ�uint8_t :���ض�ȡ��������
  * ��    ע��������һ�㲻����ʹ�ã���Ҫ���Ƭѡ�ź���ʹ��
  *
  *
******************************************************************************/
uint8_t SPI_RW(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) 
    {
        if(data & 0x80)
        {
            SPI_MOSI_H;
        }else
        {
            SPI_MOSI_L;
        }
        
        data <<= 1;                           // ��һλ��λ�����λ
        
        SPI_SLK_H;
        if (SPI_MISO) 
        {
           data |= 0x01;
        }
        SPI_SLK_L;  
    }
    
    return (data); 
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
