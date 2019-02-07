/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.h
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
#ifndef _UART_H
#define _UART_H
//�ⲿ�ļ�����
#include "include.h"

typedef struct
{
    uint16_t tx_idle;     // �Ƿ����(0: ָ�������ݰ���û�з������)
    uint16_t tx_totle;    // ��������
    uint16_t tx_cnt;      // �ѷ�����
    uint16_t rx_cnt;
    
    uint8_t tx_buf[200];  // ���ͻ���    
    uint8_t rx_buf[200];  // ���ջ���
}Usart_t;

extern Usart_t Usart1;
extern Usart_t Usart2;

void USART_Init(uint8_t ClockSource, uint32_t Baudrate);
void U2_UCA1_Send(uint8_t* pTx, uint8_t len);

void PollingUSART();

#endif

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/

