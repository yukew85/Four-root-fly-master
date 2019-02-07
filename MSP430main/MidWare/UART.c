/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�UART.c
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
�����ڵ��������

*/
//�ⲿ�ļ�����
#include "UART.h"
#include "include.h"
#include "communication.h"

#define USART_RX_TIMEOUT_MAX    20
bool lbUsartRx = false;
int TimeoutUSART = 0;

Usart_t Usart2 = 
{
    .tx_idle = 1,
    .tx_totle = 0,
    .tx_cnt = 0,
};

/******************************************************************************
  * �������ƣ�USART_Init
  * �������������ڳ�ʼ��
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void USART_Init(uint8_t ClockSource, uint32_t Baudrate)
{
    USCI_A_UART_initParam USCI_A1_UART_initParam;
    
    float frequency = 0.0;
    float N_f = 0.0;
    
    if(USCI_A_UART_CLOCKSOURCE_ACLK == ClockSource)
    {
        frequency = UCS_getACLK();
    }
    else if(USCI_A_UART_CLOCKSOURCE_SMCLK == ClockSource)
    {
        frequency = UCS_getSMCLK();
    }
    else
    {
      return;
    }
    
    // Tx: P4.4     Rx: P4.5
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4);
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);
    
    N_f = frequency / Baudrate;
    
    USCI_A1_UART_initParam.selectClockSource = ClockSource;
    USCI_A1_UART_initParam.clockPrescalar = (uint16_t)(N_f / 16);    
    USCI_A1_UART_initParam.firstModReg = (uint16_t)(((N_f / 16) - (uint16_t)(N_f / 16)) * 16);
    USCI_A1_UART_initParam.secondModReg = 0;
    USCI_A1_UART_initParam.parity = USCI_A_UART_NO_PARITY;
    USCI_A1_UART_initParam.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    USCI_A1_UART_initParam.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    USCI_A1_UART_initParam.uartMode = USCI_A_UART_MODE;
    USCI_A1_UART_initParam.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    
    USCI_A_UART_init(USCI_A1_BASE, &USCI_A1_UART_initParam);
    
    USCI_A_UART_enable(USCI_A1_BASE);
    
    /*
        http://bbs.eeworld.com.cn/forum.php?mod=viewthread&tid=430117&highlight=msp430%2B%B4%AE%BF%DA%D6%D0%B6%CF
        �����ж�ʹ��Ҫ���ڴ��ڳ�ʼ������֮�� �����ܽ����ж�
    */
    USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT);
    USCI_A_UART_enableInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT);
}

/******************************************************************************
  * �������ƣ�PollingUSART
  * ������������ѯ���ڽ�����Ϣ
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע������ϵͳ����λ��ͨ�ţ��˴�Ϊ���
  *
  *
******************************************************************************/
void PollingUSART()
{    
    if(lbUsartRx)
    {
        TimeoutUSART++;
        
        if(TimeoutUSART > USART_RX_TIMEOUT_MAX)
        {
            //�������
            SwitchPort(Connect_Port_USB, Usart2.rx_buf);
            
            TimeoutUSART = 0;
            Usart2.rx_cnt = 0;
            lbUsartRx = false;
        }
    }
}

/******************************************************************************
  * �������ƣ�U2_UCA1_Send
  * �������������жϵķ�ʽ��������
  * ��    �룺
  * uint8_t *ptx:Ҫ���͵����ݵ�ַ
  * uint8_t len:Ҫ���͵����ݳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *
  *
******************************************************************************/
void U2_UCA1_Send(uint8_t* pTx, uint8_t len)
{
    uint8_t i;
    
    // �ȴ����ڿ���
    while(!Usart2.tx_idle)
        ;
    
    // ռ�ô���
    Usart2.tx_idle = 0;
    
    // ����Ҫ���͵�����
    for(i=0; i<len; i++)
    {
        Usart2.tx_buf[i] = pTx[i];
    }
    
    // ��������
    Usart2.tx_totle = len;
    Usart2.tx_cnt = 0;
    
    if(Usart2.tx_cnt < Usart2.tx_totle)
    {
        UCA1TXBUF = Usart2.tx_buf[Usart2.tx_cnt++];
    }
}

/******************************************************************************
  * �������ƣ�USCI_A1_ISR
  * ����������USCI_AI�жϷ�����
  * ��    �룺void
  * ��    ����void
  * ��    �أ�void
  * ��    ע���жϺ����Զ�����
  *
  *
******************************************************************************/
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
    #pragma vector = USCI_A1_VECTOR
    __interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
    void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
    #error Compiler not supported!
#endif
{
    // USART Tx Interrupt
    if(USCI_A_UART_getInterruptStatus(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG))
    {
        if(Usart2.tx_cnt < Usart2.tx_totle)
        {
            UCA1TXBUF = Usart2.tx_buf[Usart2.tx_cnt++];
        }
        else
        {
            Usart2.tx_idle = 1;
        }
        
        USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_TRANSMIT_INTERRUPT_FLAG);
    }
    
    // USART Rx Interrupt
    if(USCI_A_UART_getInterruptStatus(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT_FLAG))
    {
        lbUsartRx = true;
        
        TimeoutUSART = 0;
        Usart2.rx_buf[Usart2.rx_cnt++] = USCI_A_UART_receiveData(USCI_A1_BASE);
        USCI_A_UART_clearInterrupt(USCI_A1_BASE, USCI_A_UART_RECEIVE_INTERRUPT_FLAG);
    }
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
