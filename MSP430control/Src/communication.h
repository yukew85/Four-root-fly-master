#ifndef _COMMUNICATION_H
#define _COMMUNICATION_H

#include "stdint.h"
#include "queue.h"
#include "nrf24l01.h"

#define NRF24L01_FREQ   20

typedef enum
{
    Rx_Mode = 0,
    Tx_Mode,
    Unknow_Mode,
    
    Number_Mode,
}mode_t;

typedef struct
{
    uint16_t Tx_Success_Count;
    uint16_t Tx_Fail_Count;
    float Tx_Packet_Loss_Rate;
    
    uint16_t Rx_Success_Count;
}communication_info_t;

typedef struct
{
    NRF24L01_Manager_t *Hardware_Mannager;
    Queue_t qTx;
    Queue_t qRx;
    mode_t mode;
    communication_info_t communication_info;
}NRF_Mannager_t;

extern NRF_Mannager_t NRF_Mannager;

void NRF_Radio_Init(void);
void NRF_Radio_Send(uint8_t *ptr,uint8_t length);
void NRF_IRQ(void);
void NRF_Polling(void);

#endif
