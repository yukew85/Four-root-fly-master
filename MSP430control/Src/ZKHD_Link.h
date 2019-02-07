#ifndef _ZKHD_LINK_H
#define _ZKHD_LINK_H

#include "stm32f1xx_hal.h"

#pragma pack (1)
typedef struct
{
    uint8_t Start;
    uint16_t Length;
    
    uint8_t Send_ID;
    uint8_t Receive_ID;
    uint8_t Message_ID;
}ZKHD_Link_Head_t;
#pragma pack ()        

typedef enum{
    Device_GCS,
    Device_FMU,
    Device_Remote,
    
    Num_Device,
}emDevice_t;

void ZKHD_Link_Handle(uint8_t  *Rx_Buff);
void ZKHD_Link_MakeFrame(uint8_t Send_ID,
                          uint8_t Receive_ID,
                          uint8_t Msg_ID,
                          uint8_t *ptr,
                          uint16_t Length,
                          uint8_t *Tx_Buff,uint8_t *);
#endif
