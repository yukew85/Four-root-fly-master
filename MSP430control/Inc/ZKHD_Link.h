#ifndef _ZKHD_LINK_H
#define _ZKHD_LINK_H

#include "stdint.h"

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

extern void ZKHD_Link_Handle(ZKHD_Link_Head_t *ZKHD_Link,uint8_t *ptr);


void ZKHD_Link_Send(uint8_t Send_ID,
                    uint8_t Receive_ID,
                    uint8_t Msg_ID,
                    uint8_t *ptr,
                    uint16_t Length,
                    uint8_t *Tx_Buff,uint8_t *);
#endif
