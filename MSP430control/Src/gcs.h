#ifndef _GCS_H
#define _GCS_H

#include "sys.h"
#include "ZKHD_Link.h"

typedef enum
{
    MsgID_0_Set_Tx_Addr = 0,
    MsgID_1_Set_Rx_Addr,
    MsgID_2_Set_Freq,
    
    Num_Serarch,
}MSG_GCS_TO_Remote_t;

typedef enum
{
    MsgID_0_Answer_Channel_Info = 0,
    MsgID_1_Hardware_Info,
    
    Num_Answer,
}MSG_Remote_TO_GCS_t;

void update_info_to_gcs(void);
void gcs_ReceiveHandle(ZKHD_Link_Head_t *ZKHD_Link_Info,uint8_t *ptr,uint8_t length);

void update_gcs_HardwareInfo(void);
#endif
