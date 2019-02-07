#include "gcs.h"
#include "usbd_cdc_if.h"
#include "queue.h"
#include "communication.h"
#include "RemoteConfig.h"

extern uint16_t Runtime;
Queue_t USB_Send_Queue;
int usb_send;
void update_info_to_gcs()
{
    uint8_t tx_buff[32];
    uint8_t tx_length;
    
    if(deQueue(&USB_Send_Queue,tx_buff,&tx_length))
    {
        CDC_Transmit_FS(tx_buff, tx_length);
    }
}

void gcs_ReceiveHandle(ZKHD_Link_Head_t *ZKHD_Link_Info,uint8_t *ptr,uint8_t length)
{
    switch(ZKHD_Link_Info->Message_ID)
    {
        case MsgID_0_Set_Tx_Addr:
            NRF_Mannager.Hardware_Mannager->update_tx_Addr(ptr);
            break;
        case MsgID_1_Set_Rx_Addr:
            NRF_Mannager.Hardware_Mannager->update_rx_Addr(ptr);
            break;
        case MsgID_2_Set_Freq:
            NRF_Mannager.Hardware_Mannager->update_frequency(ptr[0]);
            break;
        default:
            break;
    }
}

void update_gcs_HardwareInfo(void)
{
    uint8_t DataBuff[14];
    uint8_t SendBuff[21];
    uint8_t SendLength;
    
    DataBuff[0] = FIRMWARE_INFO;//版本信息
    memcpy(DataBuff+1,NRF_Mannager.Hardware_Mannager->Tx_Addr,5);//发送地址
    memcpy(DataBuff+1+5,NRF_Mannager.Hardware_Mannager->Rx_Addr,5);//发送地址
    DataBuff[11] = Runtime&0xFF;
    DataBuff[12] = Runtime>>8;
    DataBuff[13] = NRF_Mannager.Hardware_Mannager->RC_Frequency;
    ZKHD_Link_MakeFrame( Device_Remote,
                        Device_GCS,
                        MsgID_1_Hardware_Info,
                        DataBuff,
                        14,
                        SendBuff,
                        &SendLength
                       );
    
    enQueue(&USB_Send_Queue,SendBuff,SendLength);
}
