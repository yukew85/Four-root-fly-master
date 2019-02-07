#include "communication.h"
#include "nrf24l01.h"
#include "control.h"
#include "usbd_cdc_if.h"
#include "flash.h"

NRF_Mannager_t NRF_Mannager;
uint8_t RX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//本地地址
uint8_t TX_ADDRESS[]= {0xE1,0xE2,0xE3,0xE4,0xE5};	//接收地址RX_ADDR_P0 == RX_ADDR

#define NRF_CH_FLASH_ADDR       (uint8_t*)PAGE_ADDRESS
#define NRF_RX_ADDR_FLASH_ADDR  (uint8_t*)(PAGE_ADDRESS+2)
#define NRF_TX_ADDR_FLASH_ADDR  (uint8_t*)(PAGE_ADDRESS+2+5)

void NRF_Radio_Init(void)
{   
    uint8_t defalut[5] = {0xFF,0xFF,0xFF,0xFF,0xFF};
    
    /*此处缺少从FLASH处读取地址*/
    NRF_Mannager.Hardware_Mannager = &NRF24L01_Manager;
    
    //表示未初始化,使用默认值
    if(memcmp((uint8_t*)NRF_RX_ADDR_FLASH_ADDR,defalut,5) == 0 &&
       memcmp((uint8_t*)NRF_TX_ADDR_FLASH_ADDR,defalut,5) == 0 )
    {
        memcpy(NRF24L01_Manager.Rx_Addr,RX_ADDRESS,5);
        memcpy(NRF24L01_Manager.Tx_Addr,TX_ADDRESS,5);
        NRF24L01_Manager.RC_Frequency = NRF24L01_FREQ;
    }else
    {
        NRF24L01_Manager.RC_Frequency = *((uint8_t*)NRF_CH_FLASH_ADDR);
        memcpy(NRF24L01_Manager.Rx_Addr,NRF_RX_ADDR_FLASH_ADDR,5);
        memcpy(NRF24L01_Manager.Tx_Addr,NRF_TX_ADDR_FLASH_ADDR,5);
    }
    
    NRF24L01_Manager.NRF_Mode = NRF_Mode_TX2;
	NRF24L01_init(&NRF24L01_Manager);
}

void NRF_Radio_Send(uint8_t *ptr,uint8_t length)
{
    enQueue(&NRF_Mannager.qTx,ptr,length);
}

void NRF_IRQ()
{
    static int rx_succsess_cnt = 0;
    uint8_t state = 0;
    uint8_t rxbuff_temp[RX_PLOAD_WIDTH]; 
    
    state = NRF24L01_Read_Reg(STATUS);
    NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);
    
    if(state & MAX_TX)
    {
        NRF24L01_Write_Reg(STATUS,MAX_TX);
        
        NRF_Mannager.communication_info.Tx_Success_Count++;
        
        NRF_Mannager.Hardware_Mannager->set_rx_mode();
        NRF_Mannager.mode = Rx_Mode;
    }
   
    if(state & RX_OK)
    {
        rx_succsess_cnt++;
        NRF_Mannager.Hardware_Mannager->receive_buff(rxbuff_temp);
        enQueue(&NRF_Mannager.qRx,rxbuff_temp,sizeof(rxbuff_temp));
        
        NRF_Mannager.communication_info.Rx_Success_Count++;
    }

    //计算丢包率
    NRF_Mannager.communication_info.Tx_Packet_Loss_Rate = 
        NRF_Mannager.communication_info.Tx_Fail_Count*1.0f / (NRF_Mannager.communication_info.Tx_Fail_Count+NRF_Mannager.communication_info.Tx_Success_Count);
}

uint8_t rx_buff[32];
void NRF_Polling()
{
    uint8_t buff_temp[32];
    uint8_t length;
   
    
    uint8_t sta = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
    
    if(deQueue(&NRF_Mannager.qTx,buff_temp,&length))
    {
        switch(NRF24L01_Manager.NRF_Mode)
        {
            case NRF_Mode_TX:
            case NRF_Mode_TX2:
                TxPacket(buff_temp, length);
                break;
            case NRF_Mode_RX2:
                TxPacket_AP(buff_temp, length);
                break;
            default:
                break;
        }
    }

	if(sta & RX_OK)
	{
		uint8_t rx_len = NRF24L01_Read_Reg(RD_RX_PLOAD);
		if(rx_len<33)
		{
			NRF24L01_Read_Buf(RD_RX_PLOAD,rx_buff,32);    // read receive payload from RX_FIFO buffer
//            enQueue(&NRF_Mannager.qRx,rx_buff,rx_buff[1]);
            CDC_Transmit_FS(rx_buff, rx_buff[1]+7);
//            enQueue(&USB_Send_Queue,rx_buff,rx_buff[1]);
		}
		else 
		{
			NRF24L01_Write_Reg(FLUSH_RX,0xff);//清空缓冲区
		}
	}

    if(sta & RX_OK)
    {
        
    }
    
	if(sta & MAX_TX)
	{
		if(sta & 0x01)	//TX FIFO FULL
		{
			NRF24L01_Write_Reg(FLUSH_TX,0xff);
		}
	}
    
    NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,sta);
}

