#include "nrf24l01.h"
#include "stdbool.h"
#include "led.h"


void RX_Mode_Config()
{
    (GPIOB->BRR = 1<<5);    
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    //CEΪ��,�������ģʽ 
  	(GPIOB->BSRR = 1<<5);
}

void TX_Mode_Config()
{
    (GPIOB->BRR = 1<<5); 
    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX����ģʽ,���������ж�
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // CEΪ��,10us����������
	(GPIOB->BSRR = 1<<5);
}

bool rx_enable = true;
uint8_t Rx_buff[32];
 void Rx_Scan()
{
    if(!rx_enable)
    {
        rx_enable = true;
        RX_Mode_Config();
    }
    
    NRF24L01_RxPacket(Rx_buff);
    
    if(Rx_buff[0] == 0xAA)
    {
        LED_BLUE_ON;
        LED_GREEN_OFF;
    }else
    {
        LED_GREEN_ON;
        LED_BLUE_OFF;
    }
}

