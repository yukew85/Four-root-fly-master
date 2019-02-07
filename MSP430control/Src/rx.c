#include "nrf24l01.h"
#include "stdbool.h"
#include "led.h"


void RX_Mode_Config()
{
    (GPIOB->BRR = 1<<5);    
    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG, 0x0f); 
    //CE为高,进入接收模式 
  	(GPIOB->BSRR = 1<<5);
}

void TX_Mode_Config()
{
    (GPIOB->BRR = 1<<5); 
    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX发送模式,开启所有中断
  	NRF24L01_Write_Reg(SPI_WRITE_REG+NCONFIG,0x0e);    
    // CE为高,10us后启动发送
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

