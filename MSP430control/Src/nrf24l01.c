#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "queue.h"
#include "communication.h"
#include "flash.h"

#define MAX_TX  		0x10  //达到最大发送次数中断
#define TX_OK   		0x20  //TX发送完成中断
#define RX_OK   		0x40  //接收到数据中断

#undef  SUCCESS
#define SUCCESS 0
#undef  FAILED
#define FAILED  1
#define DELAY_US(x)     for(int i=0;i<72*x;i++);
#define FLASH_TX_ADDR_OFFSET      7
#define FLASH_RX_ADDR_OFFSET      2
#define FLASH_FREQ_ADDR_OFFSET    0

NRF24L01_Manager_t NRF24L01_Manager;

/**************************************************************************************/

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//24L01操作线   
#define Set_NRF24L01_CSN    (GPIOB->BSRR |= GPIO_PIN_12)      // PB12
#define Clr_NRF24L01_CSN    (GPIOB->BRR |= GPIO_PIN_12)       // PB12
#define Set_NRF24L01_CE     (GPIOB->BSRR |= GPIO_PIN_5)       // PB5
#define Clr_NRF24L01_CE     (GPIOB->BRR |= GPIO_PIN_5)        // PB5
#define READ_NRF24L01_IRQ   (GPIOA->IDR&GPIO_PIN_15)         //IRQ主机数据输入 PA15

uint8_t SPI_RW(uint8_t data);

//初始化24L01的IO口
void NRF24L01_Configuration(void)
{
	Clr_NRF24L01_CE; 	                                //使能24L01
	Set_NRF24L01_CSN;                                   //SPI片选取消
}

uint8_t SPI_RW(uint8_t txdata)
{
    HAL_StatusTypeDef status;
    
    uint8_t rxdata;
    status = HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 20);
   
    if(HAL_OK == status)
    {
        return rxdata;
    }
    
    return 1;
}

//通过SPI写寄存器
uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;	
    Clr_NRF24L01_CSN;                    //使能SPI传输
  	status =SPI_RW(regaddr); //发送寄存器号 
  	SPI_RW(data);            //写入寄存器的值
  	Set_NRF24L01_CSN;                    //禁止SPI传输	   
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;	    
	Clr_NRF24L01_CSN;                //使能SPI传输		
  	SPI_RW(regaddr);     //发送寄存器号
  	reg_val=SPI_RW(0XFF);//读取寄存器内容
  	Set_NRF24L01_CSN;                //禁止SPI传输		    
  	return(reg_val);                 //返回状态值
}	
//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //使能SPI传输
  	status=SPI_RW(regaddr);   //发送寄存器值(位置),并读取状态值   	   
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_RW(0XFF);//读出数据
  	Set_NRF24L01_CSN;                     //关闭SPI传输
  	return status;                        //返回读到的状态值
}
//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                    //使能SPI传输
  	status = SPI_RW(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_RW(*pBuf++); //写入数据	 
  	Set_NRF24L01_CSN;                                    //关闭SPI传输
  	return status;                                       //返回读到的状态值
}		

//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，
//相同时返回值:0，表示在位;否则返回1，表示不在位	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址  	
    
	for(i=0;i<5;i++)
    {
        if(buf1[i]!=0XA5)
        {
            break;					   
        }
    }
    
	if(i!=5)
    {
        return 1;                                   //NRF24L01不在位	        
    }        
	return 0;		                                //NRF24L01在位
}	 	

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    enQueue(&NRF_Mannager.qTx,txbuf,TX_PLOAD_WIDTH);
    return 0;
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值    	 
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&RX_OK)                                 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器 
		return SUCCESS; 
	}
	return FAILED;                                      //没收到任何数据
}

void TX_RX_Mode(void)
{
  
}

bool Update_NRF24l01_Freq(uint8_t freq)
{
    bool status = false;
    Clr_NRF24L01_CE;	 
    NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,freq); 
    if(NRF24L01_Read_Reg(SPI_READ_REG + RF_CH) == freq)
    {
        Flash_Write(FLASH_FREQ_ADDR_OFFSET,&freq,2);
        NRF24L01_Manager.RC_Frequency = freq;
        status = true;
    }
    Set_NRF24L01_CE;
    
    return status;
}

bool Update_NRF24l01_Rx_Addr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    Clr_NRF24L01_CE;	 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,ptr,RX_ADR_WIDTH); 
    NRF24L01_Read_Buf(RX_ADDR_P0,buf1,5);
    
    if(memcmp(buf1,ptr,5) == 0)
    {
        Flash_Write(FLASH_RX_ADDR_OFFSET,ptr,5);
        memcpy(NRF24L01_Manager.Rx_Addr,ptr,5);
        status = true;
    }
    Set_NRF24L01_CE;  
    
    return status;
}

bool Update_NRF24l01_Tx_Addr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    Clr_NRF24L01_CE;	 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,ptr,RX_ADR_WIDTH); 
    NRF24L01_Read_Buf(TX_ADDR,buf1,5);
    
    if(memcmp(buf1,ptr,5) == 0)
    {
        Flash_Write(FLASH_TX_ADDR_OFFSET,ptr,5);
        memcpy(NRF24L01_Manager.Tx_Addr,ptr,5);
        status = true;
    }
    Set_NRF24L01_CE;
    
    return status;
}

void NRF24L01_init(NRF24L01_Manager_t *ptr)
{   
    NRF24L01_Configuration();  //相关引脚配置
    while(NRF24L01_Check() == FAILED)
    ;
    Clr_NRF24L01_CE;	    

  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)ptr->Tx_Addr,TX_ADR_WIDTH);                            //写TX节点地址 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)ptr->Rx_Addr,RX_ADR_WIDTH);                         //写RX节点地址 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);                                                      //使能通道0的自动应答  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);                                                  //使能通道0的接收地址 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);                                                 //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ptr->RC_Frequency);                                         //设置RF通道为40    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);                                                   //0x27  250K   0x07 1M     

    switch(ptr->NRF_Mode)
    {
        case NRF_Mode_RX:
            NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		 
            break;
        case NRF_Mode_TX:
            NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
            break;
        case NRF_Mode_RX2:
            NRF24L01_Write_Reg(FLUSH_TX,0xff);
            NRF24L01_Write_Reg(FLUSH_RX,0xff);
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		 // IRQ收发完成中断开启,16位CRC,主接收

            SPI_RW(0x50);
            SPI_RW(0x73);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
            break;
        case NRF_Mode_TX2:
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 // IRQ收发完成中断开启,16位CRC,主发送
            NRF24L01_Write_Reg(FLUSH_TX,0xff);
            NRF24L01_Write_Reg(FLUSH_RX,0xff);

            SPI_RW(0x50);
            SPI_RW(0x73);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
            break;
        default:
            break;
    }
	Set_NRF24L01_CE;  
    
    ptr->receive_buff = NRF24L01_RxPacket;
    ptr->send_buff = NRF24L01_TxPacket;
    ptr->update_rx_Addr = Update_NRF24l01_Rx_Addr; 
    ptr->update_tx_Addr = Update_NRF24l01_Tx_Addr;
    ptr->update_frequency = Update_NRF24l01_Freq;
}

void TxPacket(uint8_t * tx_buf, uint8_t len)
{	
    Clr_NRF24L01_CE;	
	NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF24L01_Manager.Tx_Addr, TX_ADR_WIDTH);   // 装载接收端地址
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			                    // 装载数据	
	Set_NRF24L01_CE; 
}

void TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
    Clr_NRF24L01_CE;	
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 // 装载数据
	Set_NRF24L01_CE; 
}
/*********************END OF FILE******************************************************/
