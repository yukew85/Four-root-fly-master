#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "nrf24l01.h"
#include "SPI.h"
#include <string.h>
#include "queue.h"
#include "communication.h"
#include "flash.h"

#define MAX_TX  		0x10  //�ﵽ����ʹ����ж�
#define TX_OK   		0x20  //TX��������ж�
#define RX_OK   		0x40  //���յ������ж�

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
//24L01������   
#define Set_NRF24L01_CSN    (GPIOB->BSRR |= GPIO_PIN_12)      // PB12
#define Clr_NRF24L01_CSN    (GPIOB->BRR |= GPIO_PIN_12)       // PB12
#define Set_NRF24L01_CE     (GPIOB->BSRR |= GPIO_PIN_5)       // PB5
#define Clr_NRF24L01_CE     (GPIOB->BRR |= GPIO_PIN_5)        // PB5
#define READ_NRF24L01_IRQ   (GPIOA->IDR&GPIO_PIN_15)         //IRQ������������ PA15

uint8_t SPI_RW(uint8_t data);

//��ʼ��24L01��IO��
void NRF24L01_Configuration(void)
{
	Clr_NRF24L01_CE; 	                                //ʹ��24L01
	Set_NRF24L01_CSN;                                   //SPIƬѡȡ��
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

//ͨ��SPIд�Ĵ���
uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;	
    Clr_NRF24L01_CSN;                    //ʹ��SPI����
  	status =SPI_RW(regaddr); //���ͼĴ����� 
  	SPI_RW(data);            //д��Ĵ�����ֵ
  	Set_NRF24L01_CSN;                    //��ֹSPI����	   
  	return(status);       		         //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;	    
	Clr_NRF24L01_CSN;                //ʹ��SPI����		
  	SPI_RW(regaddr);     //���ͼĴ�����
  	reg_val=SPI_RW(0XFF);//��ȡ�Ĵ�������
  	Set_NRF24L01_CSN;                //��ֹSPI����		    
  	return(reg_val);                 //����״ֵ̬
}	
//��ָ��λ�ö���ָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ 
uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,u8_ctr;	       
  	Clr_NRF24L01_CSN;                     //ʹ��SPI����
  	status=SPI_RW(regaddr);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
	for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_RW(0XFF);//��������
  	Set_NRF24L01_CSN;                     //�ر�SPI����
  	return status;                        //���ض�����״ֵ̬
}
//��ָ��λ��дָ�����ȵ�����
//*pBuf:����ָ��
//����ֵ,�˴ζ�����״̬�Ĵ���ֵ
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,u8_ctr;	    
	Clr_NRF24L01_CSN;                                    //ʹ��SPI����
  	status = SPI_RW(regaddr);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  	for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_RW(*pBuf++); //д������	 
  	Set_NRF24L01_CSN;                                    //�ر�SPI����
  	return status;                                       //���ض�����״ֵ̬
}		

//�ϵ���NRF24L01�Ƿ���λ
//д5������Ȼ���ٶ��������бȽϣ�
//��ͬʱ����ֵ:0����ʾ��λ;���򷵻�1����ʾ����λ	
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
	uint8_t buf1[5];
	uint8_t i;   	 
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //����д��ĵ�ַ  	
    
	for(i=0;i<5;i++)
    {
        if(buf1[i]!=0XA5)
        {
            break;					   
        }
    }
    
	if(i!=5)
    {
        return 1;                                   //NRF24L01����λ	        
    }        
	return 0;		                                //NRF24L01��λ
}	 	

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:�������״��
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    enQueue(&NRF_Mannager.qTx,txbuf,TX_PLOAD_WIDTH);
    return 0;
}

//����NRF24L01����һ������
//txbuf:�����������׵�ַ
//����ֵ:0��������ɣ��������������
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;		    							      
	state=NRF24L01_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ    	 
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
	if(state&RX_OK)                                 //���յ�����
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ��� 
		return SUCCESS; 
	}
	return FAILED;                                      //û�յ��κ�����
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
    NRF24L01_Configuration();  //�����������
    while(NRF24L01_Check() == FAILED)
    ;
    Clr_NRF24L01_CE;	    

  	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)ptr->Tx_Addr,TX_ADR_WIDTH);                            //дTX�ڵ��ַ 
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)ptr->Rx_Addr,RX_ADR_WIDTH);                         //дRX�ڵ��ַ 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);                                                      //ʹ��ͨ��0���Զ�Ӧ��  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);                                                  //ʹ��ͨ��0�Ľ��յ�ַ 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);                                                 //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ptr->RC_Frequency);                                         //����RFͨ��Ϊ40    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);                                                   //0x27  250K   0x07 1M     

    switch(ptr->NRF_Mode)
    {
        case NRF_Mode_RX:
            NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		 
            break;
        case NRF_Mode_TX:
            NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//ѡ��ͨ��0����Ч���ݿ�� 
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
            break;
        case NRF_Mode_RX2:
            NRF24L01_Write_Reg(FLUSH_TX,0xff);
            NRF24L01_Write_Reg(FLUSH_RX,0xff);
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		 // IRQ�շ�����жϿ���,16λCRC,������

            SPI_RW(0x50);
            SPI_RW(0x73);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
            NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
            break;
        case NRF_Mode_TX2:
            NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 // IRQ�շ�����жϿ���,16λCRC,������
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
	NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)NRF24L01_Manager.Tx_Addr, TX_ADR_WIDTH);   // װ�ؽ��ն˵�ַ
	NRF24L01_Write_Buf(WR_TX_PLOAD, tx_buf, len); 			                    // װ������	
	Set_NRF24L01_CE; 
}

void TxPacket_AP(uint8_t * tx_buf, uint8_t len)
{	
    Clr_NRF24L01_CE;	
	NRF24L01_Write_Buf(0xa8, tx_buf, len); 			 // װ������
	Set_NRF24L01_CE; 
}
/*********************END OF FILE******************************************************/
