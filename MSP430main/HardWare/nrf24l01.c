/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�nrf24l01.c
  * ժ    Ҫ��
  *
  * ��ǰ�汾��V1.0
  * ��    �ߣ������пƺƵ�Ƽ����޹�˾�з��� 
  * ������ڣ�
  * �޸�˵����
  *
  *
  * ��ʷ�汾��
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
NRF24L01������ʹ�÷������£�
1.ʵ����һ��NRF24L01_Manager_t�ṹ�壻
2.����NRF24L01Init����ʼ���ṹ�壻
3.���ݳ�ʼ����ģʽ��ѡ�������ݵķ��ͺ���

*/
//�ⲿ�ļ�����
#include "nrf24l01.h"
#include <string.h>
#include "nrf24l01.h"
#include <string.h>
#include "fmuConfig.h"
#include "communication.h"
#include "flash.h"
#include "spi.h"

//�궨����
#define MAX_TX                      0x10  //�ﵽ����ʹ����ж�
#define TX_OK                       0x20  //TX��������ж�
#define RX_OK                       0x40  //���յ������ж�

#define SUCCESS                     0
#define FAILED                      1
#define FLASH_TX_ADDR_OFFSET        7
#define FLASH_RX_ADDR_OFFSET        2
#define FLASH_FREQ_ADDR_OFFSET      0

#define NRF2401_CE_PORT         P3OUT
#define NRF2401_CE_PIN          GPIO_PIN5

#define NRF2401_CSN_PORT        P4OUT
#define NRF2401_CSN_PIN         GPIO_PIN0

#define NRF2401_IRQ_PORT        P3IN
#define NRF2401_IRQ_PIN         GPIO_PIN6

#define SET_NRF24L01_CSN        NRF2401_CSN_PORT |= NRF2401_CSN_PIN
#define CLR_NRF24L01_CSN        NRF2401_CSN_PORT &= ~NRF2401_CSN_PIN
#define SET_NRF24L01_CE         NRF2401_CE_PORT  |= NRF2401_CE_PIN
#define CLR_NRF24L01_CE         NRF2401_CE_PORT  &= ~NRF2401_CE_PIN

//Extern����


//˽�к�����
uint8_t SPI_RW(uint8_t data);                                         //д������            
uint8_t WriteBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);    //д������
uint8_t ReadBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);     //��������          
uint8_t ReadReg(uint8_t regaddr);                                     //���Ĵ���
uint8_t WriteReg(uint8_t regaddr, uint8_t data);                      //д�Ĵ���
uint8_t Check(void);                                                  //���NRF24L01�Ƿ���λ
uint8_t TxPacket(uint8_t *txbuf);                                     //����һ����������
uint8_t RxPacket(uint8_t *rxbuf);                                     //����һ����������

//˽�б�����
NRF24L01_Manager_t g_NRFManager;




/******************************************************************************
  * �������ƣ�WriteReg
  * ����������SPI����дNRF24L01�Ĵ���
  * ��    �룺uint8_t regaddr:�Ĵ�����ַ
              uint8_t data:��Ҫд�������
  * ��    ����void
  * ��    �أ��Ƿ�д��ɹ���0��д��ɹ�
                           1��д��ʧ��
  * ��    ע��null
  *    
  *
******************************************************************************/
uint8_t WriteReg(uint8_t regaddr, uint8_t data)
{
    uint8_t status = 0;   
    
    CLR_NRF24L01_CSN;                    //ʹ��SPI����
    status = SPI_RW(regaddr); //���ͼĴ����� 
    SPI_RW(data);            //д��Ĵ�����ֵ
    SET_NRF24L01_CSN;                    //��ֹSPI���� 
    
    return(status);                        //����״ֵ̬
}
//��ȡSPI�Ĵ���ֵ ��regaddr:Ҫ���ļĴ���
/******************************************************************************
  * �������ƣ�ReadReg
  * ����������SPI���߶�NRF24L01�Ĵ���
  * ��    �룺uint8_t regaddr:�Ĵ�����ַ
  * ��    ����void
  * ��    �أ��Ƿ��ȡ�ɹ���0����ȡ�ɹ�
                           1����ȡʧ��
  * ��    ע��null  
  *    
  *
******************************************************************************/
uint8_t ReadReg(uint8_t regaddr)
{
    uint8_t reg_val = 0; 
    
    CLR_NRF24L01_CSN;                //ʹ��SPI����        
    SPI_RW(regaddr);                 //���ͼĴ�����
    reg_val = SPI_RW(0XFF);          //��ȡ�Ĵ�������
    SET_NRF24L01_CSN;                //��ֹSPI����
    
    return(reg_val);                 //����״ֵ̬
}    

/******************************************************************************
  * �������ƣ�ReadBuf
  * ����������������ȡNRF24L01�Ĵ�������
  * ��    �룺uint8_t regaddr:�Ĵ�����ַ
              uint8_t *pBuf:��ȡ���ݵĴ�ŵ�ַ
              uint8_t datalen:��ȡ���ݵĳ���
  * ��    ����void
  * ��    �أ��Ƿ��ȡ�ɹ���0����ȡ�ɹ�
                           1����ȡʧ��
  * ��    ע��    
  *    
  *
******************************************************************************/
uint8_t ReadBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status = 0;
    
    CLR_NRF24L01_CSN;                       //ʹ��SPI����
    status = SPI_RW(regaddr);                 //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬          
    
    for(uint8_t ctr = 0; ctr < datalen; ctr++)
    {
        pBuf[ctr] = SPI_RW(0XFF);           //��������
    }
    
    SET_NRF24L01_CSN;                       //�ر�SPI����
    
    return status;                          //���ض�����״ֵ̬
}

/******************************************************************************
  * �������ƣ�WriteBuf
  * ��������������д��NRF24L01�Ĵ�������
  * ��    �룺uint8_t regaddr:Ҫд�����ݵļĴ�����ַ
              uint8_t *pBuf:Ҫд������ݵĵ�ַ
              uint8_t datalen:Ҫд������ݵĳ���
  * ��    ����void
  * ��    �أ��Ƿ��ȡ�ɹ���0����ȡ�ɹ�
                           1����ȡʧ��
  * ��    ע��    
  *    
  *
******************************************************************************/
uint8_t WriteBuf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
    uint8_t status = 0;
    
    CLR_NRF24L01_CSN;                                    //ʹ��SPI����
    status = SPI_RW(regaddr);                            //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
    
    for(int i = 0; i < datalen; i++)
    {
        SPI_RW(*pBuf++);                                 //д������     
    }
    SET_NRF24L01_CSN;                                    //�ر�SPI����

    return status;                                       //���ض�����״ֵ̬
}        

/******************************************************************************
  * �������ƣ�Check
  * �������������NRF24L01Ӳ���Ƿ����ӳɹ�
  * ��    �룺void
  * ��    ����void
  * ��    �أ��Ƿ��ȡ�ɹ���0�����ӳɹ�
                           1������ʧ��
  * ��    ע��null
  *    
  *
******************************************************************************/
uint8_t Check(void)
{
    uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
    uint8_t buf1[5] = {0};
    uint8_t i = 0;
    
    WriteBuf(SPI_WRITE_REG + TX_ADDR, buf, 5); //д��5���ֽڵĵ�ַ.    
    ReadBuf(TX_ADDR, buf1, 5);                 //����д��ĵ�ַ      
    
    for(i = 0; i < 5; i++)
    {
        if(buf1[i] != 0xA5)
        {
            break;                       
        }
    }
    
    if(i != 5)
    {
        return 1;                               //NRF24L01����λ            
    }else
    {
        return 0;                               //NRF24L01��λ
    }
}

/******************************************************************************
  * �������ƣ�TxPacket
  * ��������������һ�����ݰ�������ϵͳ��NRF�ķ��ͺͽ��ղ�����ѯ��ʽ���˺���ֻ��Ҫ
              ���͵�����ѹ��ͨ�Ŷ�����
  * ��    �룺uint8_t *txbuf:���͵����ݵ�ַ
  * ��    ����void
  * ��    �أ�0
  * ��    ע�����ڷ��ͳ��ȹ̶�Ϊ32�����Դ˴�Ҳû�з��͵����ݳ���ά�ȿ���ѡ��   
  *    
  *
******************************************************************************/
uint8_t TxPacket(uint8_t *txbuf)
{
    enQueue(&NRF_Mannager.qTx, txbuf, TX_PLOAD_WIDTH);
    return 0;
}

/******************************************************************************
  * �������ƣ�RxPacket
  * ����������NRF24L01���պ�����������ѯ��ʽ��ѯ��ǰ�Ƿ������ݽ��յ�
  * ��    �룺uint8_t *rxbuf:�������ݵĵ�ַ
  * ��    ����void
  * ��    �أ��Ƿ��ȡ�ɹ���0�����ճɹ�
                           1������ʧ��
  * ��    ע��    
  *    
  *
******************************************************************************/
uint8_t RxPacket(uint8_t *rxbuf)
{
    uint8_t state = 0;                                              
    
    state = ReadReg(RF_STATUS);                //��ȡ״̬�Ĵ�����ֵ         
    WriteReg(SPI_WRITE_REG + RF_STATUS, state); //���TX_DS��MAX_RT�жϱ�־
    
    if(state & RX_OK)                                 //���յ�����
    {
        ReadBuf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);//��ȡ����
        WriteReg(FLUSH_RX, 0xff);          //���RX FIFO�Ĵ���
        
        return SUCCESS; 
    }
    return FAILED;                                      //û�յ��κ�����
}

/******************************************************************************
  * �������ƣ�UpdateFreq
  * ��������������NRF24L01���շ�Ƶ��
  * ��    �룺uint8_t freq:�����շ�Ƶ��      ���ݴ�СΪ0-64

  * ��    ����void
  * ��    �أ��Ƿ���ĳɹ���true:���ĳɹ�
                            false������ʧ��
  * ��    ע��    
  *    
  *
******************************************************************************/
bool UpdateFreq(uint8_t freq)
{
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteReg(SPI_WRITE_REG + RF_CH, freq); 
    if(ReadReg(SPI_READ_REG + RF_CH) == freq)
    {
        Flash_Write(FLASH_FREQ_ADDR_OFFSET,&freq, 2);
        g_NRFManager.RC_Frequency = freq;
        
        status = true;
    }
    SET_NRF24L01_CE;
    
    return status;
}

/******************************************************************************
  * �������ƣ�UpdateRxAddr
  * ��������������NRF24L01�Ľ��յ�ַ
  * ��    �룺uint8_t *ptr:���յ�ַ��ŵĿ�ʼ��ַ�����ȹ̶�Ϊ5
  * ��    ����void
  * ��    �أ��Ƿ���ĳɹ���true:���ĳɹ�
                            flase:����ʧ��
  * ��    ע��    
  *    
  *
******************************************************************************/
bool UpdateRxAddr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, ptr, RX_ADR_WIDTH); 
    ReadBuf(RX_ADDR_P0, buf1, 5);
    
    if(memcmp(buf1, ptr, 5) == 0)
    {
        Flash_Write(FLASH_RX_ADDR_OFFSET, ptr, 5);
        memcpy(g_UAVinfo.NRF_MannagerPtr->Rx_Addr, ptr, 5);
        
        status = true;
    }
    SET_NRF24L01_CE;  
    
    return status;
}

/******************************************************************************
  * �������ƣ�UpdateTxAddr
  * ��������������NRF24L01���͵�ַ
  * ��    �룺uint8_t *ptr:���͵�ַ��ŵĿ�ʼ��ַ�����ȹ̶�Ϊ5
  * ��    ����void
  * ��    �أ��Ƿ���ĳɹ���true:���ĳɹ�
                            flase:����ʧ�� 
  * ��    ע��    
  *    
  *
******************************************************************************/
bool UpdateTxAddr(uint8_t *ptr)
{
    uint8_t buf1[5];
    bool status = false;
    
    CLR_NRF24L01_CE;     
    WriteBuf(SPI_WRITE_REG + TX_ADDR, ptr, RX_ADR_WIDTH); 
    ReadBuf(TX_ADDR, buf1, 5);
    
    if(memcmp(buf1, ptr, 5) == 0)
    {
        Flash_Write(FLASH_TX_ADDR_OFFSET, ptr, 5);
        memcpy(g_NRFManager.Tx_Addr, ptr, 5);
        
        status = true;
    }
    SET_NRF24L01_CE;
    
    return status;
}

/******************************************************************************
  * �������ƣ�NRF24L01Init
  * ����������NRF24L01��ʼ��������ʼ��ָ��ָ��
  * ��    �룺NRF24L01_Manager_t *ptr��NRF24L01����ṹ��ָ��
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void NRF24L01Init(NRF24L01_Manager_t *ptr)
{   
    GPIO_setAsOutputPin(GPIO_PORT_P3, NRF2401_CE_PIN);
    GPIO_setAsOutputPin(GPIO_PORT_P4, NRF2401_CSN_PIN);
    SPI_init();
    SET_NRF24L01_CSN; // Spi disable
    CLR_NRF24L01_CE;  // chip enable
    
    while(Check() == FAILED)
    {
        static int ChectCnt = 0;
        
        ChectCnt++;
        
        if(ChectCnt == 5)
        {
            g_NRFManager.Check = false;
            return;
        }
    }
    g_NRFManager.Check = true;
    
    CLR_NRF24L01_CE;        

    WriteBuf(SPI_WRITE_REG + TX_ADDR,(uint8_t*)ptr->Tx_Addr, TX_ADR_WIDTH);                            //дTX�ڵ��ַ 
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0,(uint8_t*)ptr->Rx_Addr, RX_ADR_WIDTH);                         //дRX�ڵ��ַ 
    WriteReg(SPI_WRITE_REG + EN_AA, 0x01);                                                      //ʹ��ͨ��0���Զ�Ӧ��  
    WriteReg(SPI_WRITE_REG + EN_RXADDR, 0x01);                                                  //ʹ��ͨ��0�Ľ��յ�ַ 
    WriteReg(SPI_WRITE_REG + SETUP_RETR, 0x1a);                                                 //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
    WriteReg(SPI_WRITE_REG + RF_CH, ptr->RC_Frequency);                                         //����RFͨ��Ϊ40    
    WriteReg(SPI_WRITE_REG + RF_SETUP, 0x0f);                                                   //0x27  250K   0x07 1M     

    switch(ptr->NRF_Mode)
    {
        case NRF_Mode_RX:
            WriteReg(SPI_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                                //ѡ��ͨ��0����Ч���ݿ�� 
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0f);            
            break;
        case NRF_Mode_TX:
            WriteReg(SPI_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);                                //ѡ��ͨ��0����Ч���ݿ�� 
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0e);            // IRQ�շ�����жϿ���,16λCRC,������
            break;
        case NRF_Mode_RX2:
            WriteReg(FLUSH_TX, 0xff);
            WriteReg(FLUSH_RX, 0xff);
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0f);            // IRQ�շ�����жϿ���,16λCRC,������

            SPI_RW(0x50);
            SPI_RW(0x73);
            WriteReg(SPI_WRITE_REG + 0x1c, 0x01);
            WriteReg(SPI_WRITE_REG + 0x1d, 0x06);
            break;
        case NRF_Mode_TX2:
            WriteReg(SPI_WRITE_REG + NCONFIG, 0x0e);            // IRQ�շ�����жϿ���,16λCRC,������
            WriteReg(FLUSH_TX, 0xff);
            WriteReg(FLUSH_RX, 0xff);

            SPI_RW(0x50);
            SPI_RW(0x73);
            WriteReg(SPI_WRITE_REG + 0x1c, 0x01);
            WriteReg(SPI_WRITE_REG + 0x1d, 0x06);
            break;
        default:
            break;
    }
    
    SET_NRF24L01_CE;  
    
    ptr->receive_buff = RxPacket;
    ptr->send_buff = TxPacket;
    ptr->update_rx_Addr = UpdateRxAddr;
    ptr->update_tx_Addr = UpdateTxAddr;
    ptr->update_frequency = UpdateFreq;
}

/******************************************************************************
  * �������ƣ�NRF24L01TxPacket
  * �����������������ݣ��Է���ģʽ1����
  * ��    �룺uint8_t * tx_buf:����buff�ĵ�ַ
              uint8_t len:����buff�ĳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null
  *    
  *
******************************************************************************/
void NRF24L01TxPacket(uint8_t * tx_buf, uint8_t len)
{    
    CLR_NRF24L01_CE;    
    WriteBuf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)g_NRFManager.Tx_Addr, TX_ADR_WIDTH);   // װ�ؽ��ն˵�ַ
    WriteBuf(WR_TX_PLOAD, tx_buf, len);                                 // װ������    
    SET_NRF24L01_CE; 
}

/******************************************************************************
  * �������ƣ�NRF24L01TxPacketAp
  * �����������������ݣ��Է���ģʽ2����
  * ��    �룺uint8_t * tx_buf:����buff�ĵ�ַ
              uint8_t len:����buff�ĳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null 
  *    
  *
******************************************************************************/
void NRF24L01TxPacketAp(uint8_t * tx_buf, uint8_t len)
{    
    CLR_NRF24L01_CE;    
    WriteBuf(0xa8, tx_buf, len);              // װ������
    SET_NRF24L01_CE; 
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
