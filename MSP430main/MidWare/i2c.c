/**
  ******************************************************************************
  * Copyright (c) 2018,北京中科浩电科技有限公司
  * All rights reserved.
  * 文件名称：i2c.c
  * 摘    要：
  *
  * 当前版本：V1.0
  * 作    者：北京中科浩电科技有限公司研发部 
  * 完成日期：    
  * 修改说明：
  * 
  *
  * 历史版本：
  *
  *
  *******************************************************************************/

/*==============================================================================
                         ##### How to use this driver #####
==============================================================================
i2c初始化函数


*/
//外部文件引用
#include "i2c.h"
#include "include.h"


//私有函数区
void IIC_SendByte(uint32_t dat);
void IIC_RecvACK();
void IIC_SendACK(uint32_t ack);
void IIC_Stop();
uint32_t IIC_RecvByte();
void IIC_Start();

//私有变量区
unsigned char Ack_Count = 0; 

/******************************************************************************
  * 函数名称：I2C_Init
  * 函数描述：初始化I2c总线
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void I2C_Init()
{
  P3DIR |= (1 << SCL);
}

/******************************************************************************
  * 函数名称：delay
  * 函数描述：I2c延时
  * 输    入：
  * uint32_t n:延时的周期
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void delay(uint32_t n)
{
  while (n--)
    ;
}

/******************************************************************************
  * 函数名称：IIC_Start
  * 函数描述：I2c开始信号
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_Start()
{
    P3DIR |= (1 << SDA);
    IIC |= (1 << SDA);
    delay(1);
    IIC |= (1 << SCL);
    delay(1);
    IIC &= ~(1 << SDA);
    delay(1);
    IIC &= ~(1 << SCL);
    delay(1);
}

/******************************************************************************
  * 函数名称：IIC_Stop
  * 函数描述：I2c停止信号
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_Stop()
{
  P3DIR |= (1 << SDA);
  IIC &= ~(1 << SCL);
  IIC &= ~(1 << SDA);
  delay(1);
  IIC |= (1 << SCL);
  delay(1);
  IIC |= (1 << SDA);
}

/******************************************************************************
  * 函数名称：IIC_SendACK
  * 函数描述：发送ACK
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_SendACK(uint32_t ack)
{
  P3DIR |= (1 << SDA);
  IIC &= ~(1 << SCL);
  delay(1);
  if (ack)
  {
    IIC |= (1 << SDA);
  }
  else
  {
    IIC &= ~(1 << SDA);
  }
  delay(1);
  IIC |= (1 << SCL);
  delay(1);
  IIC &= ~(1 << SCL);
  delay(1);
}

/******************************************************************************
  * 函数名称：IIC_RecvACK
  * 函数描述：接收ACk
  * 输    入：void
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_RecvACK()
{
  P3DIR &= ~(1 << SDA);

  IIC |= (1 << SCL);
  while ((P3IN & (1 << SDA)))
  {
    Ack_Count++;
    if (Ack_Count > 250)
    {
      IIC_Stop();
      Ack_Count = 0;
      break;
    }
  }

  IIC &= ~(1 << SCL);
  delay(1);
}

/******************************************************************************
  * 函数名称：IIC_SendByte
  * 函数描述：I2c发送一个byte
  * 输    入：
  * uint32_t dat:要发送的数据
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void IIC_SendByte(uint32_t dat)
{
  uint32_t i;

  P3DIR |= (1 << SDA);
  i = 8;
  IIC &= ~(1 << SCL);
  delay(1);
  while (i--) 
  {
    if (((dat & 0x80) >> 7))
    {
      IIC |= (1 << SDA);
    }
    else
    {
      IIC &= ~(1 << SDA);
    }
    dat <<= 1; 
    delay(1);
    IIC |= (1 << SCL);
    delay(1);
    IIC &= ~(1 << SCL);
    delay(1);
  }

  IIC |= (1 << SDA);
  IIC_RecvACK();
}

/******************************************************************************
  * 函数名称：IIC_RecvByte
  * 函数描述：I2c接收一个byte
  * 输    入：void
  * 输    出：void
  * 返    回：接收到的数据
  * 备    注：null    
  *    
  *
******************************************************************************/
uint32_t IIC_RecvByte()
{
  uint32_t i;
  uint32_t dat = 0;

  P3DIR &= ~(1 << SDA);
  i = 8;
  while (i--) 
  {
    IIC &= ~(1 << SCL);
    delay(1);
    IIC |= (1 << SCL);
    dat <<= 1;
    if (P3IN & BIT0)
    {
      dat |= 1;
    }
    else
    {
      dat |= 0;
    }
    delay(1);
  }
  return dat;
}

/******************************************************************************
  * 函数名称：I2C_Write_Byte
  * 函数描述：I2c往从机寄存器地址写数据
  * 输    入：
  * uint32_t Slaveaddr:从机地址
  * uint32_t REG_Address:寄存器地址
  * uint32_t REG_data：寄存器数据
  * 输    出：void
  * 返    回：void 
  * 备    注：null    
  *    
  *
******************************************************************************/
void I2C_Write_Byte(uint8_t Slaveaddr, uint8_t REG_Address, uint8_t REG_data)
{
  IIC_Start(); 
  IIC_SendByte(Slaveaddr);   
  IIC_SendByte(REG_Address); 
  IIC_SendByte(REG_data);    
  IIC_Stop(); 
}

/******************************************************************************
  * 函数名称：I2C_Read_Byte
  * 函数描述：I2c往从机寄存器读数据
  * 输    入：
  * uint32_t Slaveaddr：从机地址
  * uint32_t REG_Address:寄存器地址
  * 输    出：void
  * 返    回：读取到的数据
  * 备    注：null    
  *    
  *
******************************************************************************/
uint8_t I2C_Read_Byte(uint8_t Slaveaddr, uint8_t REG_Address)
{
  uint32_t REG_data;
  IIC_Start();                 
  IIC_SendByte(Slaveaddr);    
  IIC_SendByte(REG_Address);   
  IIC_Start();                 
  IIC_SendByte(Slaveaddr + 1); 
  REG_data = IIC_RecvByte();  
  IIC_SendACK(1);
  IIC_Stop(); 
  return REG_data;
}


/******************* (C) 版权所有 2018 北京中科浩电科技有限公司 *******************/
