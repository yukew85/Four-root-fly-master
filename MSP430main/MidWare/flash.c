/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�flash.c
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
Flash��������



*/
//�ⲿ�ļ�����
#include "include.h"
#include "flash.h"
#include "string.h"


//�궨����
#define LENGTH 50
#define INFOA_START   (0x1980)

//Extern����



//˽�к�����



//˽�б�����




/******************************************************************************
  * �������ƣ�Flash_Write
  * ������������Flash��д����
  * ��    �룺
  * uint8_t offset:д������ƫ��
  * uint8_t *ptr:Ҫд�����ݵĵ�ַ
  * uint8_t length:Ҫд�����ݵĳ���
  * ��    ����void
  * ��    �أ�void
  * ��    ע��null    
  *    
  *
******************************************************************************/
void Flash_Write(uint8_t offset, uint8_t *ptr, uint8_t length)
{
    uint8_t temp[LENGTH];
    uint8_t status = 1;
    
    memcpy(temp, (uint8_t*)INFOA_START, LENGTH);
    memcpy(temp + offset, ptr, length);
    
    FlashCtl_unlockInfoA();
    do
    {
        FlashCtl_eraseSegment((uint8_t *)INFOA_START);
        status = FlashCtl_performEraseCheck((uint8_t *)INFOA_START,
            128);
    } while (status == STATUS_FAIL);

    FlashCtl_write8(temp,  (uint8_t *)INFOA_START, LENGTH);
    FlashCtl_lockInfoA();
}

/******************* (C) ��Ȩ���� 2018 �����пƺƵ�Ƽ����޹�˾ *******************/
