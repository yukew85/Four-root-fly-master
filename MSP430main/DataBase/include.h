/**
  ******************************************************************************
  * Copyright (c) 2018,�����пƺƵ�Ƽ����޹�˾
  * All rights reserved.
  * �ļ����ƣ�include.h
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
#ifndef _INCLUDE_
#define _INCLUDE_
//�ⲿ�ļ�����
//��һ�������õ�ͷ�ļ�
#include "driverlib.h"
#include "queue.h"
#include "delay.h"
#include "board.h"

//�궨����
#define SUCCESS 0
#define FAILED  1

//READONLY
#include "stdint.h"
#include "stdbool.h"

void Hardware_Init(void);
void PollingKernel(void);
#endif