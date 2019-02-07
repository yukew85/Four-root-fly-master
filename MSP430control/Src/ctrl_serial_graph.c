/**
  ******************************************************************************
  * @file    beep.c 
  * @brief   ��Դ����ʵ����F150ң������
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  �����пƺƵ�Ƽ����޹�˾   www.bj-zkhd.com  
  * @date    2017-11-5
  * @Update
  *
  * @historyVersion        @author        @date
  *    
  *    
  ******************************************************************************
  * @attention
  *
  * �������ṩ��Դ�����Ϊ���������߱�д��������˾�ڲ�ʹ�á�ʹ�ñ�������û��������ף�
  * ���ǲ��ܱ�֤���ṩ��Դ���׼ȷ�ԡ���ȫ�Ժ������ԣ�������ǽ�������е��κ�ֱ�ӡ��� 
  * ����ʹ����ЩԴ����Լ�����������κ���ʽ����ʧ���˺����κ�����ʹ�ñ�����ʱ����ע��
  * ���ߺͳ�����
  * <h2><center>��Ȩ����(C) 2017 �����пƺƵ�Ƽ����޹�˾</center></h2>
  ******************************************************************************
  */  

/*
    �ļ����ƣ�   ctrl_serial_graph.c
    �ļ����ߣ�   �пƺƵ� www.bj-zkhd.com (guozhenjiang@bj-zkhd.com)
    �ļ����ܣ�   ������������������
    �޸����ڣ�   2017-7-7
    �޸����ݣ�   �޸�ע��
*/

#include "ctrl_serial_graph.h"
#include "usart.h"
// ɽ��๦�ܵ������� ���ݽṹ
typedef struct
{
    int16_t St;       // ֡ͷ �̶�����  0x03 0xFC
    int16_t Ch[8];    // �������� 8��ͨ��
    int16_t Sp;       // ֡β �̶�����  0x03 0xFC
}ShanWaiGraph_Pkg_t;

// ɽ��๦�ܵ������� ͨ��Э��
typedef union
{
    ShanWaiGraph_Pkg_t Pkg;
    uint8_t Buf[sizeof(ShanWaiGraph_Pkg_t)];
}ShanWaiGraph_t;

// ����ͨ������
static ShanWaiGraph_t ShanWaiGraph;

/*************************************************
* Function Name:    GraphGen
* Description:      �� ɽ��๦�ܵ������� ����������Ϣ
* Input:            ch1:    ���ߵ�һͨ��ֵ
*                   ch2:    ���ߵڶ�ͨ��ֵ
*                   ch3:    ���ߵ���ͨ��ֵ
*                   ch4:    ���ߵ���ͨ��ֵ
*                   ch5:    ���ߵ���ͨ��ֵ
*                   ch6:    ���ߵ���ͨ��ֵ
*                   ch7:    ���ߵ���ͨ��ֵ
*                   ch8:    ���ߵڰ�ͨ��ֵ
* Output:           void
*************************************************/
extern void GraphGen(int16_t ch1,int16_t ch2,int16_t ch3,int16_t ch4,int16_t ch5,int16_t ch6,int16_t ch7,int16_t ch8)
{
    
    ShanWaiGraph.Pkg.St = 0xFC03;   // ֡ͷ
    ShanWaiGraph.Pkg.Sp = 0x03FC;   // ֡β
    
    ShanWaiGraph.Pkg.Ch[0] = ch1;   // ͨ������
    ShanWaiGraph.Pkg.Ch[1] = ch2;
    ShanWaiGraph.Pkg.Ch[2] = ch3;
    ShanWaiGraph.Pkg.Ch[3] = ch4;
    ShanWaiGraph.Pkg.Ch[4] = ch5;
    ShanWaiGraph.Pkg.Ch[5] = ch6;
    ShanWaiGraph.Pkg.Ch[6] = ch7;
    ShanWaiGraph.Pkg.Ch[7] = ch8;
    
    // ͨ�����ڷ����������ݵ���λ��
	  HAL_UART_Transmit(&huart1, ShanWaiGraph.Buf, sizeof(ShanWaiGraph_t), 0xff);
}
