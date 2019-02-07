/*
    �ļ����ƣ�   ctrl_serial_graph.c
    �ļ����ߣ�   �пƺƵ� www.bj-zkhd.com (guozhenjiang@bj-zkhd.com)
    �ļ����ܣ�   ������������������
    �޸����ڣ�   2017-7-7
    �޸����ݣ�   �޸�ע��
*/

#include "ctrl_serial_graph.h"
#include "ctrl_usart.h"

// ɽ��๦�ܵ������� ���ݽṹ
typedef struct
{
    int16 St;       // ֡ͷ �̶�����  0x03 0xFC
    int16 Ch[8];    // �������� 8��ͨ��
    int16 Sp;       // ֡β �̶�����  0x03 0xFC
}ShanWaiGraph_Pkg_t;

// ɽ��๦�ܵ������� ͨ��Э��
typedef union
{
    ShanWaiGraph_Pkg_t Pkg;
    uint8 Buf[sizeof(ShanWaiGraph_Pkg_t)];
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
extern void GraphGen(int16 ch1,int16 ch2,int16 ch3,int16 ch4,int16 ch5,int16 ch6,int16 ch7,int16 ch8)
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
    Usart_Send_Data(&huart3, ShanWaiGraph.Buf, sizeof(ShanWaiGraph_t));
}
