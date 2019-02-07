/*
    文件名称：   ctrl_serial_graph.c
    文件作者：   中科浩电 www.bj-zkhd.com (guozhenjiang@bj-zkhd.com)
    文件功能：   输出调试曲线相关内容
    修改日期：   2017-7-7
    修改内容：   修改注释
*/

#include "ctrl_serial_graph.h"
#include "ctrl_usart.h"

// 山外多功能调试助手 数据结构
typedef struct
{
    int16 St;       // 帧头 固定内容  0x03 0xFC
    int16 Ch[8];    // 曲线数据 8个通道
    int16 Sp;       // 帧尾 固定内容  0x03 0xFC
}ShanWaiGraph_Pkg_t;

// 山外多功能调试助手 通信协议
typedef union
{
    ShanWaiGraph_Pkg_t Pkg;
    uint8 Buf[sizeof(ShanWaiGraph_Pkg_t)];
}ShanWaiGraph_t;

// 定义通信数据
static ShanWaiGraph_t ShanWaiGraph;

/*************************************************
* Function Name:    GraphGen
* Description:      向 山外多功能调试助手 发送曲线信息
* Input:            ch1:    曲线第一通道值
*                   ch2:    曲线第二通道值
*                   ch3:    曲线第三通道值
*                   ch4:    曲线第四通道值
*                   ch5:    曲线第五通道值
*                   ch6:    曲线第六通道值
*                   ch7:    曲线第七通道值
*                   ch8:    曲线第八通道值
* Output:           void
*************************************************/
extern void GraphGen(int16 ch1,int16 ch2,int16 ch3,int16 ch4,int16 ch5,int16 ch6,int16 ch7,int16 ch8)
{
    
    ShanWaiGraph.Pkg.St = 0xFC03;   // 帧头
    ShanWaiGraph.Pkg.Sp = 0x03FC;   // 帧尾
    
    ShanWaiGraph.Pkg.Ch[0] = ch1;   // 通道数据
    ShanWaiGraph.Pkg.Ch[1] = ch2;
    ShanWaiGraph.Pkg.Ch[2] = ch3;
    ShanWaiGraph.Pkg.Ch[3] = ch4;
    ShanWaiGraph.Pkg.Ch[4] = ch5;
    ShanWaiGraph.Pkg.Ch[5] = ch6;
    ShanWaiGraph.Pkg.Ch[6] = ch7;
    ShanWaiGraph.Pkg.Ch[7] = ch8;
    
    // 通过串口发送曲线数据到上位机
    Usart_Send_Data(&huart3, ShanWaiGraph.Buf, sizeof(ShanWaiGraph_t));
}
