/**
  ******************************************************************************
  * @file    beep.c 
  * @brief   本源码中实现了F150遥控器的
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  北京中科浩电科技有限公司   www.bj-zkhd.com  
  * @date    2017-11-5
  * @Update
  *
  * @historyVersion        @author        @date
  *    
  *    
  ******************************************************************************
  * @attention
  *
  * 本程所提供的源代吗均为本程序作者编写，仅供公司内部使用。使用本程序的用户必须明白，
  * 我们不能保证所提供的源码的准确性、安全性和完整性，因此我们将不负责承担任何直接、间 
  * 接因使用这些源码对自己和他人造成任何形式的损失或伤害。任何人在使用本代码时，请注明
  * 作者和出处。
  * <h2><center>版权所有(C) 2017 北京中科浩电科技有限公司</center></h2>
  ******************************************************************************
  */  

/*
    文件名称：   ctrl_serial_graph.c
    文件作者：   中科浩电 www.bj-zkhd.com (guozhenjiang@bj-zkhd.com)
    文件功能：   输出调试曲线相关内容
    修改日期：   2017-7-7
    修改内容：   修改注释
*/

#include "ctrl_serial_graph.h"
#include "usart.h"
// 山外多功能调试助手 数据结构
typedef struct
{
    int16_t St;       // 帧头 固定内容  0x03 0xFC
    int16_t Ch[8];    // 曲线数据 8个通道
    int16_t Sp;       // 帧尾 固定内容  0x03 0xFC
}ShanWaiGraph_Pkg_t;

// 山外多功能调试助手 通信协议
typedef union
{
    ShanWaiGraph_Pkg_t Pkg;
    uint8_t Buf[sizeof(ShanWaiGraph_Pkg_t)];
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
extern void GraphGen(int16_t ch1,int16_t ch2,int16_t ch3,int16_t ch4,int16_t ch5,int16_t ch6,int16_t ch7,int16_t ch8)
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
	  HAL_UART_Transmit(&huart1, ShanWaiGraph.Buf, sizeof(ShanWaiGraph_t), 0xff);
}
