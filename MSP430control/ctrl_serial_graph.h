/*
    文件名称：   ctrl_serial_graph.h
    文件作者：   中科浩电 www.bj-zkhd.com (guozhenjiang@bj-zkhd.com)
    文件功能：   输出调试曲线相关内容
    修改日期：   2017-7-7
    修改内容：   修改注释
*/

#ifndef __SERIAL_GRAPH_H__
#define __SERIAL_GRAPH_H__

#include "ctrl_basic.h"

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
extern void GraphGen(int16 ch1,int16 ch2,int16 ch3,int16 ch4,int16 ch5,int16 ch6,int16 ch7,int16 ch8);

#endif  // __SERIAL_GRAPH_H__
