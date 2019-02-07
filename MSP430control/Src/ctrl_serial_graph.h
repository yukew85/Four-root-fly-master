/**
  ******************************************************************************
  * @file    ctrl_serial_graph.h
  * @brief   本源码中定义了基于F150遥控器的USART1输出调试曲线相关内容。 
  * @functionList   
  *     1.
  *
  * @currentVersion  V1.1
  * @author  北京中科浩电科技有限公司   www.bj-zkhd.com  (guozhenjiang@bj-zkhd.com)
  * @date    2017-11-5
  * @Update  修改注释
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

#ifndef __SERIAL_GRAPH_H__
#define __SERIAL_GRAPH_H__
#include "stm32f1xx_hal.h"

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
extern void GraphGen(int16_t ch1,int16_t ch2,int16_t ch3,int16_t ch4,int16_t ch5,int16_t ch6,int16_t ch7,int16_t ch8);

#endif  // __SERIAL_GRAPH_H__
