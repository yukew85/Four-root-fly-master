#ifndef __SYSCONFIG_H__
#define __SYSCONFIG_H__
//
#include "data_type.h"

#define YAW_CORRECT
#define IMU_SW          // 姿态解算使用软件解算，不再使用MPU6050的硬件解算单元DMP
#define NEW_RC

//#define UART_DEBUG	//开启改宏，则可以使用串口助手打印调试。否则使用Crazepony上位机
//#define S02A
//#define FBM320
#define MS5611
#define F150_V1_5

enum
{
    SRC_PC,
    SRC_APP
};

extern uint8 btSrc;

#endif  // __SYSCONFIG_H__

