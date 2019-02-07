/*
    文件名称：   ctrl_flash.h
    文件作者：   中科浩电 www.bj-zkhd.com
    文件功能：   Flash 读写相关，主要用于保存遥控器微调参数
    修改日期：   2017-7-7
    修改内容：   修改注释
*/

#ifndef __FLASH_GZJ_H__
#define __FLASH_GZJ_H__

#include "control.h"
#include "stdbool.h"

#define RC_OF_SAVED         0xAAAA      // 已经保存过微调值
#define RC_OF_NOT_SAVED     0x5555      // 没有保存过微调值

// 数据类型定义
typedef signed              char    int8;
typedef signed      short   int     int16;
typedef signed              int     int32;
typedef signed      long    long    int64;

typedef unsigned            char    uint8;
typedef unsigned    short   int     uint16;
typedef unsigned            int     uint32;
typedef unsigned    long    long    uint64;

typedef signed              char    int8_t;
typedef signed      short   int     int16_t;
typedef signed              int     int32_t;
typedef signed      long    long    int64_t;

typedef unsigned            char    uint8_t;
typedef unsigned    short   int     uint16_t;
typedef unsigned            int     uint32_t;
typedef unsigned    long	long    uint64_t;

typedef struct
{
    uint16 IsSaved;             // 0xAAAA: Saved.   0x5555: Not Saved.
    int16 RC_Rol_Offset;
    int16 RC_Pit_Offset;
    
    uint16 CRCVal;              // ^ Front Data.
}RC_Offset_t;

extern YesNo_t En_RC_Offset_Update;             // 是否可以更新遥控器微调值
extern YesNo_t Saving_RC_Offset;                // 是否正在保存遥控器微调值

extern void Data_Quick_Sort_uint16(uint16* pDataBuf, uint16 Num);
extern uint16 Data_Filter_uint16(uint16* pDataBuf, uint16 Num);

extern YesNo_t Save_RC_Offset_To_Flash(RC_Para_t* para);                 // 保存遥控器微调值到 Flash
extern YesNo_t Read_RC_Offset_From_Flash(RC_Para_t* pOffset);             // 从 Flash 读取遥控器微调值
extern bool Write_Flash(uint8_t page,uint8_t *pData,uint16 len);
#endif  // __FLASH_GZJ_H__

