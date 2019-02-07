/*
    �ļ����ƣ�   ctrl_flash.h
    �ļ����ߣ�   �пƺƵ� www.bj-zkhd.com
    �ļ����ܣ�   Flash ��д��أ���Ҫ���ڱ���ң����΢������
    �޸����ڣ�   2017-7-7
    �޸����ݣ�   �޸�ע��
*/

#ifndef __FLASH_GZJ_H__
#define __FLASH_GZJ_H__

#include "control.h"
#include "stdbool.h"

#define RC_OF_SAVED         0xAAAA      // �Ѿ������΢��ֵ
#define RC_OF_NOT_SAVED     0x5555      // û�б����΢��ֵ

// �������Ͷ���
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

extern YesNo_t En_RC_Offset_Update;             // �Ƿ���Ը���ң����΢��ֵ
extern YesNo_t Saving_RC_Offset;                // �Ƿ����ڱ���ң����΢��ֵ

extern void Data_Quick_Sort_uint16(uint16* pDataBuf, uint16 Num);
extern uint16 Data_Filter_uint16(uint16* pDataBuf, uint16 Num);

extern YesNo_t Save_RC_Offset_To_Flash(RC_Para_t* para);                 // ����ң����΢��ֵ�� Flash
extern YesNo_t Read_RC_Offset_From_Flash(RC_Para_t* pOffset);             // �� Flash ��ȡң����΢��ֵ
extern bool Write_Flash(uint8_t page,uint8_t *pData,uint16 len);
#endif  // __FLASH_GZJ_H__

