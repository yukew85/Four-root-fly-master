/*
    文件名称：   ctrl_flash.c
    文件作者：   中科浩电 www.bj-zkhd.com
    文件功能：   Flash 读写相关，主要用于保存遥控器微调参数
    修改日期：   2017-7-7
    修改内容：   修改注释
*/

#include "flash_gzj.h"
#include "stm32f1xx_hal.h"
#include "stm32_hal_legacy.h"
#include "stdbool.h"

// 1024 Byte for User, at the last page(1kB(1024 Byte) per page).
#define USER_FLASH_ADDR     (FLASH_BASE + ((128 - 1) * 1024))       // 用户 Flash 数据 起始地址
#define USER_FLASH_SAVE_MAX (1024 - 4)                              // 用户 Flash 数据 最大长度

extern YesNo_t En_RC_Offset_Update = No;        // 是否可以更新遥控器微调值
extern YesNo_t Saving_RC_Offset = No;           // 是否正在保存遥控器微调值

static YesNo_t Flash_Erase_User_Zone(void);     // 擦除用户 Flash 区内容
bool Erase_Flash(uint8_t page);
// 擦除用户 Flash 区内容
static YesNo_t Flash_Erase_User_Zone(void)
{
    FLASH_EraseInitTypeDef FLASH_Erase_UserZone;
    uint32 PageError;
    YesNo_t OptResult;
    
    OptResult = No;
    
    FLASH_Erase_UserZone.TypeErase = FLASH_TYPEERASE_PAGES;    
    FLASH_Erase_UserZone.PageAddress = USER_FLASH_ADDR;
    FLASH_Erase_UserZone.NbPages = 1;
    
    if(HAL_OK == HAL_FLASHEx_Erase(&FLASH_Erase_UserZone, &PageError))
        OptResult = Yes;
    return OptResult;
}

bool Erase_Flash(uint8_t page)
{
    FLASH_EraseInitTypeDef FLASH_Erase_UserZone;
    uint32 PageError;
    bool OptResult;
    
    OptResult = false;
    
    FLASH_Erase_UserZone.TypeErase = FLASH_TYPEERASE_PAGES;    
    FLASH_Erase_UserZone.PageAddress = (FLASH_BASE + ((128 - page) * 1024));
    FLASH_Erase_UserZone.NbPages = 1;
    
    if(HAL_OK == HAL_FLASHEx_Erase(&FLASH_Erase_UserZone, &PageError))
        OptResult = true;
    
    return OptResult;
}

bool Write_Flash(uint8_t page,uint8_t *pData,uint16 len)
{
    uint16 NeedWordNum = 0;
    uint32* pWord = 0;
    uint16 i;
    
    // 1 Unlock Flash   解锁FLASH
    if(HAL_OK != HAL_FLASH_Unlock())
        return false;
    
    // 2 Erase Flash    擦除用户区(擦除时总是按块 整块擦除)
    if(false == Erase_Flash(page))
        return false;
    
    // 3 Write to Flash 写数据到 Flash 中
    NeedWordNum = len / 4;
    if(len % 4)
        NeedWordNum += 1;
    
    pWord = (uint32*)pData;

    for(i=0; i<NeedWordNum; i++)
    {
        if(HAL_OK != HAL_FLASH_Program(TYPEPROGRAM_WORD, (FLASH_BASE + ((128 - page) * 1024))+(i*4), pWord[i]))
        {
            return false;
        }
            
    }
    
    // 4 Lock Flash     锁定 Flash 曹组
    if(HAL_OK != HAL_FLASH_Lock())
        return false;
    
    return Yes;
}

// 将遥控器微调值保存到 Flash 中具体操作
extern YesNo_t Save_RC_Offset_To_Flash(RC_Para_t* para)
{
    static RC_Para_t RC_Offset_Cache;
    uint16 NeedWordNum = 0;
    uint32* pWord = 0;
    uint16 i;

	RC_Offset_Cache.IsSaved = RC_OF_SAVED;
	RC_Offset_Cache.rol.Max = para->rol.Max;
	RC_Offset_Cache.rol.Mid = para->rol.Mid;
	RC_Offset_Cache.rol.Min = para->rol.Min;
	                                                  
	RC_Offset_Cache.pit.Max = para->pit.Max;
	RC_Offset_Cache.pit.Mid = para->pit.Mid;
	RC_Offset_Cache.pit.Min = para->pit.Min;
	                                              
	RC_Offset_Cache.thr.Max = para->thr.Max;
	RC_Offset_Cache.thr.Mid = para->thr.Mid;
	RC_Offset_Cache.thr.Min = para->thr.Min;
	                                            
	RC_Offset_Cache.yaw.Max = para->yaw.Max;
	RC_Offset_Cache.yaw.Mid = para->yaw.Mid;
	RC_Offset_Cache.yaw.Min = para->yaw.Min;
	RC_Offset_Cache.crc     = para->crc;
        
    // 1 Unlock Flash   解锁FLASH
    if(HAL_OK != HAL_FLASH_Unlock())
        return No;
    
    // 2 Erase Flash    擦除用户区(擦除时总是按块 整块擦除)
    if(No == Flash_Erase_User_Zone())
        return No;
    
    // 3 Write to Flash 写数据到 Flash 中
    NeedWordNum = sizeof(RC_Para_t) / 4;
    if(sizeof(RC_Para_t) % 4)
        NeedWordNum += 1;
    
    pWord = (uint32*)&RC_Offset_Cache;

    for(i=0; i<NeedWordNum; i++)
    {
        if(HAL_OK != HAL_FLASH_Program(TYPEPROGRAM_WORD, USER_FLASH_ADDR+(i*4), pWord[i]))
            return No;
    }
    
    // 4 Lock Flash     锁定 Flash 曹组
    if(HAL_OK != HAL_FLASH_Lock())
        return No;
    
    return Yes;
}

// 从 Flash 中读取遥控器微调值
extern YesNo_t Read_RC_Offset_From_Flash(RC_Para_t* pOffset)
{
    RC_Para_t* pOffsetCache;
    
    pOffsetCache = (RC_Para_t* )USER_FLASH_ADDR;
    
    // 通过自定义的校验方式检查 用户自定义 Flash 区域是否正确保存了遥控器微调参数	
	
    if(     (RC_OF_SAVED == pOffsetCache->IsSaved)
        &&  ((uint16)pOffsetCache->crc == (uint16)(RC_Parc_CRC(pOffsetCache)))   )
    {
		pOffset->IsSaved = pOffsetCache->IsSaved;
		pOffset->rol.Max = pOffsetCache->rol.Max;
		pOffset->rol.Mid = pOffsetCache->rol.Mid;
		pOffset->rol.Min = pOffsetCache->rol.Min;
                                               
		pOffset->pit.Max = pOffsetCache->pit.Max;
		pOffset->pit.Mid = pOffsetCache->pit.Mid;
		pOffset->pit.Min = pOffsetCache->pit.Min;
                                            
		pOffset->thr.Max = pOffsetCache->thr.Max;
		pOffset->thr.Mid = pOffsetCache->thr.Mid;
		pOffset->thr.Min = pOffsetCache->thr.Min;
                                            
		pOffset->yaw.Max = pOffsetCache->yaw.Max;
		pOffset->yaw.Mid = pOffsetCache->yaw.Mid;
		pOffset->yaw.Min = pOffsetCache->yaw.Min;
		pOffset->crc     = pOffsetCache->crc;
        return Yes;
    }
    
    return No;
}

extern void Data_Quick_Sort_uint16(uint16* pDatas, uint16 Num)
{
    uint16 CntL, CntR;
    uint16 Compare;
    
    if(Num < 2)
        return;
	
    CntL = 0;
    CntR = Num - 1;
	
	Compare = pDatas[CntL];  		// first item saved, and be used for comparer
   
    while(CntL < CntR)
    {
        while(CntL < CntR)
        {
            if(pDatas[CntR] < Compare)
            {
                pDatas[CntL] = pDatas[CntR];
                break;
            }
            CntR--;
        }
        
        while(CntL < CntR)
        {
            if(pDatas[CntL] > Compare)
            {
                pDatas[CntR] = pDatas[CntL];
                break;
            }
            CntL++;
        }
    }
    
    pDatas[CntL++] = Compare;                           // rember to cover the last saved element with Comparer
    Data_Quick_Sort_uint16(&pDatas[0], CntL);           // pDatas[0]        ~   pDatas[CntL-1]
    Data_Quick_Sort_uint16(&pDatas[CntL], Num-(CntL));  // pDatas[CntL] ~   pDatas[Num-1]
}

extern uint16 Data_Filter_uint16(uint16* pDataBuf, uint16 Num)
{
    #define DATA_NUM_MAX    100
    
    static uint16 DataCache[DATA_NUM_MAX];
    static uint32 Sum;
    uint16 i;
    
    if(Num > DATA_NUM_MAX)
      return 0;
    
    if(1 == Num)
    {
        return pDataBuf[0];
    }
    else if(2 == Num)
    {
        Sum = (pDataBuf[0] + pDataBuf[1]) / 2;
        return Sum;
    }
    else if(Num >= 3)
    {
        for(i=0; i<Num; i++)
            DataCache[i] = pDataBuf[i];
        
        Data_Quick_Sort_uint16(DataCache, Num);
        Sum = 0.00;
        for(i=1; i<=Num-2; i++)
            Sum += DataCache[i];
        
        return Sum/(Num-2);
    }
    
    return 0;
}

