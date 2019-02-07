#include "flash.h"
#include "string.h"


#define BUFFLENGTH      100
void Flash_Write(uint8_t offset,uint8_t *ptr,uint64_t length)
{
    uint8_t Buff[BUFFLENGTH];
    uint16_t *u16_ptr;
    
    memcpy(Buff,(uint8_t*)PAGE_ADDRESS,BUFFLENGTH);
    memcpy(Buff+offset,ptr,length);
    
    u16_ptr = (uint16_t*)Buff;
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef f;
    f.TypeErase = FLASH_TYPEERASE_PAGES;//页面擦除
    f.PageAddress = PAGE_ADDRESS;
    f.NbPages = 1;//擦除多少页面
    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&f,&PageError);
    
    for(int i = 0;i<BUFFLENGTH;i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,PAGE_ADDRESS + i*2,*u16_ptr++);
    }
    
    HAL_FLASH_Lock();
}

uint8_t* Flash_Read()
{
    return (uint8_t*)PAGE_ADDRESS;
}
