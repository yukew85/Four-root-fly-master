#ifndef _FLASH_H
#define _FLASH_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define PAGE_ADDRESS    (0x0801F000)

void Flash_Write(uint8_t offset,uint8_t *ptr,uint64_t length);
uint8_t* Flash_Read(void);

#endif
