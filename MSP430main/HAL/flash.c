#include "flash.h"
#include "msp430f5529_flash.h"
#include "string.h"

#define LENGTH  50

void Flash_Write(uint8_t offset,uint8_t *ptr,uint8_t length)
{
  uint8_t temp[LENGTH];
  uint8_t *base = &Flash_Read(MAX_SEGMENT,0,uint8_t);
  
  memcpy(temp,base,LENGTH);
  memcpy(temp+offset,ptr,length);

  Flash_Erase_Segment(MAX_SEGMENT);
  Flash_Write_buf(MAX_SEGMENT,0,LENGTH,temp);
}