#ifndef __CTRL_FLASH_H
#define __CTRL_FLASH_H

#include "stm32f1xx_hal.h"
#include "control.h"

//extern uint32_t writeFlashData;
extern uint32_t addr;

void Flash_write(RC_Para_t *RC_Para);

void printFlashTest(void);

extern void flash_test(void);

#endif
