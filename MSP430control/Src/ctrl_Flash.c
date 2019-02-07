#include "ctrl_Flash.h"

extern uint32_t addr = 0x08007C00;
//extern uint32_t USER_FLASH_ADDR = 0x08007C00; 

static uint32_t  flash_test_addr = 0x08007C00;

uint16_t flash_test_w[] = {1, 2, 3, 4, 5};
uint16_t flash_test_r[sizeof(flash_test_w)/sizeof(uint16_t)] = {0};

static void Flash_Program(uint16_t *pData, uint16_t len);

extern void flash_test(void)
{	
	//擦除Flash
	//初始化Flash结构体
	FLASH_EraseInitTypeDef f;
	uint8_t i;
	uint16_t*  pRead = (uint16_t*)flash_test_addr;
	
	for(i=0; i<sizeof(flash_test_r); i++)
	{
		flash_test_r[i] = pRead[i];
	}	
	
	flash_test_w[0] = flash_test_r[0] + 1;
	
	
	//解锁Flash
	HAL_FLASH_Unlock();
	
	f.TypeErase = FLASH_TYPEERASE_PAGES;  //擦除类型mass和page
	f.PageAddress = addr;                 //擦除的起始地址
	f.NbPages = 1;                        //要擦除的页数 1~MAX之间的数值
	
	//设置PageError
	uint32_t PageError = 0;
	
	//调用擦除函数
	HAL_FLASHEx_Erase(&f, &PageError);
	
	//对Flash进行烧写
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, writeFlashData);
	Flash_Program((uint16_t *)flash_test_w, sizeof(flash_test_w));
	
	//上锁Flash
	HAL_FLASH_Lock();
}

static void Flash_Program(uint16_t *pData, uint16_t len)
{
//	static uint16_t*  pFlash_Last_Bank = (void *)0x08007C00;
	uint16_t i;
	
	for(i=0; i<len; i++)
	{
		// HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flash_test_addr+(i*2), pData[i]);
	}
}

void Flash_write(RC_Para_t *RC_Para)
{
	//解锁Flash
	HAL_FLASH_Unlock();
	
	//擦除Flash
	//初始化Flash结构体
	FLASH_EraseInitTypeDef f;
	
	f.TypeErase = FLASH_TYPEERASE_PAGES;  //擦除类型mass和page
	f.PageAddress = addr;                 //擦除的起始地址
	f.NbPages = 1;                        //要擦除的页数 1~MAX之间的数值
	
	//设置PageError
	uint32_t PageError = 0;
	
	//调用擦除函数
	HAL_FLASHEx_Erase(&f, &PageError);
	
	//对Flash进行烧写
	//HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, writeFlashData);
	Flash_Program((uint16_t *)RC_Para, sizeof(RC_Para_t));
	
	//上锁Flash
	HAL_FLASH_Lock();
}

////从Flash中Read
void Flash_read(uint16_t *buff, uint16_t len)
{
	uint16_t i;
	uint32_t readAddr;
	readAddr = addr;
	
	for (i = 0; i < len; i++)
	{
		buff[i] = ( *(uint32_t*)readAddr );
//		readAddr
	}
}

void printFlashTest(void)
{
	uint32_t temp = *(__IO uint32_t *)(addr);
	
	printf("addr:0x%x, data:0x%x\r\n",addr, temp);
}
