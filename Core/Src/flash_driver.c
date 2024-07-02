/*
 * flash_driver.c
 *
 *  Created on: Jun 27, 2024
 *      Author: madison.vosburg
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "flash_driver.h"

/**
* writes button press counts to flash memory
*/
HAL_StatusTypeDef flash_write(uint32_t address, uint32_t data, bool timer_flag, WWDG_HandleTypeDef *hwwdg){
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef FLASH_EraseInitStruct = {0};
	FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	FLASH_EraseInitStruct.PageAddress = address;
	FLASH_EraseInitStruct.NbPages = 1;
	uint32_t PageError;
	HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &PageError);

	if(timer_flag){
		HAL_WWDG_Refresh(hwwdg);
	}

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, (uint64_t)data);
	HAL_FLASH_Lock();

	return HAL_OK;
}

/**
* reads button press counts from flash memory
*/
uint32_t flash_read(uint32_t address){
	return *(uint32_t*)address;
}
