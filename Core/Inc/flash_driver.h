/*
 * flash_driver.h
 *
 *  Created on: Jun 27, 2024
 *      Author: madison.vosburg
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

#include "stm32f3xx_hal.h"
#include <stdbool.h>

HAL_StatusTypeDef flash_write(uint32_t address, uint32_t data, bool timer_flag, WWDG_HandleTypeDef *hwwdg);

uint32_t flash_read(uint32_t address);

#endif /* INC_FLASH_DRIVER_H_ */
