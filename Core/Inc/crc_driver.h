/*
 * crc_driver.h
 *
 *  Created on: Jun 27, 2024
 *      Author: madison.vosburg
 */

#ifndef INC_CRC_DRIVER_H_
#define INC_CRC_DRIVER_H_

#include "stm32f3xx_hal.h"

uint64_t crc_xor(uint64_t div_data);

uint64_t crc_division(uint64_t data, int curs_pos, int shift_pos, uint64_t answer);

#endif /* INC_CRC_DRIVER_H_ */
