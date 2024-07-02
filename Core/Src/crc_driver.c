/*
 * crc_driver.c
 *
 *  Created on: Jun 27, 2024
 *      Author: madison.vosburg
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "crc_driver.h"

uint64_t crc_key = 0xD;

/**
 * XOR logic used to divide data by key
 */
uint64_t crc_xor(uint64_t div_data){
	uint64_t ans = div_data;
	if(ans & 0b1000){
		ans = ans ^ crc_key;		//if leftmost bit is 1, perform xor with key
	}else{
		ans = ans ^ 0b0000;			//if leftmost bit is 0, perform xor with all zeros
	}
	return ans;
}
/**
 * divides data by key to get remainder
 *
 * takes 4 bits at a time and XORs them until 4 bit remainder is left
 */
uint64_t crc_division(uint64_t data, int curs_pos, int shift_pos, uint64_t answer){
	int cursor = curs_pos;
	int bit_shift = shift_pos;
	uint64_t remain = answer;
	uint64_t dividend = 0;

	while(bit_shift > 0){
		bit_shift--;
		dividend = data & (0x0800000000000000 >> cursor);
		dividend = dividend >> bit_shift;
		remain = remain << 1;
		remain += dividend;
		remain = crc_xor(remain);
		cursor++;
	}
	return remain;
}
