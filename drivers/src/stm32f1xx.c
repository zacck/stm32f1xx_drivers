/*
 * stm32f1xx.c
 *
 *  Created on: Mar 18, 2024
 *      Author: zaccko
 */

#include "stm32f1xx.h"


void delay(void) {
	for (uint32_t i = 0; i < 250000 / 2; i++)
		;
}


