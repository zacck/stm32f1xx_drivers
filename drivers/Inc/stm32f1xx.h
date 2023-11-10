/*
 * stm32f1xx.h
 *
 *  Created on: Nov 10, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_

#define FLASH_BASEADDR		0x08000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BASEADDR		0x1FFFF000U


// Bus domains of the controllers
#define PERIPH_BASE			0xA0000000U
#define APB1PERIPH_BASE		0x40007800U
#define APB2PERIPH_BASE		0x40015800U
#define AHBPERIPH_BASE		0xA0000000U




#endif /* INC_STM32F1XX_H_ */
