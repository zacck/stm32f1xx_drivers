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
#define APB1PERIPH_BASE		0x40000000U
#define APB2PERIPH_BASE		0x40010000U
#define AHBPERIPH_BASE		0x40018000U



//APB2  port addresses
#define GPIOA_BASEADDR		(APB2PERIPH_BASE + 0800)
#define GPIOB_BASEADDR		(APB2PERIPH_BASE + 0C00)
#define GPIOC_BASEADDR		(APB2PERIPH_BASE + 1000)
#define GPIOD_BASEADDR		(APB2PERIPH_BASE + 1400)
#define GPIOE_BASEADDR		(APB2PERIPH_BASE + 1800)
#define GPIOF_BASEADDR		(APB2PERIPH_BASE + 1C00)
#define GPIOG_BASEADDR		(APB2PERIPH_BASE + 2000)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 3800)
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0400)



//APB1 Peripherals
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 5800)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 5000)






#endif /* INC_STM32F1XX_H_ */
