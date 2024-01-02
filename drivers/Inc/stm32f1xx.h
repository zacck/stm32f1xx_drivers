/*
 * stm32f1xx.h
 *
 *  Created on: Nov 10, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_

#include <stdint.h>

#define __vo volatile

#define FLASH_BASEADDR		0x08000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BASEADDR		0x1FFFF000U


// Bus domains of the controllers
#define PERIPH_BASE			0xA0000000U
#define APB1PERIPH_BASE		0x40000000U
#define APB2PERIPH_BASE		0x40010000U
#define AHBPERIPH_BASE		0x40018000U



//APB2  port addresses
#define GPIOA_BASEADDR		(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR		(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR		(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR		(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR		(APB2PERIPH_BASE + 0x2000)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x0400)



//APB1 Peripherals
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)

//AHB Peripherals
#define RCC_BASEADDR 		(AHBPERIPH_BASE + 0x9000)

//Peripheral Register structs


//GPIO
typedef struct
{
	__vo uint32_t CRL;
	__vo uint32_t CRH;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t BRR;
	__vo uint32_t LCKR;

}GPIO_RegDef_t;


//RCC
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
}RCC_RegDef_t;


//Peripheral Definitions


//AHB
#define RCC ((RCC_RegDef_t *) RCC_BASEADDR)

//APB2  port addresses
#define GPIOA ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t * )GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t * )GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t * )GPIOC_BASEADDR)
#define GPIOE ((GPIO_RegDef_t * )GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t * )GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t * )GPIOG_BASEADDR)

//Clock Enable and Disable
#define GPIOA_PCLK_EN()		(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()		(RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()		(RCC->APB2ENR |= (1 << 8))


#define GPIOA_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 8))


// GPIO REGISTER RESET MACROS
#define GPIOA_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while (0)
#define GPIOB_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while (0)
#define GPIOC_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while (0)
#define GPIOD_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while (0)
#define GPIOE_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while (0)
#define GPIOF_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7));} while (0)
#define GPIOG_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8));} while (0)



// Conviniences
#define ENABLE 1
#define DISABLE  0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET


#include "stm32f103xx_gpio.h"



#endif /* INC_STM32F1XX_H_ */
