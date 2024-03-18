/*
 * stm32f1xx.h
 *
 *  Created on: Nov 10, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F1XX_H_
#define INC_STM32F1XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo volatile
#define __weak __attribute((weak))


/**************** Processor specific details *********/
//Arm Cortex MX processor NVIC ISERx register addresses
//Set/Enable Interrupts

#define NVIC_ISER0			((__vo uint32_t *)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t *)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t *)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t *)0xE000E10C)

//Arm Cortex MX processor NVIC ICERx register addresses
//Clear/Disable interrupts
#define NVIC_ICER0			((__vo uint32_t *)0XE000E180)
#define NVIC_ICER1			((__vo uint32_t *)0XE000E184)
#define NVIC_ICER2			((__vo uint32_t *)0XE000E188)
#define NVIC_ICER3			((__vo uint32_t *)0XE000E18C)

// NVIC Priority base address
#define NVIC_PR_BASEADDR	((__vo uint32_t *)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4






/****************** End Processor ********************/

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
#define AFIO_BASEADDR		(APB2PERIPH_BASE + 0x0000)



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


//EXTI
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;


//AFIO
typedef struct
{
	__vo  uint32_t EVCR;
	__vo  uint32_t MAPR;
	__vo  uint32_t EXTICR[4];
}AFIO_RegDef_t;


// SPI
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


// I2C
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
}I2C_RegDef_t;


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
#define EXTI  ((EXTI_RegDef_t *	)EXTI_BASEADDR)
#define AFIO  ((AFIO_RegDef_t * )AFIO_BASEADDR)
#define SPI1  ((SPI_RegDef_t *) SPI1_BASEADDR)


//APB1 port addresses
#define SPI2  ((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3  ((SPI_RegDef_t *) SPI3_BASEADDR)
#define I2C1  ((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2  ((I2C_RegDef_t *)	I2C2_BASEADDR)




//Clock Enable and Disable
//Enable
#define AFIO_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))
#define GPIOA_PCLK_EN()		(RCC->APB2ENR |= (1 << 2))
#define GPIOB_PCLK_EN()		(RCC->APB2ENR |= (1 << 3))
#define GPIOC_PCLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define GPIOD_PCLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define GPIOE_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))
#define GPIOF_PCLK_EN()		(RCC->APB2ENR |= (1 << 7))
#define GPIOG_PCLK_EN()		(RCC->APB2ENR |= (1 << 8))
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))




//Disable
#define AFIO_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0))
#define GPIOA_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 2))
#define GPIOB_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 3))
#define GPIOC_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 4))
#define GPIOD_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 5))
#define GPIOE_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 6))
#define GPIOF_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 7))
#define GPIOG_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 8))
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))



// GPIO REGISTER RESET MACROS
#define GPIOA_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 2)); (RCC->APB2RSTR &= ~(1 << 2));} while (0)
#define GPIOB_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 3)); (RCC->APB2RSTR &= ~(1 << 3));} while (0)
#define GPIOC_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while (0)
#define GPIOD_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while (0)
#define GPIOE_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while (0)
#define GPIOF_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7));} while (0)
#define GPIOG_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 8)); (RCC->APB2RSTR &= ~(1 << 8));} while (0)


// SPI REGISTER RESET MACROS
#define SPI1_REG_RESET() 	do {(RCC->APB2RSTR |=  (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while (0)
#define SPI2_REG_RESET()	do {(RCC->APB1RSTR |=  (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while (0)
#define SPI3_REG_RESET()	do {(RCC->APB1RSTR |=  (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while (0)


// I2C Register Reset
#define I2C1_REG_RESET()	do {(RCC->APB1RSTR |=  (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while (0)
#define I2C2_REG_RESET()	do {(RCC->APB1RSTR |=  (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while (0)



// IRQ Numbers
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI15_10		40
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34




//IRQ Priorities
#define NVIC_IRQ_PRIO0		0
#define NVIC_IRQ_PRIO1		1
#define NVIC_IRQ_PRIO2		2
#define NVIC_IRQ_PRIO3		3
#define NVIC_IRQ_PRIO4		4
#define NVIC_IRQ_PRIO5		5
#define NVIC_IRQ_PRIO6		6
#define NVIC_IRQ_PRIO7		7
#define NVIC_IRQ_PRIO8		8
#define NVIC_IRQ_PRIO9		9
#define NVIC_IRQ_PRIO10		10
#define NVIC_IRQ_PRIO11		11
#define NVIC_IRQ_PRIO12		12
#define NVIC_IRQ_PRIO13		13
#define NVIC_IRQ_PRIO14		14
#define NVIC_IRQ_PRIO15		15


// Conviniences
#define ENABLE 1
#define DISABLE  0
#define SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET
#define FLAG_SET SET
#define FLAG_RESET RESET

//Bit Positions
// SPI CR1 BIT POSITIONS
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR	 		2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE 		15

//SPI CR2 Bits
#define SPI_CR2_RXDMAEN			0
#define SPI_CR2_TXDMAEN			1
#define SPI_CR2_SSOE			2
#define SPI_CR2_RES				3
#define SPI_CR2_FRF				4
#define SPI_CR2_ERRIE			5
#define SPI_CR2_RXNEIE			6
#define SPI_CR2_TXEIE			7

//SPI SR Bits
#define SPI_SR_RXNE				0
#define SPI_SR_TXE				1
#define SPI_SR_CHSIDE			2
#define SPI_SR_UDR				3
#define SPI_SR_CRCERR			4
#define SPI_SR_MODF				5
#define SPI_SR_OVR				6
#define SPI_SR_BSY				7


/************* END SPI BITS **************/


// I2C BIT Definitions
//CR1  bits
#define I2C_CR1_PE				0
#define I2C_CR1_NOSTRETCH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_SWRST			15


//CR2 bits
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERREN			8
#define I2C_CR2_ITEVEN			9
#define I2C_CR2_ITBUFEN			10

//SR1
#define I2C_SR1_SB						0
#define I2C_SR1_ADDR					1
#define I2C_SR1_BTF						2
#define I2C_SR1_ADD10					3
#define I2C_SR1_STOPF					4
#define I2C_SR1_RXNE					6
#define I2C_SR1_TXE						7
#define I2C_SR1_BERR					8
#define I2C_SR1_ARLO					9
#define I2C_SR1_AF						10
#define I2C_SR1_OVR						11
#define I2C_SR1_TIMEOUT					14


//SR2
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY					1
#define I2C_SR2_TRA						2
#define I2C_SR2_GENCALL					4
#define I2C_SR2_DUALF					7


/***** END I2C Bits *******************/


#include "stm32f103xx_gpio.h"
#include "stm32f103xx_spi.h"
#include "stm32f103xx_i2c.h"
#include "ssd1306.h"



#endif /* INC_STM32F1XX_H_ */
