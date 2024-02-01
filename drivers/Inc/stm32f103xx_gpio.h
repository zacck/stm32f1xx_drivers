/*
 * stm32f103xx_gpio.h
 *
 *  Created on: Nov 19, 2023
 *      Author: zaccko
 */

#ifndef INC_STM32F103XX_GPIO_H_
#define INC_STM32F103XX_GPIO_H_

#include "stm32f1xx.h"

typedef struct
{
	uint8_t GPIO_PinDirection;
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOpType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;


/* GPIO APIs */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOHandle);

//clock set up for GPIO
void GPIO_PCLK_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Data Read and Write
//Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ config and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


// MACROS
// GPIO INPUT TYPES MACROS
#define GPIO_MODE_IN_AN 	0
#define GPIO_MODE_IN_FP 	1
#define GPIO_MODE_IN_PUPD	2
#define GPIO_MODE_ALTFN		3
#define GPIO_MODE_IT_FT  	4
#define GPIO_MODE_IT_RT  	5
#define GPIO_MODE_IT_RFT  	6

// GPIO OUTPUT TYPES MACROS
#define GPIO_MODE_OUT_PP 	0
#define GPIO_MODE_OUT_OD 	1

//DIRECTION & SPEED MACROS
#define GPIO_DIR_IN				0
#define GPIO_DIR_OUT_LOW 		1
#define GPIO_DIR_OUT_MEDIUM 	2
#define GPIO_DIR_OUT_FAST 		3




//pin Number macros
#define GPIO_PIN_NO_0 0
#define GPIO_PIN_NO_1 1
#define GPIO_PIN_NO_2 2
#define GPIO_PIN_NO_3 3
#define GPIO_PIN_NO_4 4
#define GPIO_PIN_NO_5 5
#define GPIO_PIN_NO_6 6
#define GPIO_PIN_NO_7 7
#define GPIO_PIN_NO_8 8
#define GPIO_PIN_NO_9 9
#define GPIO_PIN_NO_10 10
#define GPIO_PIN_NO_11 11
#define GPIO_PIN_NO_12 12
#define GPIO_PIN_NO_13 13
#define GPIO_PIN_NO_14 14
#define GPIO_PIN_NO_15 15


#endif /* INC_STM32F103XX_GPIO_H_ */
