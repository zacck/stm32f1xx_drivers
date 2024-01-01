/*
 * stm32f103xx_gpio.c
 *
 *  Created on: Nov 20, 2023
 *      Author: zaccko
 */

#include "stm32f103xx_gpio.h"

/******
 * @fn GPIO_INITL
 *
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pGPIOx] port handle structure
 *
 * @return void
 * @note
 *  */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {
	//pin port position
	uint8_t pin_port_pos;
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8) {
		pin_port_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	} else {
		pin_port_pos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber - 8;
	}

	uint32_t temp_reg_setting = 0;

	// set mode of pin
	temp_reg_setting = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 + (4 * pin_port_pos)));


	// set direction and speed of pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinDirection > GPIO_DIR_IN){
	temp_reg_setting = (pGPIOHandle->GPIO_PinConfig.GPIO_PinDirection
			<< (4 * pin_port_pos));
	}

	// use CRH or CRL when setting direction and speed
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8) {
		pGPIOHandle->pGPIOx->CRL |= temp_reg_setting;
	} else {
		pGPIOHandle->pGPIOx->CRH |= temp_reg_setting;
	}

	temp_reg_setting = 0;



}
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

}

/******
 * @fn GPIO_PCLK_CTRL
 *
 * @brief  Enables or Disables clock for a GPIO Port
 *
 * @params[pGPIOx] port base address
 * @params[EnorDi] Enable or Disable MAcros from header
 *
 * @return void
 * @note
 *  */
void GPIO_PCLK_CTRL(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
	} else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
	}

}
//Data Read and Write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	return 0;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx) {
	return 0;

}
/******
 * @fn GPIO_WriteToOutputPin
 *
 * @brief Writes a value to an output pin
 *
 * @params[pGPIOx] port base address
 * @params[PinNumber] Pin Number to write to
 * @params[value] value to write to the Pin
 * @return none
 * @note
 *  */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber,
		uint8_t Value) {

	//based on set and reset macros
	if(Value == GPIO_PIN_SET){
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}
/******
 * @fn GPIO_WriteToOutputPin
 *
 * @brief Writes a value to an output port
 *
 * @params[pGPIOx] port base address
 * @params[value] value to write to the Port
 * @return none
 * @note
 *  */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value) {
	pGPIOx->ODR = Value;

}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

}

//IRQ config and ISR handling
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {

}
void GPIO_IRQHandling(uint8_t PinNumber) {

}
