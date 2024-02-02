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
	uint8_t afio_exti_cr_index, afio_exti_cr_pos;

	// set mode of pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_IN_PUPD) {

		temp_reg_setting = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
				<< (2 + (4 * pin_port_pos)));
	} else {
		// Enable clock for altfunction IO
		AFIO_PCLK_EN();
		// enable the interrupt for pin and port
		afio_exti_cr_index = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
		afio_exti_cr_pos = ((pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4) * 4);

		if (pGPIOHandle->pGPIOx == GPIOA) {
			AFIO->EXTICR[afio_exti_cr_index] |= (0 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOB) {
			AFIO->EXTICR[afio_exti_cr_index] |= (1 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOC) {
			AFIO->EXTICR[afio_exti_cr_index] |= (2 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOD) {
			AFIO->EXTICR[afio_exti_cr_index] |= (3 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOE) {
			AFIO->EXTICR[afio_exti_cr_index] |= (4 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOF) {
			AFIO->EXTICR[afio_exti_cr_index] |= (5 << afio_exti_cr_pos);

		} else if (pGPIOHandle->pGPIOx == GPIOB) {
			AFIO->EXTICR[afio_exti_cr_index] |= (6 << afio_exti_cr_pos);

		}
		//1. Set pin to input mode
		temp_reg_setting = (GPIO_MODE_IN_FP << (2 + (4 * pin_port_pos)));
		//4.handle rising and falling interrupt
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT) {
			//handle falling intterrupt
			//set FTSR and reset RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT) {
			//handle rising interrupt
			//set RTSR and reset FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		} else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT) {
			// Handle Interrupt modes
			//set RTSR and FTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Handle MCU specific config for interrupts

		//Unmask Interrupt using IMR register
		EXTI->IMR |= (1 <<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	// Handle Alternate function mode, we will need this for interrupts
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
		//TODO


	}


	// set direction and speed of pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinDirection > GPIO_DIR_IN) {
		temp_reg_setting |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinDirection
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
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}
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
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		pGPIOx->ODR &= ~(1 << PinNumber);

	}
}
/******
 * @fn GPIO_WriteToOutputPort
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

/******
 * @fn GPIO_ToggleOutputPin
 *
 * @brief Flips the value at a given output pin
 *
 * @params[pGPIOx] port base address
 * @params[PinNumber] Pin Number to Flip
 *
 * @return none
 * @note
 *  */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {
	//use XOR to toggle bitfield
	pGPIOx->ODR = pGPIOx->ODR ^ (1 << PinNumber);

}

//IRQ config and ISR handling
/******
 * @fn GPIO_IRQConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[EnorDi] Enable or disable
 *
 * @return none
 * @note
 *  */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		// we oonly handle the first 3 registers as our MCU
		// Can only handle 88 interrupts
		if (IRQNumber <= 31) {
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}

	} else {
		if (IRQNumber <= 31) {
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}

	}
}
/******
 * @fn GPIO_IRQPriorityConfig
 *
 * @brief Flips the value at a given output pin
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[IRQPriority] Priority of the given IRQ
 *
 * @return none
 * @note
 *  */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority) {

	//find out the IPR register to use
	uint8_t iprx = IRQNumber / 4;

	//section of register
	uint8_t iprx_section = IRQNumber % 4;

	//this is due to the lower nibble not being implemented in iPR regusters
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber) {
	// clear the pending ISR register to avoid looping
	if (EXTI->PR & (1 << PinNumber)) {
		EXTI->PR |= (1 << PinNumber);
	}
}
