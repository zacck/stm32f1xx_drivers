/*
 * stm32f103xx_spi.c
 *
 *  Created on: Feb 5, 2024
 *      Author: zaccko
 */

#include "stm32f103xx_gpio.h"

/***
 * @fn SPI_PCLK_CTRL
 *
 * @brief Enables or Diables Clock for a SPI port
 *
 * @params[pSPIx]
 * @params[EnorDi]
 *
 *
 * @return void
 */
void SPI_PCLK_CTRL(SPI_RegDef_t *pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
	} else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		} else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		} else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
	}
}


/***
 * @fn SPI_Init
 *
 * @brief Initializes the specific SPI peripheral you want to use
 *
 * @params[pSPIHandle] peripheral handle structure
 *
 * @return void
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint32_t temp_cr1_reg = 0;

	//device mode
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//bus config
	if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_FD){
		// clear bidi mode
		temp_cr1_reg &= ~(1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_HD){
		//set bidi mode
		temp_cr1_reg |= (1 << SPI_CR1_BIDIMODE);
	} else if(pSPIHandle->SPIConfig.SPI_BUSConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		//clear bidi mode
		temp_cr1_reg &= ~(1 << SPI_CR1_BIDIMODE);

		// set rx only
		temp_cr1_reg |= (1 << SPI_CR1_RXONLY);

	}

	// configure speed using baud rate
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// configure frame size
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//configure CPOL
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//configure CPHA
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// configure SSM
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//configure LSBFIRST
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_LSBFIRST << SPI_CR1_LSBFIRST;

	pSPIHandle->pSPIx->CR1 = temp_cr1_reg;

}


/***
 * @fn SPI_DeInit
 *
 * @brief DeInitializes the specific SPI peripheral you have used
 *
 * @params[pSPIx] peripheral address
 *
 * @return void
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	} else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	} else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
}


















