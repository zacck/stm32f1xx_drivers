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
	//Enable Peripheral clock for SPI peripheral
	SPI_PCLK_CTRL(pSPIHandle->pSPIx, ENABLE);

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

	//configure DFF 
	temp_cr1_reg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

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

/***
 * @fn SPI_SendData
 *
 *
 * @brief Blocking API for sending data over SPI, this waits until transfer is complete
 *
 * @params[pSPIx] perioheral to use
 * @params[pTxBuffer] Outbound Data Buffer
 * @params[Len] Length of the data we want to send
 *
 * @return void
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len){
	while(Len > 0) {
		//wait for TXE to be set
		while (SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) ==(uint8_t)FLAG_RESET);


		//check DFF
		if((pSPIx->CR1 && (1  << SPI_CR1_DFF))){
			//16bit DFF
			pSPIx->DR =* ((uint16_t*)pTxBuffer);
			//Decrease len twice since we picked a half word(2 bytes)
			Len--;
			Len--;
			//increment data pointer location
			(uint16_t*)pTxBuffer++;
		} else {
			// 8 bit frame format
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}

/****
 * Blocking API to receive data via SPI
 * @fn SPI_ReceiveData
 *
 * @params[pSPIx] the base address of the peripheral we are using
 * @params[pRxBuffer] the buffer to hold the data we receive
 * @params[Len] the number of bytes of data that we intend to receive
 *
 *
 * @return void
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len){
	while(Len > 0){
		//wait for RXNE
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		//check DFF
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			//16 bit
			*((uint16_t *) pRxBuffer) = pSPIx->DR;
			//dec lenght
			Len--;
			Len--;
			(uint16_t*) pRxBuffer++;
		} else {
			// 8 bit DFF
			*(pRxBuffer) = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}


/***
 * @fn SPI_GetFlagStatus
 *
 * @params[pSPIx] the address of the peripheral in use
 * @params[FlagName] the name of the flag whose status we want
 *
 * @return 8 bit integer
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName){
	if(pSPIx->SR & FlagName){
		return FLAG_SET;
	}

	return FLAG_RESET;
}


/***
 * @fn SPI_Peripheral control
 *
 * @params[pSPIx] the address of the peripheral in use
 * @params[EnorDi] enable or disable param
 * @return void
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	} else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***
 * @fn SPI_SSIConfig
 *
 * @params[pSPIx] the address of the peripheral in use
 * @params[EnorDi] enable or disable param
 * @return void
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if(EnorDi == ENABLE){
			pSPIx->CR1 |= (1 << SPI_CR1_SSI);
		} else {
			pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
		}
}


















