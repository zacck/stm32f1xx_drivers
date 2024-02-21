/*
 * stm32f103xx_spi.c
 *
 *  Created on: Feb 5, 2024
 *      Author: zaccko
 */

#include "stm32f1xx.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);


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


/***
 * @fn SPI_SendDataIT
 *
 *
 * @brief Interrupt API for sending data over SPI.
 *
 * @params[pSPIHandle] perioheral and state to use
 * @params[pTxBuffer] Outbound Data Buffer
 * @params[Len] Length of the data we want to send
 *
 * @return uint8_t
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t  Len){

	// check that SPI is not busy
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX) {
		// hold on to the data to send
		pSPIHandle->TxLen = Len;
		pSPIHandle->pTxBuffer = pTxBuffer;

		// mark spi as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// Enable TXEIE
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		// transmission now handled in ISR

	}

	return state;

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
 * @fn SPI_ReceiveDataIT
 *
 *
 * @brief Interrupt API for receiving data over SPI.
 *
 * @params[pSPIHandle] perioheral and state to use
 * @params[pRxBuffer] Inbound Data Buffer
 * @params[Len] Length of the data we want to rx
 *
 * @return uint8_t
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t  Len){
	// check that SPI is not busy
		uint8_t state = pSPIHandle->RxState;

		if(state != SPI_BUSY_IN_RX) {
			// hold on to the data to send
			pSPIHandle->RxLen = Len;
			pSPIHandle->pRxBuffer = pRxBuffer;

			// mark spi as busy
			pSPIHandle->RxState = SPI_BUSY_IN_RX;

			// Enable TXEIE
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

			// reception now handled in ISR

		}

		return state;
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

//IRQ config and ISR handling
/******
 * @fn SPI_IRQConfig
 *
 * @brief Flips the value at a given IRQ number mask
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[EnorDi] Enable or disable
 *
 * @return none
 * @note
 *  */
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		// we only handle the first 3 registers as our MCU
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
 * @fn SPI_IRQPriorityConfig
 *
 * @brief Flips the value at a given IRQ number
 *
 * @params[IRQNumber] IRQ number to configure
 * @params[IRQPriority] Priority of the given IRQ
 *
 * @return none
 * @note
 *  */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{

	//find out the IPR register to use
	uint8_t iprx = IRQNumber / 4;

	//section of register
	uint8_t iprx_section = IRQNumber % 4;


	//this is due to the lower nibble not being implemented in iPR regusters
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);


	*(NVIC_PR_BASEADDR + iprx) |= (IRQPriority << shift_amount);


}




void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	// TX interrupt
	uint8_t temp1, temp2;
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2){
		// we have a tx interrupt so send
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// RX interrupt
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// we have a tx interrupt so send
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// handle errors
	// check the OVR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) {
		// we have a tx interrupt so send
		spi_ovr_interrupt_handle(pSPIHandle);
	}

}

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIHandle){
	uint8_t temp;
	temp = pSPIHandle->pSPIx->DR;
	temp = pSPIHandle->pSPIx->SR;
	(void) temp;
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){
	//gracefully finish tx and reset to sane state
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_TX_CMPLT);
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){
	//gracefull finish RX
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_RX_CMPLT);
}

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//check DFF
	if ((pSPIHandle->pSPIx->CR1 && (1 << SPI_CR1_DFF))) {
		//16bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		//Decrease len twice since we picked a half word(2 bytes)
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		//increment data pointer location
		(uint16_t*) pSPIHandle->pTxBuffer++;
	} else {
		// 8 bit frame format
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLen) {
		SPI_CloseTransmission(pSPIHandle);
	}
}
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle) {
	//check DFF
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		//16 bit
		*((uint16_t*) pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		//dec lenght
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
		(uint16_t*) pSPIHandle->pRxBuffer++;
	} else {
		// 8 bit DFF
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(!pSPIHandle->RxLen) {
		SPI_CloseReception(pSPIHandle);
	}
}
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle){

	uint8_t temp;
	//clear ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp;
	//Application Callback
	SPI_ApplicationEventCallBack(pSPIHandle, SPI_EVENT_OVR_ERR);

}


__weak void SPI_ApplicationEventCallBack(SPI_Handle_t *pSPIHandle, uint8_t APPEv){
	// weak implementation for application to override
}






















