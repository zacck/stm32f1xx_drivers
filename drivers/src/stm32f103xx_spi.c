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

