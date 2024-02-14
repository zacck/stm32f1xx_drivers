/*
 * stm32f103xx_spi.h
 *
 *  Created on: Feb 5, 2024
 *      Author: zaccko
 */

#ifndef INC_STM32F103XX_SPI_H_
#define INC_STM32F103XX_SPI_H_


// COnfigurable items for SPIx
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BUSConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
	uint8_t SPI_LSBFIRST;

}SPI_Config_t;


//Handle structure to actually use

typedef struct
{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
}SPI_Handle_t;


// Device Modes
#define SPI_DEVICE_MODE_MASTER	1;
#define SPI_DEVICE_MODE_SLAVE	0;

//Bus MODES
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3


// SPI CLOCK SPEED
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7


//SPI DFF
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS 	1

//SPI POLARITY
#define SPI_CPOL_HIGH	1
#define SPI_CPOL_LOW	0

//SPI PHASES
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0


//SPI SLAVE MANAGEMENT
#define SPI_SSM_EN		1
#define SPI_SSM_DI		0


// Init and DeInit APIs
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


//Clock Set Up
void SPI_PCLK_CTRL(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Send and Receive APIs
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);


//SPI Interrupt Configs
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

// SPI Flags
#define SPI_TXE_FLAG 	(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG 	(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG 	(1 << SPI_SR_BSY)
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

// MISc APIs
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);





#endif /* INC_STM32F103XX_SPI_H_ */
