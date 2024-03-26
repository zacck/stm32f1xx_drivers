/*
 * stm32f103xx_i2c.h
 *
 *  Created on: Feb 21, 2024
 *      Author: zaccko
 */

#ifndef INC_STM32F103XX_I2C_H_
#define INC_STM32F103XX_I2C_H_

#include "stm32f1xx.h"


//config  type and struct
typedef struct {
	uint32_t  I2C_SCLSpeed;
	uint8_t   I2C_DeviceAddress;
	uint8_t   I2C_AckControl;
	uint16_t  I2C_FMDutyCyle;
}I2C_Config_t;

//Handle struct
typedef struct
{
	I2C_RegDef_t *pI2Cx;     //which peripheral
	I2C_Config_t I2C_Config; // how to use this peripheral
}I2C_Handle_t;

// Speeds
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCK_SPEED_FM 400000


// Ack Control
#define I2C_ACK_EN 	1
#define I2C_ACK_DI  0

// FM Duty Cycle
#define I2C_FM_DUTY_2 		0
#define I2C_FM_DUTY_16_9	1


// APIs
//Init  & DeInit
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


//clock enable
void I2C_PCLK_CTRL(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

// Data TX & RX
// master tx & Rx
// slave tx & rx


//IRQ Config
void I2C_IRQConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);


// Other Peipheral control APIs
void I2C_PeripheralControl(I2C_RegDef_t *pI2cx, uint8_t EnorDi);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


//Application callback
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);


#endif /* INC_STM32F103XX_I2C_H_ */
