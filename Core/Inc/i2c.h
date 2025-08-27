/*
 * i2c.h
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32PeripheralAddr.h"
#include "gpioWriteRead.h"

#define SR2_BUSY_Pos		1u
#define SR1_SB_Pos 			0u
#define SR1_ADDR_Pos		1u
#define SR1_BTF_Pos			2u
#define	SR1_TXE_Pos			7u
#define	SR1_AF_Pos			10u
#define	CR1_START_Pos		8u
#define	CR1_STOP_Pos		9u

typedef enum{
	I2C_CR1,
	I2C_CR2,
	I2C_OAR1,
	I2C_OAR2,
	I2C_DR,
	I2C_SR1,
	I2C_SR2,
	I2C_CCR,
	I2C_TRISE,
	I2C_FLTR,

	I2C_REG_COUNT
}I2C_Reg_t;

typedef enum{
	_I2C1,
	_I2C2,
	_I2C3,

	I2C_count
}I2C_Name_t;

typedef enum{
	I2C_SM_MODE,
	I2C_FM_MODE_2LOW_1HIGH,
	I2C_FM_MODE_16LOW_9HIGH
}I2C_CCR_Mode_t;

typedef struct{
	I2C_Name_t i2cBus;

	GPIO_Pin_t sclPin;
	GPIO_PortName_t sclPort;

	GPIO_Pin_t sdaPin;
	GPIO_PortName_t sdaPort;
}I2C_GPIO_Config_t;

#endif /* INC_I2C_H_ */
