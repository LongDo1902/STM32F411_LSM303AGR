///*
// * i2c.h
// *
// *  Created on: Aug 20, 2025
// *      Author: dobao
// */
//
//#ifndef INC_I2C_H_
//#define INC_I2C_H_
//
//#include <stdio.h>
//#include <stdbool.h>
//#include <stdint.h>
//
//#include "stm32f4xx_hal.h"
//#include "stm32PeripheralAddr.h"
//#include "gpioWriteRead.h"
//#include "timer.h"
//
//#define SR1_SB_Pos 			0u
//#define SR1_ADDR_Pos		1u
//#define SR1_BTF_Pos			2u
//#define SR1_RXNE_Pos		6u
//#define	SR1_TXE_Pos			7u
//#define	SR1_AF_Pos			10u
//
//#define SR2_BUSY_Pos		1u
//
//#define	CR1_START_Pos		8u
//#define	CR1_STOP_Pos		9u
//#define CR1_ACK_Pos			10u
//
//typedef enum{
//	I2C_OK = 0,
//	I2C_ERR_BUSY_TIMEOUT,
//	I2C_ERR_ADDR_NACK,
//	I2C_ERR_SUBADDR_TIMEOUT,
//	I2C_ERR_DATA_NACK,
//	I2C_ERR_READ_ADDR_NACK,
//	I2C_ERR_RXNE_TIMEOUT,
//	I2C_ERR_UNKNOWN,
//	I2C_WRITE_FAIL
//}I2C_Status_t;
//
//typedef enum{
//	I2C_CR1,
//	I2C_CR2,
//	I2C_OAR1,
//	I2C_OAR2,
//	I2C_DR,
//	I2C_SR1,
//	I2C_SR2,
//	I2C_CCR,
//	I2C_TRISE,
//	I2C_FLTR,
//
//	I2C_REG_COUNT
//}I2C_Reg_t;
//
//typedef enum{
//	_I2C1,
//	_I2C2,
//	_I2C3,
//
//	I2C_count
//}I2C_Name_t;
//
//typedef enum{
//	I2C_SM_MODE,
//	I2C_FM_MODE_2LOW_1HIGH,
//	I2C_FM_MODE_16LOW_9HIGH
//}I2C_CCR_Mode_t;
//
//typedef struct{
//	I2C_Name_t i2cBus;
//
//	GPIO_Pin_t sclPin;
//	GPIO_PortName_t sclPort;
//
//	GPIO_Pin_t sdaPin;
//	GPIO_PortName_t sdaPort;
//}I2C_GPIO_Config_t;
//
//
///*
// * =======================================
// * FUNCTION DECLARATION
// * =======================================
// */
//bool writeI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t value);
//uint32_t readI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition);
//bool I2C_sclPinInit(GPIO_Pin_t sclPin, GPIO_PortName_t sclPort, I2C_Name_t i2cBus);
//bool I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus);
//bool I2C_GPIO_init(I2C_GPIO_Config_t config);
//bool I2C_getCCR(I2C_CCR_Mode_t ccrMode, uint32_t sclFreq, uint32_t fclk1, I2C_GPIO_Config_t config);
//bool I2C_writeReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t value);
//bool I2C_readReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t* outResult);
//void I2C_basicConfigInit(I2C_GPIO_Config_t config, I2C_CCR_Mode_t ccrMode, uint32_t sclFreq, uint32_t fclk1_Hz);
//#endif /* INC_I2C_H_ */
/*
 * i2c.h
 *
 *  Created on: Jul 4, 2025
 *      Author: dobao
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "gpioWriteRead.h"
#include "stm32PeripheralAddr.h"
#include "rcc.h"


#define GET_I2C1_REG(mode) (&(I2C1_REG -> mode))
#define GET_I2C2_REG(mode) (&(I2C2_REG -> mode))
#define GET_I2C3_REG(mode) (&(I2C3_REG -> mode))

/*
 * -----------------------------------------
 * Enumeration
 * -----------------------------------------
 */
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
}I2C_Mode_t;

typedef enum{
	my_I2C1,
	my_I2C2,
	my_I2C3,

	my_I2C_COUNT
}I2C_Name_t;

typedef enum{
	I2C_SM_100K,
	I2C_FM_400K_DUTY_2LOW_1HIGH,
	I2C_FM_400K_DUTY_16LOW_9HIGH,
}I2C_CCR_Mode_t;

typedef enum{
	I2C_OK = 0,
	I2C_ERROR,
	I2C_INVALID_PIN,
	I2C_INVALID_BUS,
	I2C_TIMEOUT,
	I2C_NACK,
	I2C_ACK,
	I2C_BUSY
}I2C_Status_t;

typedef struct{
	I2C_Name_t i2cBus;

	GPIO_Pin_t sclPin;
	GPIO_PortName_t sclPort;

	GPIO_Pin_t sdaPin;
	GPIO_PortName_t sdaPort;
}I2C_GPIO_Config_t;

/*
 * ---------------------------------------------------------
 * Function Declarations
 * ---------------------------------------------------------
 */
void writeI2C(uint8_t bitPosition, I2C_Name_t i2cBus, I2C_Mode_t mode, uint32_t value);
uint32_t readI2C(uint8_t bitPosition, I2C_Name_t i2cBus, I2C_Mode_t mode);
void I2C_basicConfigInit(I2C_GPIO_Config_t config,
						 I2C_CCR_Mode_t ccrMode,
						 uint32_t sclFreq,
						 uint32_t sysClkFreq);
I2C_Status_t I2C_singleByteRead(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr);
I2C_Status_t I2C_singleByteWrite(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t value);

#endif /* INC_I2C_H_ */
