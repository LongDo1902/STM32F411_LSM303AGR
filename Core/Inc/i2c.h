/*
 * @file	i2c.h
 * @brief	Low-level I2C register helpers and GPIO mapping for STM32F411
 *
 * Created: Aug 20, 2025
 * Author: dobao
 *
 * This header declares:
 * 		Field-safe register R/W helpers with reserved-bit protection
 * 		GPIO initialization for legal SCL/SDA mappings (with correct pull-up handling)
 * 		Timing helpers for CCR/TRISE based on SM/FM modes
 * 		Simple 8-bit sub-register read/write helpers for 7-bit I2C slaves
 *
 * All names, values, and function signature are preserved for compatibility
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "stm32PeripheralAddr.h"
#include "gpioWriteRead.h"
#include "timer.h"


/* ========================================================================== */
/*                          Public Constants & Timeouts                        */
/* ========================================================================== */
/* Error sentinel returned by read helpers on failure */
#define I2C_VALUE_ERROR		0xFFFFFFFFu

/* Busy-wait timeouts (microseconds) */
#define I2C_TIMEOUT_LONG_US		100000u
#define I2C_TIMEOUT_NORMAL_US	10000u
#define I2C_TIMEOUT_SHORT_US	1000u

/*
 * ===========================================================
 * 					I2C_CR1 BIT POSITION
 * ===========================================================
 */
#define CR1_PE_Pos			0u
#define CR1_SMBUS_Pos		1u
#define CR1_SMBTYPE_Pos		3u
#define CR1_ENARP_Pos		4u
#define CR1_ENPEC_Pos		5u
#define CR1_ENGC_Pos		6u
#define CR1_NOSTRETCH_Pos	7u
#define CR1_START_Pos		8u
#define	CR1_STOP_Pos		9u
#define CR1_ACK_Pos			10u
#define CR1_POS_Pos			11u
#define CR1_PEC_Pos			12u
#define CR1_ALERT_Pos		13u
#define CR1_SWRST_Pos		15u

/*
 * ===========================================================
 * 					I2C_CR2 BIT POSITION
 * ===========================================================
 */
#define CR2_FREQ_Pos		0u
#define CR2_ITERREN_Pos		8u
#define CR2_ITEVTEN_Pos		9u
#define CR2_ITBUFEN_Pos		10u
#define CR2_DMAEN_Pos		11u
#define CR2_LAST_Pos		12u

/*
 * ===========================================================
 * 					I2C_OAR1 BIT POSITION
 * ===========================================================
 */
#define OAR1_ADD0_Pos		0u
#define OAR1_ADDLOW_Pos		1u
#define OAR1_ADDHIGH_Pos 	8u
#define OAR1_ADDMODE_Pos	15u

/*
 * ===========================================================
 * 					I2C_OAR2 BIT POSITION
 * ===========================================================
 */
#define OAR2_ENDUAL_Pos		0u
#define OAR2_ADD2_Pos		1u

/*
 * ===========================================================
 * 					I2C_SR1 BIT POSITION
 * ===========================================================
 */
#define SR1_SB_Pos 			0u
#define SR1_ADDR_Pos		1u
#define SR1_BTF_Pos			2u
#define SR1_ADD10_Pos		3u
#define SR1_STOPF_Pos		4u
#define SR1_RXNE_Pos		6u
#define	SR1_TXE_Pos			7u
#define SR1_BERR_Pos		8u
#define SR1_ARLO_Pos		9u
#define	SR1_AF_Pos			10u
#define SR1_OVR_Pos			11u
#define SR1_PECERR_Pos		12u
#define	SR1_TIMEOUT_Pos		14u
#define SR1_SMBALERT_Pos	15u

/*
 * ===========================================================
 * 					I2C_SR2 BIT POSITION
 * ===========================================================
 */
#define SR2_MSL_Pos			0u
#define SR2_BUSY_Pos		1u
#define SR2_TRA_Pos			2u
#define SR2_GENCALL_Pos		4u
#define SR2_SMBDEFAULT_Pos	5u
#define SR2_SMBHOST_Pos		6u
#define SR2_DUALF_Pos		7u
#define SR2_PEC_Pos			8u

/*
 * ===========================================================
 * 					I2C_CCR BIT POSITION
 * ===========================================================
 */
#define I2C_CCR_Pos		0u
#define I2C_DUTY_Pos	14u
#define I2C_FS_Pos		15u

/*
 * ===========================================================
 * 					I2C_FLTR BIT POSITION
 * ===========================================================
 */
#define I2C_DNF_Pos		0u
#define I2C_ANOFF_Pos	4u


/*
 * ===========================================================
 * 					ENUMS
 * ===========================================================
 */
/* Status Codes for high-level I2C operations */
typedef enum{
	I2C_OK = 0,
	I2C_ERR_BUSY,
	I2C_ERR_START,
	I2C_ERR_ADDR,
	I2C_ERR_ACK,
	I2C_ERR_TXE,
	I2C_ERR_BTF,
	I2C_ERR_RXNE,
	I2C_WRITE_FAIL
}I2C_Status_t;

/* Identifers for I2C registers, used as indices into lookup tables */
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

/* I2C peripherals instances available on STM32F411 */
typedef enum{
	_I2C1,
	_I2C2,
	_I2C3,

	I2C_count
}I2C_Name_t;

/* CCR/Timing modes (standard-mode and Fast-mode variants */
typedef enum{
	I2C_SM_MODE,
	I2C_FM_MODE_2LOW_1HIGH,
	I2C_FM_MODE_16LOW_9HIGH
}I2C_CCR_Mode_t;

/*
 * ================================================================
 * 					TYPES
 * ================================================================
 */
/* GPIO mapping for a specific I2C Bus (SCL, SDA pins + ports */
typedef struct{
	I2C_Name_t 		i2cBus;

	GPIO_Pin_t 		sclPin;
	GPIO_PortName_t sclPort;

	GPIO_Pin_t 		sdaPin;
	GPIO_PortName_t sdaPort;
}I2C_GPIO_Config_t;

/* Initializa parameters for I2C timing*/
typedef struct{
	I2C_CCR_Mode_t 	ccrMode; //Timing mode (SM/FM)
	uint32_t 		sclFreq; //Desired SCL frequency in Hz
	uint32_t 		fclk1_Hz; //APB1 clock feeding I2C in Hz
}I2C_Timing_t;


/*
 * =================================================================
 * 					PUBLIC API FUNCTIONS
 * =================================================================
 */
/*
 * @brief	Write a field (bit or multi-bit) to an I2C register
 *
 * @param	i2cBus		Target I2C instance
 * @param	regName		Register identifier
 * @param	bitPosition First bit of the field
 * @param	value		Value to write (must fit the field for that position)
 */
bool writeI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t value);


/*
 * @brief Read a field (bit or multi-bit) from an I2C register.
 *
 * @param i2cBus       Target I2C instance.
 * @param regName      Register identifier (index into LUT).
 * @param bitPosition  First bit of the field (0..31).
 *
 * @return Extracted value on success; @ref I2C_VALUE_ERROR on failure.
 */
uint32_t readI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition);


/*
 * @brief Initialize SCL pin for a given bus (AF, OD, speed, pull).
 * @return true if the mapping is valid and configuration written; false otherwise.
 */
bool I2C_sclPinInit(GPIO_Pin_t sclPin, GPIO_PortName_t sclPort, I2C_Name_t i2cBus);


/*
 * @brief Initialize SDA pin for a given bus (AF, OD, speed, pull).
 * @return true if the mapping is valid and configuration written; false otherwise.
 */
bool I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus);

/*
 * @brief Initialize both SCL and SDA pins for the selected I2C bus.
 * @return true if both SCL and SDA were configured successfully; false otherwise.
 */
bool I2C_GPIO_init(I2C_GPIO_Config_t config);


/*
 * @brief Configure CCR[11:0], DUTY, F/S, and TRISE for a desired SCL rate.
 *
 * @param ccrMode  Timing mode (SM/FM variants).
 * @param sclFreq  Desired SCL frequency in Hz.
 * @param fclk1    APB1 clock feeding I2C, in Hz.
 * @param config   Bus selection (used to access the correct registers).
 *
 * @return true on success; false on invalid parameters or if PE is enabled.
 */
bool I2C_getCCR(I2C_CCR_Mode_t ccrMode, uint32_t sclFreq, uint32_t fclk1, I2C_GPIO_Config_t config);


/*
 * @brief Write one byte to an 8-bit sub-register on a 7-bit I2C slave.
 *
 * @param config       Bus/pin selection.
 * @param slaveAddr    7-bit slave address (unshifted).
 * @param slaveRegAddr 8-bit sub-register index on the slave.
 * @param value        Byte to write.
 *
 * @return true on success; false on NACK/timeout/parameter errors.
 */
bool I2C_writeReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t value);


/*
 * @brief	Write multiple bytes starting at a sub-register (burst write)
 *
 * Sequence (naster)
 * 		Wait untul BUSY = 0
 * 		START
 * 		Send <SlaveAddr | W>, wait ADDR = 1 (Abort on AF/NACK)
 * 		Clear ADDR by reading SR1 then SR2
 * 		Send sub-register, typically with autoIncrement bit set, wait TXE/BTF (abort on AF/NACK)
 * 		Send @p len data bytes back to back =, each waiting TXE/BTF
 * 		STOP
 *
 * @param	config			I2C bus/pin selection
 * @param	slaveAddr		7-bit UNSHIFTED slave address
 * @param	startRegAddr	SUB-register address
 * @param	autoIncreBitSet	Auto Increment bit/cmd of slave internal auto increment
 * @param	dataBuf			Pointer to @p len bytes to transmit
 * @param	len				Number of data bytes to write.
 */
bool I2C_writeBurst(I2C_GPIO_Config_t config,
                    uint8_t           slaveAddr,
                    uint8_t           startRegAddr,
					uint8_t 		  autoIncreBitSet,
                    const uint8_t*    dataBuf,
					uint16_t		  len);


/*
 * @brief	Read one byte from an 8-bit sub-register on a 7-bit I2C slave
 *
 * @param	config			Bus/pin selection
 * @param	slaveAddr		7-bit slave address (unshifted)
 * @param 	slaveRegAddr	8-bit sub register index on the slave
 * @param	outResult		Pointer to receive the read byte
 *
 * @return	true on success
 * 			false on NACK/timeout/parameter errors
 */
bool I2C_readReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t* outResult);


/*
 * @brief	Read multiple byte from an 8-bit sub-register on a 7-bit I2C slave
 *
 * @param	config			I2C bus/pin selection
 * @param	slaveAddr		7-bit UNSHIFTED slave address
 * @param	startRegAddr	SUB-register address
 * @param	autoIncreBitSet	Auto Increment bit/cmd of slave internal auto increment
 * @param	dataBuf			Pointer to @p len bytes to transmit
 * @param	len				Number of data bytes to write.
 */
bool I2C_readBurst(I2C_GPIO_Config_t config,
				   uint8_t           slaveAddr,
				   uint8_t           startRegAddr,
				   uint8_t			 autoIncreBitSet,
				   uint16_t			 len,
				   uint8_t* 		 outResultBuf);

/*
 * @brief Basic initialization: enable clock, configure GPIOs, program CR2/CCR/TRISE, enable PE.
 *
 * @param config     Bus + GPIO mapping.
 */
void I2C_Init(I2C_GPIO_Config_t config);

#endif /* INC_I2C_H_ */



















