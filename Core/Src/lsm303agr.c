/*
 * lsm303agr.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */
#include "lsm303agr.h"

/*
 * ==================================================================
 * 					INTERNAL HELPERS OF ACCELEROMETER
 * ==================================================================
 */
static inline bool writeAcc(const LSM303AGR_t* dev, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(dev -> i2c, dev -> addrAcc, regAddr, value);
}

static inline bool readAcc(const LSM303AGR_t* dev, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(dev -> i2c, dev -> addrAcc, regAddr, outResult);
}

static inline bool multiWriteAcc(const LSM303AGR_t* dev, uint8_t startRegAddr, uint8_t* dataBuf, uint16_t quantityOfReg){
	return I2C_writeBurst(dev -> i2c, dev -> addrAcc, startRegAddr, LSM303AGR_ADDR_AUTO_INC, dataBuf, quantityOfReg);
}

static inline bool multiReadAcc(const LSM303AGR_t* dev, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	return I2C_readBurst(dev -> i2c, dev -> addrAcc, startRegAddr, LSM303AGR_ADDR_AUTO_INC, quantityOfReg, outResultBuf);
}

/*
 * ==================================================================
 * 					INTERNAL HELPERS OF MAGNOMETER
 * ==================================================================
 */
static inline bool writeMag(const LSM303AGR_t* dev, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(dev -> i2c, dev -> addrMag, regAddr, value);
}

static inline bool readMag(const LSM303AGR_t* dev, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(dev -> i2c, dev -> addrMag, regAddr, outResult);
}

static inline bool multiWriteMag(const LSM303AGR_t* dev, uint8_t startRegAddr, uint8_t* dataBuf, uint16_t quantityOfReg){
	return I2C_writeBurst(dev -> i2c, dev -> addrMag, startRegAddr, LSM303AGR_ADDR_AUTO_INC, dataBuf, quantityOfReg);
}

static inline bool multiReadMag(const LSM303AGR_t* dev, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	return I2C_readBurst(dev -> i2c, dev -> addrMag, startRegAddr, LSM303AGR_ADDR_AUTO_INC, quantityOfReg, outResultBuf);
}

/*
 * ==================================================================
 * 					PUBLIC HELPERS OF ACCELOROMETER
 * ==================================================================
 */
bool LSM303AGR_writeAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return writeAcc(lsm303agrStruct, regAddr, value);
}

bool LSM303AGR_readAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return readAcc(lsm303agrStruct, regAddr, outResult);
}

bool LSM303AGR_multiWriteAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint8_t* dataBuf, uint16_t quantityOfReg){
	return multiWriteAcc(lsm303agrStruct, startRegAddr, dataBuf, quantityOfReg);
}

bool LSM303AGR_multiReadAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	return multiReadAcc(lsm303agrStruct, startRegAddr, quantityOfReg, outResultBuf);
}

/*
 * ==================================================================
 * 					PUBLIC HELPERS OF MAGNOMETER
 * ==================================================================
 */
bool LSM303AGR_writeMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return writeMag(lsm303agrStruct, regAddr, value);
}

bool LSM303AGR_readMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return readMag(lsm303agrStruct, regAddr, outResult);
}

bool LSM303AGR_multiWriteMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint8_t* dataBuf, uint16_t quantityOfReg){
	return multiWriteMag(lsm303agrStruct, startRegAddr, dataBuf, quantityOfReg);
}

bool LSM303AGR_multiReadMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	return multiReadMag(lsm303agrStruct, startRegAddr, quantityOfReg, outResultBuf);
}

/*
 * ==========================================================
 * 					PUBLIC HELPERS
 * ==========================================================
 */
/* @brief	Returns the WHO_AM_I values of both sub-devices */
bool LSM303AGR_whoAmI(const LSM303AGR_t* lsm303agrStruct, uint8_t* whoAcc, uint8_t* whoMag){
	if((lsm303agrStruct == NULL) | (whoAcc == NULL) | (whoMag == NULL)) return false;

	/* Accelerometer WHO_AM_I_ACC @ 0x0F should be 0x33 */
	if(!readAcc(lsm303agrStruct, LSM303AGR_WHO_AM_I_ACC, whoAcc)) return false;

	/* Magnetometer WHO_AM_I_MAG @ 0x4F should be 0x40 */
	if(!readMag(lsm303agrStruct, LSM303AGR_WHO_AM_I_MAG, whoMag)) return false;
	return true;
}

/* @brief	Check if ACC & MAG respond correctly */
bool LSM303AGR_isPresent(const LSM303AGR_t* lsm303agrStruct){
	uint8_t whoAcc = 0u;
	uint8_t whoMag = 0u;
	if(!LSM303AGR_whoAmI(lsm303agrStruct, &whoAcc, &whoMag)) return false;

	/* Expect IDs: ACC = 0x33 & MAG = 0x40 */
	return ((whoAcc == LSM303AGR_WHO_AM_I_ACC_VAL) &&
			(whoMag == LSM303AGR_WHO_AM_I_MAG_VAL));
}

/*
 * @brief	Reboot accelerometer memory content
 */
bool LSM303AGR_rebootAcc(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, &regVal)) return false; //Extract the current status bit-field and assign it to regVal

	/* Start to assign and write regVal to CTRL_REG5_ACC to start the reboot process */
	regVal |= (SET << REG5_ACC_BOOT_Pos);
	if(!writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, regVal)) return false; // Pass boot command
	HAL_Delay(50);
	return true;
}

/*
 * @brief	Reset all configuration registers and user registers
 */
bool LSM303AGR_softReset(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, &regVal)) return false; //Extract the current status bit-field and assign it to regVal

	/* Start to assign and write regVal to CFG_REG_A to start FULL CHIP RESET */
	regVal |= (SET << REG_A_SOFT_RST_Pos);
	if(!writeMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, regVal)) return false;
	HAL_Delay(50);
	return true;
}

/*
 * @brief	Disable internal temperature sensor
 */
bool LSM303AGR_disableTemperature(const LSM303AGR_t* lsm303agrStruct){
	uint8_t disableTemp = (0b00 << TEMP_ACC_ENABLE_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, disableTemp);
}

/*
 * @brief	Enable the internal temperature sensor
 */
bool LSM303AGR_enableTemperature(const LSM303AGR_t* lsm303agrStruct){
	uint8_t enableTemp = (0b11 << TEMP_ACC_ENABLE_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, enableTemp);
}

/*
 * @brief	Set ODR
 */
bool LSM303AGR_setODR(const LSM303AGR_t* lsm303agrStruct, ODR_Sel_t odr){
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &regVal)) return false;

	regVal |= (odr << REG1_ACC_ODR_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, regVal);
}


/*
 * @brief	Enable Block Data Update for coherent 16-bit reads
 * 			Output registers not updated until MSB and LSB have been read
 */
bool LSM303AGR_enableBDU(const LSM303AGR_t* lsm303agrStruct){
	uint8_t readReg4 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &readReg4)) return false;

	readReg4 |= (uint8_t)(SET << REG4_ACC_BDU_Pos); //BDU is bit 7
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, readReg4);
}

/*
 * @brief	Read Temperature value
 */
int8_t LSM303AGR_getTemperature(const LSM303AGR_t* lsm303agrStruct){
	if(!LSM303AGR_enableTemperature(lsm303agrStruct)) return false;
	if(!LSM303AGR_enableBDU(lsm303agrStruct)) return false;

	uint8_t tempBuf[2]; //tempBuf[0] = LSM303AGR_OUT_TEMP_L_ACC, tempBuf[1] = LSM303AGR_OUT_TEMP_H_ACC
	if(!multiReadAcc(lsm303agrStruct, LSM303AGR_OUT_TEMP_L_ACC, sizeof(tempBuf), tempBuf)) return false;

	int8_t rawTemp = (int8_t) tempBuf[1]; //LSM303AGR_OUT_TEMP_H_ACC is a signed 8-bit

	return rawTemp;
}











