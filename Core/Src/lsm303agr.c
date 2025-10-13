/*
 * lsm303agr.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */
#include "lsm303agr.h"
/*
 * ==================================================================
 * 					CONSTANTS/PARAMETERS DECLARATION
 * ==================================================================
 */
static const uint8_t LSM303AGR_accDefault[] = {
		//0x1F -> -0x26
		TEMP_CFG_REG_A_DF,
		CTRL_REG1_A_DF,
		CTRL_REG2_A_DF,
		CTRL_REG3_A_DF,
		CTRL_REG4_A_DF,
		CTRL_REG5_A_DF,
		CTRL_REG6_A_DF,
		REFERENCE_A_DF,

		//0x2E
		FIFO_CTRL_REG_A_DF,

		//0x30
		INT1_CFG_A_DF,

		//0x32 -> 0x34
		INT1_THS_A_DF,
		INT1_DURATION_A_DF,
		INT2_CFG_A_DF,

		//0x36 -> 0x38
		INT2_THS_A_DF,
		INT2_DURATION_A_DF,
		CLICK_CFG_A_DF,

		//0x3A -> 0x3F
		CLICK_THS_A_DF,
		TIME_LIMIT_A_DF,
		TIME_LATENCY_A_DF,
		TIME_WINDOW_A_DF,
		ACT_THS_A_DF,
		ACT_DUR_A_DF
};

static const RegSpan_t accSpans[] = {
		{LSM303AGR_TEMP_CFG_REG_ACC, 8},
		{LSM303AGR_FIFO_CTRL_REG_ACC, 1},
		{LSM303AGR_INT1_CFG_ACC, 1},
		{LSM303AGR_INT1_THS_ACC, 3},
		{LSM303AGR_INT2_THS_ACC, 3},
		{LSM303AGR_CLICK_THS_ACC, 6}
};

static const uint8_t LSM303AGR_magDefault[] = {
		//0x45 -> 0x4A
		OFFSET_X_REG_L_M_DF,
		OFFSET_X_REG_H_M_DF,
		OFFSET_Y_REG_L_M_DF,
		OFFSET_Y_REG_H_M_DF,
		OFFSET_Z_REG_L_M_DF,
		OFFSET_Z_REG_H_M_DF,

		//0x60 -> 0x63
		CFG_REG_A_M_DF,
		CFG_REG_B_M_DF,
		CFG_REG_C_M_DF,
		INT_CTRL_REG_M_DF,

		//0x65 -> 0x66
		INT_THS_L_REG_M_DF,
		INT_THS_H_REG_M_DF
};

static const RegSpan_t magSpan[] = {
		{LSM303AGR_OFFSET_X_REG_L_MAG, 6},
		{LSM303AGR_CFG_REG_A_MAG, 4},
		{LSM303AGR_INT_THS_L_REG_MAG, 2}
};

/*
 * ==================================================================
 * 					INTERNAL HELPERS OF ACCELEROMETER
 * ==================================================================
 */
static inline bool writeAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, regAddr, value);
}

static inline bool readAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, regAddr, outResult);
}

static inline bool multiWriteAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_writeBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, startRegAddr, isIncrement, dataBuf, quantityOfReg);
}

static inline bool multiReadAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_readBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, startRegAddr, isIncrement, quantityOfReg, outResultBuf);
}

/*
 * ==================================================================
 * 					INTERNAL HELPERS OF MAGNOMETER
 * ==================================================================
 */
static inline bool writeMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, regAddr, value);
}

static inline bool readMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, regAddr, outResult);
}

static inline bool multiWriteMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_writeBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, startRegAddr, isIncrement, dataBuf, quantityOfReg);
}

static inline bool multiReadMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_readBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, startRegAddr, isIncrement, quantityOfReg, outResultBuf);
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

bool LSM303AGR_multiWriteAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
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

bool LSM303AGR_multiWriteMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
	return multiWriteMag(lsm303agrStruct, startRegAddr, dataBuf, quantityOfReg);
}

bool LSM303AGR_multiReadMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	return multiReadMag(lsm303agrStruct, startRegAddr, quantityOfReg, outResultBuf);
}

/*
 * ==========================================================
 * 					ID VERIFICATION
 * ==========================================================
 */
/* @brief	Returns the WHO_AM_I values of both sub-devices */
bool LSM303AGR_whoAmI(const LSM303AGR_t* lsm303agrStruct, uint8_t* whoAcc, uint8_t* whoMag){
	if((lsm303agrStruct == NULL) || (whoAcc == NULL) || (whoMag == NULL)) return false;

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
 * ==========================================================
 * 				REBOOT AND RESET BOTH SENSORS
 * ==========================================================
 */
/*
 * @brief	Reboot acceleromter memory (reload factory trim)
 *			Does not reset user registers.
 */
static bool rebootAcc(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, &regVal)) return false; //Extract the current status bit-field and assign it to regVal

	/* Start to assign and write regVal to CTRL_REG5_ACC to start the reboot process */
	regVal |= (SET << REG5_ACC_BOOT_Pos);
	if(!writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, regVal)) return false; // Pass reboot command
	HAL_Delay(30);
	return true;
}

/*
 * @brief	Reboot magnetometer memory (read factory trim)
 * 			Does not reset user registers.
 */
static bool rebootMag(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, &regVal)) return false; //Extract the current status bit-field and assign it to regVal

	/* Start assigning and writting regVal to CTRL_REG5_ACC to start the reboot process */
	regVal |= (SET << REG_A_REBOOT_Pos);
	if(!writeMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, regVal)) return false; //Pass reboot command
	HAL_Delay(30);
	return true;
}

/*
 * @brief	Manually restore accelerometer user registers to power-up defaults.
 * 			Writes only contiguous R/W spans; skips Read Only and Reserved area
 */
static bool restoreAccDefaults(const LSM303AGR_t* lsm303agrStruct){
	const uint8_t* p = LSM303AGR_accDefault; //Separate pointer variable

	for(size_t i = 0; i < ARRLEN(accSpans); i++){
		if(!multiWriteAcc(lsm303agrStruct, accSpans[i].startReg, p, accSpans[i].len)) return false;
		//Move/update pointer p by accSpans[i].len
		p += accSpans[i].len;
	}
	return true;
}

/*
 * @brief	Manually restore magnetometer user registers to power-up defaults
 */
static bool restoreMagDefaults(const LSM303AGR_t* lsm303agrStruct){
	const uint8_t* p = LSM303AGR_magDefault; //Separate pointer variable

	for(size_t i = 0; i < ARRLEN(magSpan); i++){
		if(!multiWriteMag(lsm303agrStruct, magSpan[i].startReg, p, magSpan[i].len)) return false;
		p += magSpan[i].len;
	}
	return true;
}

/*
 * @brief	SET reset bit
 */
static bool getSoftReset(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, &regVal)) return false; //Extract the current status bit-field and assign it to regVal

	/* Start to assign and write regVal to CFG_REG_A to start FULL CHIP RESET */
	regVal |= (SET << REG_A_SOFT_RST_Pos);
	if(!writeMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, regVal)) return false;
	HAL_Delay(30);
	return true;
}

/*
 * @brief	Public helper for rebooting and resetting LSM303AGR registers to default values
 * 			FULL software reset:
 * 				1. Reboot accel and mag (factory trim reload)
 * 				2. SOFT_RST
 * 				3. MANUALLY write accel and mag defauls to user registers
 */
bool LSM303AGR_softReset(const LSM303AGR_t* lsm303agrStruct){
	/* Reboot the memory content of Acc and Mag	 */
	if(!rebootAcc(lsm303agrStruct)) return false;
	if(!rebootMag(lsm303agrStruct)) return false;

	/* Enable soft-reset */
	if(!getSoftReset(lsm303agrStruct)) return false;

	/* Manually reset Acc and Mag to Default Values */
	if(!restoreAccDefaults(lsm303agrStruct)) return false;
	if(!restoreMagDefaults(lsm303agrStruct)) return false;

	return true;
}

/*
 * ==========================================================
 * 			   DISABLE AND ENABLE TEMPERATURE
 * ==========================================================
 */
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
 * @brief	Read Temperature value
 */
bool LSM303AGR_getTemperature(const LSM303AGR_t* lsm303agrStruct, float* outTempC){
	/* Check for NULL pointer */
	if((lsm303agrStruct == NULL) || (outTempC == NULL)) return false;
	/* Get the current power mode */
	PowerMode_t currentMode;
	if(!LSM303AGR_getPowerMode(lsm303agrStruct, &currentMode)) return false;

	/* Read the raw temperature data */
	uint8_t temperatureBuf[2];
	if(!multiReadAcc(lsm303agrStruct, LSM303AGR_OUT_TEMP_L_ACC, sizeof(temperatureBuf), temperatureBuf)) return false;

	int16_t raw_16_val = 0;
	/* Process temperature value base on the power mode */
	switch(currentMode){
		case LOW_POWER_MODE:
			//Use the 8-bit formula for Low-Power mode
			int8_t raw_8_val = (int8_t)temperatureBuf[1];
			*outTempC = (float)raw_8_val + 25.0f;
			break;

		case NORMAL_POWER_MODE:
			//Use 10-bit formula for Normal Mode
			raw_16_val = (int16_t)((temperatureBuf[1] << 8) | temperatureBuf[0]);
			int16_t raw_10_val = raw_16_val >> 6; //Right-shift by 6 to extract 10-bit signed value
			*outTempC = ((float) raw_10_val / 4.0f) + 25.0f;
			break;

		case HIGH_RES_POWER_MODE:
			raw_16_val = (int16_t)((temperatureBuf[1] << 8) | temperatureBuf[0]);
			int16_t raw_12_val = raw_16_val >> 4; //Right-shift by 4 to extract 12-bit signed value
			*outTempC = ((float) raw_12_val / 16.0f) + 25.0f;
			break;

		default: return false;
	}
	return true; //Success!
}

/*
 * ===============================================================
 * 							OTHERS
 * ===============================================================
 */
/*
 * @brief	Enable Block Data Update for coherent 16-bit reads
 * 			Output registers not updated until MSB and LSB have been read
 */
bool LSM303AGR_enableBDU_acc(const LSM303AGR_t* lsm303agrStruct){
	uint8_t readReg4 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &readReg4)) return false;

	readReg4 |= (uint8_t)(SET << REG4_ACC_BDU_Pos); //BDU is bit 7
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, readReg4);
}

/*
 * @brief	Set ODR
 */
bool LSM303AGR_setODR_acc(const LSM303AGR_t* lsm303agrStruct, ODR_Sel_t odr){
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &regVal)) return false;
	regVal &= ~(0b1111 << REG1_ACC_ODR_Pos); //Clear the existing bits in the register value
	regVal |= (odr << REG1_ACC_ODR_Pos); //Set the new value
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, regVal);
}

/*
 * @brief	Set Power Mode
 */
bool LSM303AGR_setPowerMode(const LSM303AGR_t* lsm303agrStruct, PowerMode_t powerMode){
	/* Extract the existing bit values from the register CTRL_REG1 */
	uint8_t ctrlReg1Val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &ctrlReg1Val)) return false;

	/* Extract the existing but values from the register CTRL_REG4 */
	uint8_t ctrlReg4Val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &ctrlReg4Val)) return false;

	/* Clear both first */
	ctrlReg1Val &= (uint8_t) ~(1u << REG1_ACC_LPEN_Pos);
	ctrlReg4Val &= (uint8_t) ~(1u << REG4_ACC_HR_Pos);

	switch(powerMode){
		case LOW_POWER_MODE:
			//To enter Low-Power mode, we only need to SET the LPen bit.
			ctrlReg1Val |= (0b1 << REG1_ACC_LPEN_Pos);
			break;

		case NORMAL_POWER_MODE:
			//To enter Normal mode, we must CLEAR LPen and CLEAR HR bit.
			break;

		case HIGH_RES_POWER_MODE:
			//To enter High-Res mode, we must CLEAR LPen, and set HR.
			ctrlReg4Val |= (0b1 << REG4_ACC_HR_Pos);
			break;

		default: return false;
	}
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, ctrlReg1Val)
			&& writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, ctrlReg4Val);
}

/* @brief	Get power mode profile */
bool LSM303AGR_getPowerMode(const LSM303AGR_t* lsm303agrStruct, PowerMode_t* outMode){
	//Check for NULL pointers
	if(lsm303agrStruct == NULL || outMode == NULL){
		return false;
	}

	//Read the required control registers
	uint8_t valReg1 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &valReg1)) return false;

	uint8_t valReg4 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &valReg4)) return false;

	/* Check LPen bit. It has the highest priority. */
	if(valReg1 & (0b1 << REG1_ACC_LPEN_Pos)) *outMode = LOW_POWER_MODE;
	else {
		if(valReg4 & (0b1 << REG4_ACC_HR_Pos)) *outMode = HIGH_RES_POWER_MODE;
		else *outMode = NORMAL_POWER_MODE;
	}
	return true; //Success
}









