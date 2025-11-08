/*
 * lsm303agr.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */
#include "lsm303agr.h"
/*
 * ==================================================================
 * 					CONSTANTS / DEFAULT REGISTERS
 * ==================================================================
 */
/* Default values for writable accelerometer registers */
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

/* Address ranges for writable accelerometer register blocks */
static const RegSpan_t accSpans[] = {
		{LSM303AGR_TEMP_CFG_REG_ACC, 8},
		{LSM303AGR_FIFO_CTRL_REG_ACC, 1},
		{LSM303AGR_INT1_CFG_ACC, 1},
		{LSM303AGR_INT1_THS_ACC, 3},
		{LSM303AGR_INT2_THS_ACC, 3},
		{LSM303AGR_CLICK_THS_ACC, 6}
};

/* Default values for writable magnetometer registers */
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

/* Address ranges for writable accelerometer register blocks */
static const RegSpan_t magSpan[] = {
		{LSM303AGR_OFFSET_X_REG_L_MAG, 6},
		{LSM303AGR_CFG_REG_A_MAG, 4},
		{LSM303AGR_INT_THS_L_REG_MAG, 2}
};

/*
 * ===========================================================================
 * 			 INTERNAL WRITE AND READ FUNCTIONS OF ACCELEROMETER
 * ===========================================================================
 */
/*
 * @brief	Write a single Accelerometer register.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	regAddr				The desired LSM303AGR Accelerometer register to write
 * @param	value				A byte-value to write into @p regAddr
 *
 * @retVal	true	if I2C transaction succeeded
 * 			false	if I2C transaction failed
 */
static inline bool writeAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, regAddr, value);
}

/*
 * @brief	Read a single byte from Accelerometer register.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	regAddr				The desired LSM303AGR Accelerometer register to read from
 * @param	outResult			Output pointer to receive the read byte (non-NULL)
 *
 * @retVal	true	On successful I2C read and non-NULL @p outResult
 * 			false	On I2C error
 */
static inline bool readAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, regAddr, outResult);
}

/*
 * @brief	Multi-write helps to write a list of byte-values to the corresponding Accelerometer registers
 * 			Write multiple byte-values to multiple accelerometer registers in one burst using auto-increment address.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	startRegAddr		First accelerometer register address to write
 * @param	dataBuf				Data buffer stores byte-values to write
 * @param	quantityOfReg		Number of bytes/registers to write.
 *
 * @retVal	true	On successful burst write.
 * 			false	On I2C error.
 *
 * @note	Auto-increment is enabled only when @p quantityOfReg > 1
 */
static inline bool multiWriteAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_writeBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, startRegAddr, isIncrement, dataBuf, quantityOfReg);
}

/*
 * @brief	Read multiple consecutive accelerometer registers in one burst
 * 			Uses auto-increment addressing when reading more than one byte
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	startRegAddr		First Accelerometer register address to be read
 * @param	quantityOfReg		Number of registers to read
 * @param	outResultBuf		Output buffer to store the read bytes (non-NULL)
 *
 * @retVal	true	On successful burst read.
 * 			false	On I2C error.
 */
static inline bool multiReadAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_readBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrAcc, startRegAddr, isIncrement, quantityOfReg, outResultBuf);
}

/*
 * ==============================================================================
 * 				INTERNAL WRITE AND READ FUNCTIONS OF MAGNETOMETER
 * ==============================================================================
 */
/* Write a single magnetometer register */
static inline bool writeMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value){
	return I2C_writeReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, regAddr, value);
}

/* Read a single magnetometer register */
static inline bool readMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult){
	return I2C_readReg8(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, regAddr, outResult);
}

/* Write multiple consecutive magnetometer registers */
static inline bool multiWriteMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_writeBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, startRegAddr, isIncrement, dataBuf, quantityOfReg);
}

/* Read multiple consecutive magnetometer registers */
static inline bool multiReadMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf){
	uint8_t isIncrement = (quantityOfReg > 1) ? LSM303AGR_ADDR_AUTO_INC : 0u;
	return I2C_readBurst(lsm303agrStruct -> i2c, lsm303agrStruct -> addrMag, startRegAddr, isIncrement, quantityOfReg, outResultBuf);
}

/*
 * =============================================================================
 * 					PUBLIC API WRITE AND READ OF MAGNETOMETER
 * =============================================================================
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
 * ================================================================================
 * 					PUBLIC API WRITE AND READ OF MAGNETOMETER
 * ================================================================================
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
 * ========================================================================
 * 							DEVICE IDENTIFICATION
 * ========================================================================
 */
/*
 * @brief	read WHO_AM_I for accel and mag
 *
 * @param	lsm303agrStruct		Device descriptor (non-NULL)
 * @param	whoAcc				Output: Accelerometer ID (expect 0x33)
 * @param	whoMag				Output:	Magnetometer ID (expect 0x40)
 */
bool LSM303AGR_whoAmI(const LSM303AGR_t* lsm303agrStruct, uint8_t* whoAcc, uint8_t* whoMag){
	if((lsm303agrStruct == NULL) || (whoAcc == NULL) || (whoMag == NULL)) return false;
	if(!readAcc(lsm303agrStruct, LSM303AGR_WHO_AM_I_ACC, whoAcc)) return false;
	if(!readMag(lsm303agrStruct, LSM303AGR_WHO_AM_I_MAG, whoMag)) return false;
	return true;
}

/*
 * @brief	Check device presence by validating WHO_AM_I values
 *
 * @param	lsm303agrStruct		Device descriptor (non-NULL)
 * @retVal	true	if ACC = 0x33 and MAG = 0x40
 * 			false	otherwise or on read error
 */
bool LSM303AGR_isPresent(const LSM303AGR_t* lsm303agrStruct){
	uint8_t whoAcc = 0u;
	uint8_t whoMag = 0u;
	if(!LSM303AGR_whoAmI(lsm303agrStruct, &whoAcc, &whoMag)) return false;
	return ((whoAcc == LSM303AGR_WHO_AM_I_ACC_VAL) &&
			(whoMag == LSM303AGR_WHO_AM_I_MAG_VAL));
}

/*
 * =================================================================================
 * 							REBOOT AND RESET BOTH SENSORS
 * =================================================================================
 */
/*
 * @brief	Reboot the accelerometer memory (reload factory trim values).
 * 			This command reinitializes factory calibration data but does NOT reset user-accessible registers.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @retVal 	true	if reboot command succeeded
 * 			false	I2C error
 */
static bool rebootAcc(const LSM303AGR_t* lsm303agrStruct){
	/* Assign regVal to the current value/setting in register CTRL_REG5 */
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, &regVal)) return false;

	/* Set the BOOT bit to reload the internal calibration parameters */
	regVal |= (1u << REG5_ACC_BOOT_Pos);
	if(!writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG5_ACC, regVal)) return false;

	HAL_Delay(50); //Wait for reboot completion
	return true;
}

/*
 * @brief	Reboot the magnetometer memory (reload factory trim values).
 * 			Like rebootAcc, this command does not affect user registers.
 * 			Similar to rebootAcc but applies to the magnetometer block
 */
static bool rebootMag(const LSM303AGR_t* lsm303agrStruct){
	/* Assign regVal to the current values in register CFG_REG_A_MAG */
	uint8_t regVal = 0;
	if(!readMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, &regVal)) return false;

	/* Set the BOOT bit to reload the internal calibration parameters */
	regVal |= (1u << REG_A_REBOOT_Pos);
	if(!writeMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, regVal)) return false;

	HAL_Delay(50); //Wait for reboot completion
	return true;
}

/*
 * @brief	Restore all writable accelerometer registers to their power-up default values.
 * 			This manually writes contiguous writable register blocks defined in accSpans[] and skips read-only or reserved registers
 */
static bool restoreAccDefaults(const LSM303AGR_t* lsm303agrStruct){
	const uint8_t* p = LSM303AGR_accDefault; //Separate pointer variable, valid in C

	for(size_t i = 0; i < ARRLEN(accSpans); i++){
		if(!multiWriteAcc(lsm303agrStruct, accSpans[i].startReg, p, accSpans[i].len)) return false;
		p += accSpans[i].len; //Advance pointer to next data segment in LSM303AGR_accDefault array
	}
	return true;
}

/*
 * @brief	Restore all writable magnetometer registers to their power-up default values.
 * 			Write contiguous writable blocks defined in magSpan[].
 */
static bool restoreMagDefaults(const LSM303AGR_t* lsm303agrStruct){
	const uint8_t* p = LSM303AGR_magDefault; //Separate pointer variable, valid in C

	for(size_t i = 0; i < ARRLEN(magSpan); i++){
		if(!multiWriteMag(lsm303agrStruct, magSpan[i].startReg, p, magSpan[i].len)) return false;
		p += magSpan[i].len; //Advance pointer to the next data segment in LSM303AGR_magDefault array
	}
	return true;
}

/*
 * @brief	Activate global soft reset for both sensors ACC and MAG
 * 			Sets the SOFT_RST bit in magnetometer CFG_REG_A to reset both accelerometer and magnetometer internal logic and registers
 */
static bool activateSoftReset(const LSM303AGR_t* lsm303agrStruct){
	uint8_t regVal = 0;
	if(!readMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, &regVal)) return false;

	/* Set SOFT_RST bit to initiate full-chip reset	 */
	regVal |= (1u << REG_A_SOFT_RST_Pos);
	if(!writeMag(lsm303agrStruct, LSM303AGR_CFG_REG_A_MAG, regVal)) return false;

	HAL_Delay(50); //Wait for internal reset to finish
	return true;
}

/*
 * @brief	Perform a complete software reset sequence for both sensors
 * 			This function executes:
 * 				1. Reboot accel and mag (reload factory trims)
 * 				2. Activate SOFT_RST bit for global reset
 * 				3. Rewrite all user registers with their default values.
 *
 * @note	After reset, all sensor configurations (ODR, scale, power mode) must be reconfigured before normal operation.
 */
bool LSM303AGR_softReset(const LSM303AGR_t* lsm303agrStruct){
	if(!rebootAcc(lsm303agrStruct)) return false;
	if(!rebootMag(lsm303agrStruct)) return false;
	if(!activateSoftReset(lsm303agrStruct)) return false;
	if(!restoreAccDefaults(lsm303agrStruct)) return false;
	if(!restoreMagDefaults(lsm303agrStruct)) return false;
	return true;
}

/*
 * ======================================================================================
 * 			   						TEMPERATURE CONTROL
 * ======================================================================================
 */
/*
 * @brief	Disable the internal temperature sensor of the accelerometer block
 */
bool LSM303AGR_disableTemperature(const LSM303AGR_t* lsm303agrStruct){
	uint8_t val;
	if(!readAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, &val)) return false;

	/* Clear temperature enable bits [1:0] */
	val &= ~(3u << TEMP_ACC_ENABLE_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, val);
}

/*
 * @brief	Enable the internal temperature sensor of the accelerometer block
 */
bool LSM303AGR_enableTemperature(const LSM303AGR_t* lsm303agrStruct){
	uint8_t val;
	if(!readAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, &val)) return false;

	/* Set temperature enable bits [1:0] to 1 (temperature measurement ON) */
	val = (val & ~(3u << TEMP_ACC_ENABLE_Pos)) | (3u << TEMP_ACC_ENABLE_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_TEMP_CFG_REG_ACC, val);
}

/*
 * @brief	Read and convert raw temperature data to degrees Celcius
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	outTempC			Pointer to store computed temperature in C
 *
 * @note	Conversion method depends on the current power mode:
 * 				- Low Power Mode: 8-bit resolution
 * 				- Normal Mode: 10-bit resolution
 * 				- High-Res Mode: 12-bit resolution
 */
bool LSM303AGR_getTemperature(const LSM303AGR_t* lsm303agrStruct, float* outTempC){
	int8_t raw_8_val = 0;
	int16_t raw_10_val = 0;
	int16_t raw_12_val = 0;
	int16_t raw_16_val = 0;

	if((lsm303agrStruct == NULL) || (outTempC == NULL)) return false;

	/* Identify the current power mode to select proper scaling */
	PowerMode_t currentMode;
	if(!LSM303AGR_getPowerMode(lsm303agrStruct, &currentMode)) return false;

	/* Read raw temperature output (2 bytes: LSB, MSB) */
	uint8_t temperatureBuf[2];
	if(!multiReadAcc(lsm303agrStruct, LSM303AGR_OUT_TEMP_L_ACC, sizeof(temperatureBuf), temperatureBuf)) return false;

	/* Decode temperature based on power mode */
	switch(currentMode){
		case LOW_POWER_MODE:
			//Use the 8-bit formula for Low-Power mode
			raw_8_val = (int8_t)temperatureBuf[1];
			*outTempC = (float)raw_8_val + 25.0f;
			break;

		case NORMAL_POWER_MODE:
			//Use 10-bit formula for Normal Mode
			raw_16_val = (int16_t)((temperatureBuf[1] << 8) | temperatureBuf[0]);
			raw_10_val = raw_16_val >> 6; //Right-shift by 6 to extract 10-bit signed value
			*outTempC = ((float) raw_10_val / 4.0f) + 25.0f;
			break;

		case HIGH_RES_POWER_MODE:
			raw_16_val = (int16_t)((temperatureBuf[1] << 8) | temperatureBuf[0]);
			raw_12_val = raw_16_val >> 4; //Right-shift by 4 to extract 12-bit signed value
			*outTempC = ((float) raw_12_val / 16.0f) + 25.0f;
			break;

		default: return false;
	}
	return true;
}

/*
 * ==================================================================================================
 * 								SETUP AND VERIFY CONFIGURATIONS OF ACC
 * ==================================================================================================
 */
/*
 * @brief	Enable Block Data Update (BDU) for consistent 16-bit data reads.
 *			When BDU is enabled, output registers are not updated until both the LSB and MSB have beed read.
 * 			This prevents mismatched data when reading multi-byte values
 */
bool LSM303AGR_enableBDU(const LSM303AGR_t* lsm303agrStruct){
	uint8_t readReg4 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &readReg4)) return false;

	readReg4 |= (uint8_t)(1u << REG4_ACC_BDU_Pos); //SET BDU bit (bit 7)
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, readReg4);
}

/*
 * @brief	Configure accelerometer Output Data Rate (ODR)
 * 			Update the ODR bits [7:4] in CTRL_REG1_ACC to set the sampling rate
 * 			Also stores the selected value into the driver state struct (lsm303agrState)
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	lsm303agrState		Pointer to runtime state structure
 * @param	odr					Desired ODR selection from ODR_Sel_t enum
 */
bool LSM303AGR_setODR(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, ODR_Sel_t odr){
	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &regVal)) return false;

	regVal &= ~(0xFu << REG1_ACC_ODR_Pos); //Clear ODR bits [7:4]
	regVal |= (odr << REG1_ACC_ODR_Pos); //Set the new value

	/* Save the ODR selection to ODR_sel field in the struct */
	if(lsm303agrState){
		lsm303agrState -> ODR_sel = odr;
		lsm303agrState -> isODRSet = true; //Trigger this field to true to indicate it has been set successfully
	}
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, regVal);
}

/*
 * @brief	Configure accelerometer power mode (Low, Normal, or High-Resolution)
 * 			Sets or clears the LPen and HR bits in CTRL_REG1_ACC and CTRL_REG4_ACC
 * 			according to the desired power mode. Also updates state flags.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	lsm303agrState		Pointer to runtime state struct to save important configurations
 * @param	powerMode			Target power mode (LOW_POWER_MODE, NORMAL_POWER_MODE, HIGH_RES_POWER_MODE)
 *
 * @note	LPen (CTRL_REG1) enables Low Power Mode
 * 			HR (CTRL_REG4) enables High-Res Mode
 * 			Both cleared -> Normal mode
 */
bool LSM303AGR_setPowerMode(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, PowerMode_t powerMode){
	uint8_t ctrlReg1Val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &ctrlReg1Val)) return false;

	uint8_t ctrlReg4Val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &ctrlReg4Val)) return false;

	/* Clear LPen and HR bits first */
	ctrlReg1Val &= (uint8_t) ~(1u << REG1_ACC_LPEN_Pos);
	ctrlReg4Val &= (uint8_t) ~(1u << REG4_ACC_HR_Pos);

	switch(powerMode){
		case LOW_POWER_MODE:
			//To enter Low-Power mode, we only need to SET the LPen bit.
			ctrlReg1Val |= (1u << REG1_ACC_LPEN_Pos);
			break;

		case NORMAL_POWER_MODE:
			//To enter Normal mode, we must CLEAR LPen and CLEAR HR bit.
			break;

		case HIGH_RES_POWER_MODE:
			//To enter High-Res mode, we must CLEAR LPen, and set HR.
			ctrlReg4Val |= (1u << REG4_ACC_HR_Pos);
			break;

		default: return false;
	}

	/* Save power mode to the struct */
	if(lsm303agrState){
		lsm303agrState -> powerModeSel = powerMode;
		lsm303agrState -> isPowerModeSet = true; //Trigger this field to true to indicate it has been set successfully
		lsm303agrState -> isCalibrated = false; //Recommend re-calibration
	}

	return	writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, ctrlReg1Val) &&
			writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, ctrlReg4Val);
}

/*
 * @brief	Retrieve the currently active power mode
 * 			Reads LPen and HR bits to determine which power mode is currently active
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	outMode				Output pointer stores detected power mode
 */
bool LSM303AGR_getPowerMode(const LSM303AGR_t* lsm303agrStruct, PowerMode_t* outMode){
	if(lsm303agrStruct == NULL || outMode == NULL) return false;

	uint8_t valReg1 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &valReg1)) return false;

	uint8_t valReg4 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &valReg4)) return false;

	/* Check LPen bit which has priority over HR bit */
	if(valReg1 & (1u << REG1_ACC_LPEN_Pos)){
		*outMode = LOW_POWER_MODE;
	} else {
		if(valReg4 & (1u << REG4_ACC_HR_Pos)) *outMode = HIGH_RES_POWER_MODE;
		else *outMode = NORMAL_POWER_MODE;
	}
	return true;
}

/*
 * @brief	Returns mg per LSB based on current full-scale setting
 *
 * @param	lsm303agrStruct		Pointer to the device descriptor
 * @param	lsm303agrState		Pointer to the driver state.
 * @param	out_mgPerLSB		Pointer store the addr of output "ms per LSB"
 */
static bool get_mgPerLSB_acc(const LSM303AGR_t* lsm303agrStruct,
							 const LSM303AGR_State_t* lsm303agrState,
							 float* out_mgPerLSB){
	if(!lsm303agrStruct || !lsm303agrState || !out_mgPerLSB) return false;

	/* Initial declare fs as 2g */
	FullScale_t fs = _2g;
	if(lsm303agrState -> fullScaleSel){ //Check if there is full-scale value stored in state structure
		fs = lsm303agrState -> fullScaleSel;
	}
	else{
		if(!LSM303AGR_getFullScale(lsm303agrStruct, &fs)) return false; //Otherwise, get the current full-scale
	}

	float steps = 128.0f;

	switch(fs){
		case _2g: *out_mgPerLSB = 2000.0f / steps; break;
		case _4g: *out_mgPerLSB = 4000.0f / steps; break;
		case _8g: *out_mgPerLSB = 8000.0f / steps; break;
		case _16g: *out_mgPerLSB = 16000.0f / steps; break;
		default: return false;
	}
	return true;
}

/*
 * ================================================================================================
 * 									READ ACCELEROMETER MEASUREMENT
 * ================================================================================================
 */
/*
 * @brief	Check if new accelerometer XYZ data is available
 * 			Read STATUS_REG_ACC and checks the XYZDA (bit 3) flag
 *
 * @param	lsm303agrStruct		Pointer to the device descriptor
 */
bool LSM303AGR_isXYZ_available(const LSM303AGR_t* lsm303agrStruct){
	uint8_t val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_STATUS_REG_ACC, &val)) return false;

	return ((val >> XYZDA_Pos) & 0x01);
}

/*
 * @brief	Enablle all three accelerometer axes (X, Y, Z)
 *
 * 			Set bits [2:0] in CTRL_REG1_ACC to enable all axes
 * @param	lsm303agrStruct		Pointer to the device descriptor
 */
bool LSM303AGR_enableXYZ(const LSM303AGR_t* lsm303agrStruct){
	uint8_t val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, &val)) return false;

	/* Set bits [2:0] to enable X, Y, Z axes */
	val = (val & ~(0x7u << REG1_ACC_XEN_Pos)) | (0x7u << REG1_ACC_XEN_Pos);
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG1_ACC, val);
}

/*
 * @brief	Configure full-scale range (±2g, ±4g, ±8g, or ±16g)
 * 			Update FS bits [5:4] in CTRL_REG4_ACC and saves the selection to lsm303agrState
 *
 * @param	lsm303agrStruct		Pointer to the device descriptor
 * @param	lsm303agrState		Pointer to the driver state structure
 * @param	selectFullScale		Target full-scale value (FullScale_t enum)
 */
bool LSM303AGR_setFullScale(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, FullScale_t selectFullScale){
	uint8_t val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &val)) return false;

	val = (val & ~(0x3u << REG4_ACC_FS_Pos)) | (selectFullScale << REG4_ACC_FS_Pos);

	/* Save the full scale selection to the struct */
	if(lsm303agrState){
		lsm303agrState -> fullScaleSel = selectFullScale;
		lsm303agrState -> isFullScaleSet = true; //Indicate full-scale is set successfully
		lsm303agrState -> isCalibrated = false; //Recommend re-calibration
	}
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, val);
}

/*
 * @brief	Retrieve the currently configured full-scale range
 * 			Read FS bits [5:4] from CTRL_REG4_ACC
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	whichFullScale		Output pointer to store the current scale
 */
bool LSM303AGR_getFullScale(const LSM303AGR_t* lsm303agrStruct, FullScale_t* whichFullScale){
	uint8_t val = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG4_ACC, &val)) return false;

	*whichFullScale = (FullScale_t)((val >> REG4_ACC_FS_Pos) & 0x3u);
	return true;
}

/*
 * @brief	Get sensitivity (mg/LSB) according to power mode and full-scale setting
 * 			Looks up the conversion factor from datasheet tables.
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	lsm303agrState		Pointer to state (must contain powerModeSel and fullScaleSel)
 * @param	outSensitivity		Output pointer to store mg/LSB factor (float)
 */
static bool getSensitivity_mgLSB(const LSM303AGR_t* lsm303agrStruct,
								 LSM303AGR_State_t* lsm303agrState,
								 float* outSensitivity){

	/* Lookup table derived from LSM303AGR datasheet */
	switch(lsm303agrState -> powerModeSel){
		case HIGH_RES_POWER_MODE:
			switch(lsm303agrState -> fullScaleSel){
				case _2g: *outSensitivity = 0.98; break;
				case _4g: *outSensitivity = 1.95; break;
				case _8g: *outSensitivity = 3.9; break;
				case _16g: *outSensitivity = 11.72; break;
				default: return false;
			}
			break;

		case NORMAL_POWER_MODE:
			switch(lsm303agrState -> fullScaleSel){
				case _2g: *outSensitivity = 3.9; break;
				case _4g: *outSensitivity = 7.82; break;
				case _8g: *outSensitivity = 15.63; break;
				case _16g: *outSensitivity = 46.9; break;
				default: return false;
			}
			break;

		case LOW_POWER_MODE:
			switch(lsm303agrState -> fullScaleSel){
				case _2g: *outSensitivity = 15.63; break;
				case _4g: *outSensitivity = 31.26; break;
				case _8g: *outSensitivity = 62.52; break;
				case _16g: *outSensitivity = 187.58; break;
				default: return false;
			}
			break;
		default: return false;
	}
	if(lsm303agrState) lsm303agrState -> isAccSensitivitySet = true; //Indicate acc sensitivity is set successfully
	return true;
}

/*
 * @brief	Compute expected raw accelerometer values when the board is flat
 * 			Calculates theoretical +1g on Z-axis and 0g on X and Y for use in calibration
 *
 * @param	expectedAccX/Y/Z	Output expected raw counts for each axis.
 */
static bool getExpectedAccVal(const LSM303AGR_t* lsm303agrStruct,
							  LSM303AGR_State_t* lsm303agrState,
							  int32_t* expectedAccX,
							  int32_t* expectedAccY,
							  int32_t* expectedAccZ){

	if(!getSensitivity_mgLSB(lsm303agrStruct, lsm303agrState, &lsm303agrState -> accSensitivity)) return false;

	uint8_t shiftNum = (lsm303agrState -> powerModeSel == HIGH_RES_POWER_MODE) ? 4 :
					   (lsm303agrState -> powerModeSel == NORMAL_POWER_MODE) ? 6 : 8;

	/* Count for +1g AFTER shift, then convert to RAW */
	int32_t one_g_counts = (int32_t)lroundf(1000.0f / lsm303agrState -> accSensitivity); //e.g. 128 counts per 1g @ ±4g Normal
	int32_t one_g_raw = one_g_counts << shiftNum;

	/* Get sign of Z */
	int16_t raw[3] ;
	int32_t zSign = +1;
	if(LSM303AGR_readRawAcc(lsm303agrStruct, raw)){
		zSign = (raw[2] >= 0) ? +1 : -1;
	}

	*expectedAccX = 0;
	*expectedAccY = 0;
	*expectedAccZ = zSign * one_g_raw;

	return true;
}

/*
 * @brief	Read raw accelerometer output (16-bit signed) as signed 16-bit values.
 * 			Waits for new data, then reads 6 consecutive output bytes (X/Y/Z LSB_MSB)
 *
 * @param	rawAccBuf	Pointer to array of 3 int16_t for output
 */
bool LSM303AGR_readRawAcc(const LSM303AGR_t* lsm303agrStruct, int16_t rawAccBuf[3]){
	if(lsm303agrStruct == NULL) return false;

	/* Wait until new XYZ data is ready */
	while(!LSM303AGR_isXYZ_available(lsm303agrStruct));
	uint8_t rawBuf[6];

	//Read 6 consecutive bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
	if(!multiReadAcc(lsm303agrStruct, LSM303AGR_OUT_X_L_ACC, sizeof(rawBuf), rawBuf)) return false;

	/* Combine bytes into signed 16-bit integers (little endian) */
	rawAccBuf[0] = (int16_t)((rawBuf[1] << 8) | rawBuf[0]); //X
	rawAccBuf[1] = (int16_t)((rawBuf[3] << 8) | rawBuf[2]); //Y
	rawAccBuf[2] = (int16_t)((rawBuf[5] << 8) | rawBuf[4]); //Z

	return true;
}

/*
 * @brief	Perform accelerometer calibration (offset correction)
 * 			Computes offsets by averaging multiple samples while the boards is flat
 *
 * @param	sample	Number of samples to average
 */
bool LSM303AGR_accCalibrate(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, uint32_t sample){
	HAL_Delay(90); //Wait for 90ms for stable output

	int16_t tempRawBuf[3]; //A temporary buffer to store raw reading from Accelerometer

	int32_t sumAccX = 0;
	int32_t sumAccY = 0;
	int32_t sumAccZ = 0;

	int32_t averageAccX = 0;
	int32_t averageAccY = 0;
	int32_t averageAccZ = 0;

	int32_t expectedAccX = 0;
	int32_t expectedAccY = 0;
	int32_t expectedAccZ = 0;

	if(!getExpectedAccVal(lsm303agrStruct, lsm303agrState, &expectedAccX, &expectedAccY, &expectedAccZ)) return false;

	/* Discard 50 first samples because they are trash data */
	for(uint16_t i = 0; i < 50; i++){
		if(!LSM303AGR_readRawAcc(lsm303agrStruct, tempRawBuf)) return false;
	}

	/* Start storing valid data into xyz buffers */
	for(uint32_t i = 0; i < sample; i++){
		LSM303AGR_readRawAcc(lsm303agrStruct, tempRawBuf);
		sumAccX += tempRawBuf[0];
		sumAccY += tempRawBuf[1];
		sumAccZ += tempRawBuf[2];
	}

	/* Offset calculation for placing the board flat on the table
	 * offset = averageRaw - expectedRaw */
	averageAccX = sumAccX / (int32_t)sample;
	averageAccY = sumAccY / (int32_t)sample;
	averageAccZ = sumAccZ / (int32_t)sample;

	lsm303agrState -> offsetAccX = averageAccX - expectedAccX;
	lsm303agrState -> offsetAccY = averageAccY - expectedAccY;
	lsm303agrState -> offsetAccZ = averageAccZ - expectedAccZ;

	lsm303agrState -> isCalibrated = true; //Indicate calibration is done properly
	return true;
}

/*
 * @brief	Read and convert accelerometer data to mg (milli-g)
 * 			Applies calibration offsets and sensitivity scaling to raw data
 *
 * @param	outAcc_XYZ	Output array [X,Y,Z] in mg
 *
 * @note	Performs auto-calibration if not yet done.
 */
bool LSM303AGR_readAcc_mg(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, float outAcc_XYZ[3]){
	if((!lsm303agrState -> isPowerModeSet) || (!lsm303agrState -> isFullScaleSet)) return false;

	/* Check if calibration is done, if not, start to calibrate and take 300 samples as default	 */
	if((lsm303agrState -> isCalibrated) == false) LSM303AGR_accCalibrate(lsm303agrStruct, lsm303agrState, 200);
	int32_t _offsetAccX = lsm303agrState -> offsetAccX;
	int32_t _offsetAccY = lsm303agrState -> offsetAccY;
	int32_t _offsetAccZ = lsm303agrState -> offsetAccZ;

	int16_t raw[3];
	if(!LSM303AGR_readRawAcc(lsm303agrStruct, raw)) return false;

	uint8_t shiftNum = (lsm303agrState -> powerModeSel == HIGH_RES_POWER_MODE) ? 4 : (lsm303agrState -> powerModeSel == NORMAL_POWER_MODE) ? 6 : 8;

	/* Sign preserving right bit shift */
	int32_t rawCalibratedX = ((int32_t)raw[0] - _offsetAccX) >> shiftNum;
	int32_t rawCalibratedY = ((int32_t)raw[1] - _offsetAccY) >> shiftNum;
	int32_t rawCalibratedZ = ((int32_t)raw[2] - _offsetAccZ) >> shiftNum;

	outAcc_XYZ[0] = (float)(rawCalibratedX * lsm303agrState -> accSensitivity);
	outAcc_XYZ[1] = (float)(rawCalibratedY * lsm303agrState -> accSensitivity);
	outAcc_XYZ[2] = (float)(rawCalibratedZ * lsm303agrState -> accSensitivity);

	return true;
}


/*
 * ========================================================================
 * 					HIGH-PASS FILTER CONFIGURATION
 * ========================================================================
 */
/*
 * @brief	Retrieve the currently selected high-pass filter mode (HPM1:HPM0)
 * 			Reads CTRL_REG2_ACC bits [1:0] and decodes the active high-pass filter mode
 *
 * @param	lsm303agrStruct		Pointer to device descriptor
 * @param	whichHighPassMode	Output pointer to store the mode
 *
 */
bool LSM303AGR_getHighPassMode(const LSM303AGR_t* lsm303agrStruct, HighPassMode_t* whichHighPassMode){
	if(!lsm303agrStruct || !whichHighPassMode) return false;
	uint8_t val = 0u;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, &val)) return false;
	*whichHighPassMode = (HighPassMode_t) ((val >> REG2_ACC_HPM_Pos) & 0x3u);
	return true;
}

/*
 * @brief	Set high-pass filter mode (HPM1: HPM0)
 * 			Update only the high-pass mode bits in CTRL_REG2_ACC
 * 			Automatically skip the write if the mode is already set.
 *
 * @param	highPassMode	Desired mode (enum HighPassMode_t)
 */
bool LSM303AGR_setHighPassMode(const LSM303AGR_t* lsm303agrStruct, HighPassMode_t highPassMode){
	if(!lsm303agrStruct) return false;
	uint8_t regVal = 0u;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, &regVal)) return false;

	HighPassMode_t currentHighPassMode = (HighPassMode_t)((regVal >> REG2_ACC_HPM_Pos) & 0x3u);
	if(currentHighPassMode == highPassMode) return true;

	regVal = (uint8_t)((regVal & ~(0x3u << REG2_ACC_HPM_Pos)) | ((uint8_t)highPassMode << REG2_ACC_HPM_Pos));
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, regVal);
}

/*
 * @brief	Get current output data register path (filtered or bypassed)
 * 			Reads FDS bit in CTRL_REG2_ACC to determine whether the high-pass filter output is routed
 * 			to the data registers/FIFO.
 *
 * @param	outFDS	Output pointer to store FDS setting (HP_DATA_OUTREG_FIFO or HP_DATA_BYPASS)
 */
bool LSM303AGR_getFDS(const LSM303AGR_t* lsm303agrStruct, FDS_t* outFDS){
	if(!lsm303agrStruct || !outFDS) return false;

	uint8_t regVal = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, &regVal)) return false;
	*outFDS = (regVal & (1u << REG2_ACC_FDS_Pos)) ? HP_DATA_OUTREG_FIFO : HP_DATA_BYPASS;
	return true;
}

/*
 * @brief	Select output data register type
 * @note	FDS = 1 -> routes the filted data to output registers and FIFO
 * 			FDS = 0 -> filter bypassed (raw data sent to output)
 *
 * @param	selectFDS	Target mode from FDS_t enum
 */
bool LSM303AGR_setFDS(const LSM303AGR_t* lsm303agrStruct, FDS_t selectFDS){
	uint8_t regVal = 0u;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, &regVal)) return false;

	regVal = (uint8_t)((regVal & ~(0x1u << REG2_ACC_FDS_Pos)) | ((uint8_t)selectFDS << REG2_ACC_FDS_Pos));
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, regVal);
}

/*
 * @brief	Lookup table of cutoff frequencies (Hz) for each ODR and HPCF combination
 *			Each row corresponds to one Output Data Rate (ODR), containing the available cutoff frequencies for HPCF = 0...3
 */
static const HPFRow_t hpRows[] = {
		{1, {0.02f, 0.008f, 0.004f, 0.002f} },
		{10, {0.20f, 0.080f, 0.040f, 0.020f} },
		{25, {0.50f, 0.200f, 0.100f, 0.050f} },
		{50, {1.00f, 0.500f, 0.200f, 0.100f} },
		{100, {2.00f, 1.000f, 0.500f, 0.200f} },
		{200, {4.00f, 2.000f, 1.000f, 0.500f} },
		{400, {8.00f, 4.000f, 2.000f, 1.000f} },
};

/*
 * @brief	Convert ODR_Set_t enum value to frequency in Hz
 *
 * @param	odr	ODR	selection enum
 * @return	Sampling rate in Hz, or 0 if invalid
 */
static uint16_t odr_to_hz(ODR_Sel_t odr){
	switch(odr){
		case _1Hz: return 1; //1Hz
		case _10Hz: return 10; //10Hz
		case _25Hz: return 25; //25Hz
		case _50Hz: return 50; //50Hz
		case _100Hz: return 100; //100Hz
		case _200Hz: return 200; //200Hz
		case _400Hz: return 400; //400Hz
		case _1K620Hz: return 1620; //1620Hz
		case _1344HR_5376LP: return 1344; //1344Hz for HR/Normal power mode, and 5376Hz is for Low Power
		default: return 0;
	}
}

/*
 * @brief	Find matching cutoff-frequecy row for a given ODR
 *
 * @param	odr_hz	ODR in Hz
 * @param	outRow 	Output pointer to matched row (from hpRows)
 */
static bool getExactRow(uint16_t odr_hz, const HPFRow_t** outRow){
	if(!odr_hz || !outRow) return false; //NULL check
	/* Pick the matched row */
	for(size_t i = 0; i < ARRLEN(hpRows); i++){
		if(hpRows[i].ODR_HZ == odr_hz){
			*outRow = &hpRows[i];
			return true;
		}
	}
	return false; //No exact match
}

/*
 * @brief	Write desired high-pass cutoff frequency index into CTRL_REG2_ACC
 * 			Updates HPCF bits [5:4] based on selected index (0-3)
 *
 * @param	idx_desiredCutoffFreq	Index 0...3 for cutoff frequency
 */
static bool LSM303AGR_writeDesiredCutoffFreq(const LSM303AGR_t* lsm303agrStruct,
										   uint8_t idx_desiredCutoffFreq){
	if(!lsm303agrStruct) return false;
	if(idx_desiredCutoffFreq > 3u) return false; // only 4 choices

	/* Retrieve the current byte value in CTRL_REG2_ACC and assign to reg2 */
	uint8_t reg2 = 0;
	if(!readAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, &reg2)) return false;

	/* Clear HPCF [5:4], then set it to idx_desiredCutoffFreq */
	reg2 = (uint8_t)((reg2 & ~(0x3u << REG2_ACC_HPCF_Pos)) | ((idx_desiredCutoffFreq & 0x3u) << REG2_ACC_HPCF_Pos));
	return writeAcc(lsm303agrStruct, LSM303AGR_CTRL_REG2_ACC, reg2);
}

/*
 * @brief	Set high-pass cutoff frequency closet to a desired value
 * 			Finds the HPCF index whose frequency is closet to the requested cutoff
 * 			(based on current ODR). Then writes that index into CTRL_REG2_ACC
 *
 * @param	desiredCutoffFreq	This should be stay between....!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * @note	The available cutoff frequencies depend on the selected ODR
 */
bool LSM303AGR_setHPCutoffFreq(const LSM303AGR_t* lsm303agrStruct,
							   LSM303AGR_State_t* lsm303agrState,
							   float desiredCutoffFreq){
	/* Check if the pointers are NULL  */
	if(!lsm303agrStruct || !lsm303agrState) return false;

	/* Convert from ODR_Sel_t value type to a uint16_t value  */
	/* Pick the exact match high-pass filter row based on ODR profile */
	uint16_t odr_hz = odr_to_hz(lsm303agrState -> ODR_sel);
	const HPFRow_t* row = NULL;
	if(!getExactRow(odr_hz, &row)) return false;

	/* Choose the HPCF index whose frequency cutoff is closed to desired */
	uint8_t bestCutoff_idx = 0; //0 means idx = 0 assume the first CUTOFF_HZ[0] is the best match at the initial
	float bestCutoff_error = fabsf(desiredCutoffFreq - row -> CUTOFF_HZ[0]); //fabsf = float absolute value which remove sign from a float number
	/* Loop throught the remaining 3 cutoff frequencies (idx 1 to 3) */
	for(uint8_t i = 1; i < 4; i++){
		float errorEachOption = fabsf(desiredCutoffFreq - row -> CUTOFF_HZ[i]); //Calculate absolute difference for each option
		if(errorEachOption < bestCutoff_error){
			bestCutoff_error = errorEachOption;
			bestCutoff_idx = i; //Store the index of CUTOFF_HZ that has the best match to the requested cutoff frequency
		}
	}
	return LSM303AGR_writeDesiredCutoffFreq(lsm303agrStruct, bestCutoff_idx);
}

/*
 * =======================================================================================
 * 								ACTIVITY AND INACTIVITY FUNCTION
 * =======================================================================================
 */
/*
 * ACT_THS_A	0x3E	Activity Threshold defines how strong the acceleration must be to trigger "active" state
 * ACT_DUR_A	0x3F	Activity Duration defines how long the acceleration must stay below ACT_THS to be considered "inactive"
 * mg/LSB		How much physical acceleration in milli-g corresponds to one digital count in that register
 */

/*
 * @brief	Set Activity Threshold for inactivity and activity detection feature
 * 			This threshold defines the minimum acceleration change (in mg) that must
 * 			be exceeded to consider the device as active.
 * 			If the measured-acceleration-change is below this threshold for a duration
 * 			defined by @p desiredActDur, the device is considered inactive.
 *
 * @param	lsm303agrStruct		Pointer to device description
 * @param	lsm303agrState		Pointer to the sensor state
 * @param	desired_mgActThs	Desired Activity Threshold in milli-g (mg)
 */
bool setActThsAcc(const LSM303AGR_t* lsm303agrStruct,
				  LSM303AGR_State_t* lsm303agrState,
				  int32_t desired_mgActThs){
	/* Check NULL */
	if(!lsm303agrStruct || !lsm303agrState) return false;

	/* Obtain milli-g per LSB */
	float mgPerLSB = 0.0f;
	if(!get_mgPerLSB_acc(lsm303agrStruct, lsm303agrState, &mgPerLSB)) return false;
	if(mgPerLSB <= 0.0f) return false;

	/* Compute the digital count for ACT_THS and round it to the nearest integer*/
	int digitalCountActThs = (int)lroundf((float)desired_mgActThs / mgPerLSB);

	/* Clamp to 7-bits field [6:0] */
	if(digitalCountActThs < 0) digitalCountActThs = 0;
	if(digitalCountActThs > 0x7Fu) digitalCountActThs = 0x7Fu;

	/* Write digitalCountActThs to the actual register (ACT_THS_A) */
	uint8_t regVal;
	if(!readAcc(lsm303agrStruct, LSM303AGR_ACT_THS_ACC, &regVal)) return false;
	regVal = (uint8_t)((regVal & ~0x7Fu) | (uint8_t)(digitalCountActThs)); // Clear the existing value in the register before writting a new value

	return writeAcc(lsm303agrStruct, LSM303AGR_ACT_THS_ACC, regVal);
}

/*
 * @brief	Set Activity Duration for inactivity and activity feature
 * 			When the acceleration becomes lower than @p desired_mgActThs,
 * 			at a certain duration @p desiredActDur, ACC considered to be inactive.
 * @param	lsm303agrStruct		Pointer to device description
 * @param	lsm303agrState		Pointer to the sensor state
 * @param	desiredActDur		Desired inactivity duraction in seconds
 */
bool setActDurAcc(const LSM303AGR_t* lsm303agrStruct,
				  LSM303AGR_State_t* lsm303agrState,
				  float desiredActDur){
	if(!lsm303agrStruct || !lsm303agrState) return false;

	/* Retrieve the current Output Data Rate & and convert ODR to Hz */
	const uint16_t odr_hz = odr_to_hz(lsm303agrState -> ODR_sel);

	/* Compute register code with proper rounding */
	float digitalTimeCount = (float)((desiredActDur * (float)odr_hz - 1.0f) / 8.0f); //This formula according to datasheet
	int roundedDigitialTimeCount = (int)roundf(digitalTimeCount);
	if(roundedDigitialTimeCount < 0) return false;
	if(roundedDigitialTimeCount > 255) return false;

	uint8_t regVal;
	if(!readAcc(lsm303agrStruct, LSM303AGR_ACT_DUR_ACC, &regVal)) return false;
	regVal = (uint8_t)((regVal & ~0xFFu) | (roundedDigitialTimeCount));
	return writeAcc(lsm303agrStruct, LSM303AGR_ACT_DUR_ACC, regVal);
}




