///*
// * @file	i2c.c
// *
// *  Created on: Aug 20, 2025
// *      Author: dobao
// *
// *	The module provides:
// *		Register lookup tables for I2C1-3 (avoids repetitive switch-cases)
// *		Mask tables that mark *reserved* bits so we never write them by mistake
// *		Tiny read/write helpers that perform field-sized RMW operations
// *		Pin initialization helpers for every legal SCL/SDA mapping on STM32F411 including correct pull-up handling
// */
//
//#include "i2c.h"
//
///*
// * ===============================================================================
// * Private Helpers
// * ===============================================================================
// */
//
///*
// * @brief	Bit-mask of **writable** bits for every I2C reg
// * 			A clear bit (0) marks a *reserved* position that **must not** be written.
// *
// * 			Index:  ::I2C_Reg_t
// */
//static const uint32_t I2C_validBits[I2C_REG_COUNT] = {
//		[I2C_CR1] = ~((0b1u << 2) | (0b1 << 14)),
//		[I2C_CR2] = ~((0b11u << 6) | (0b111 << 13)),
//		[I2C_OAR1] = ~(0b1111u << 10),
//		[I2C_OAR2] = ~(0b11111111u << 8),
//		[I2C_DR] = ~(0b11111111u << 8),
//		[I2C_SR1] = ~((0b1u << 5) | (0b1u << 13)),
//		[I2C_SR2] = ~(0b1u << 3),
//		[I2C_CCR] = ~(0b11u << 12),
//		[I2C_TRISE] = ~(0b1111111111u << 6),
//		[I2C_FLTR] = ~(0b11111111111u << 5)
//};
//
///*
// * @brief	Lookup table for the I2Cx peripheral register
// * 			This static pointer array maps each value in I2C_Reg_t (index) to the
// * 			corresponding I2Cx Register.
// * 			Using lookup table avoiding 'switch' cases and make register access simply.
// */
//static volatile uint32_t* I2C1_regLookUpTable[I2C_REG_COUNT] = {
//		[I2C_CR1] = I2C1_GET_REG(I2C_CR1),
//		[I2C_CR2] = I2C1_GET_REG(I2C_CR2),
//		[I2C_OAR1] = I2C1_GET_REG(I2C_OAR1),
//		[I2C_OAR2] = I2C1_GET_REG(I2C_OAR2),
//		[I2C_DR] = I2C1_GET_REG(I2C_DR),
//		[I2C_SR1] = I2C1_GET_REG(I2C_SR1),
//		[I2C_SR2] = I2C1_GET_REG(I2C_SR2),
//		[I2C_CCR] = I2C1_GET_REG(I2C_CCR),
//		[I2C_TRISE] = I2C1_GET_REG(I2C_TRISE),
//		[I2C_FLTR] = I2C1_GET_REG(I2C_FLTR)
//};
//
//static volatile uint32_t* I2C2_regLookUpTable[I2C_REG_COUNT] = {
//		[I2C_CR1] = I2C2_GET_REG(I2C_CR1),
//		[I2C_CR2] = I2C2_GET_REG(I2C_CR2),
//		[I2C_OAR1] = I2C2_GET_REG(I2C_OAR1),
//		[I2C_OAR2] = I2C2_GET_REG(I2C_OAR2),
//		[I2C_DR] = I2C2_GET_REG(I2C_DR),
//		[I2C_SR1] = I2C2_GET_REG(I2C_SR1),
//		[I2C_SR2] = I2C2_GET_REG(I2C_SR2),
//		[I2C_CCR] = I2C2_GET_REG(I2C_CCR),
//		[I2C_TRISE] = I2C2_GET_REG(I2C_TRISE),
//		[I2C_FLTR] = I2C2_GET_REG(I2C_FLTR)
//};
//
//static volatile uint32_t* I2C3_regLookUpTable[I2C_REG_COUNT] = {
//		[I2C_CR1] = I2C3_GET_REG(I2C_CR1),
//		[I2C_CR2] = I2C3_GET_REG(I2C_CR2),
//		[I2C_OAR1] = I2C3_GET_REG(I2C_OAR1),
//		[I2C_OAR2] = I2C3_GET_REG(I2C_OAR2),
//		[I2C_DR] = I2C3_GET_REG(I2C_DR),
//		[I2C_SR1] = I2C3_GET_REG(I2C_SR1),
//		[I2C_SR2] = I2C3_GET_REG(I2C_SR2),
//		[I2C_CCR] = I2C3_GET_REG(I2C_CCR),
//		[I2C_TRISE] = I2C3_GET_REG(I2C_TRISE),
//		[I2C_FLTR] = I2C3_GET_REG(I2C_FLTR)
//};
//
///*
// * @brief	Flexibly enable RCC for I2Cx
// */
//static bool I2C_clockEnable(I2C_Name_t I2Cx){
//	switch(I2Cx){
//		case _I2C1: __HAL_RCC_I2C1_CLK_ENABLE(); return true;
//		case _I2C2: __HAL_RCC_I2C2_CLK_ENABLE(); return true;
//		case _I2C3: __HAL_RCC_I2C3_CLK_ENABLE(); return true;
//		default: return false;
//	}
//}
//
///*
// * @brief	Check that 'reg' is in range and 'bitPosition' is not reserved
// * 			This function flexibly check if multi-bits are valid
// *
// * @return	true	Inserted bit indice is valid
// * 			false	...invalid or reserved bit
// */
//static inline bool isValidI2CBit(uint8_t bitPosition, uint8_t bitWidth, I2C_Reg_t reg){
//	if((reg >= I2C_REG_COUNT) ||
//	   (bitWidth == 0) ||
//	   ((bitPosition + bitWidth) > 32) ||
//	   (bitPosition > 31) ||
//	   (bitWidth > 32)) return false;
//
//	uint32_t mask = ((bitWidth == 32) ? 0xFFFFFFFFu : ((1u << bitWidth) - 1u) << bitPosition);
//	return(I2C_validBits[reg] & mask) == mask;
//}
//
///*
// * @brief	Generic masked write helper
// *
// * @param	reg		Pointers to the register
// * @param	bitPosition		First bit of the field
// * @param	bitWidth		Field width in bits
// * @param	value			Field value (must fit in @p bitWidth)
// */
//static bool writeI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
//	if((bitWidth < 32) && (value > ((1U << bitWidth) - 1U))) return false;
//
//	uint32_t mask = (bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1) << bitPosition;
//	uint32_t shiftedValue = value << bitPosition;
//	*reg = (*reg & ~mask) | (shiftedValue & mask);
//	return true; //Write process successfully
//}
//
///*
// * @brief	Read a field of "bitWidth" bits from a register starting from "bitPosition"
// *
// * @param	reg		Pointer to the register
// * @param	bitPosition		First bit of the field
// * @param	bitWidth		Field width
// */
//static uint32_t readI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
//	if(bitWidth == 32) return *reg;
//
//	uint32_t mask = ((1U << bitWidth) - 1U);
//	return(*reg >> bitPosition) & mask;
//}
//
///*
// * @brief	Write a bit-field to an I2C peripheral register
// * 			Figures how many bits the setting needs
// * 			Changes only the bis you asked for, leaving the rest unchanged
// */
//bool writeI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t value){
//	uint8_t bitWidth = 1;
//	switch(regName){
//		case I2C_CR1:
//			break;
//
//		case I2C_CR2:
//			if(bitPosition == 0) bitWidth = 6;
//			break;
//
//		case I2C_OAR1:
//			if(bitPosition == 1) bitWidth = 7;
//			else if(bitPosition == 8) bitWidth = 2;
//			break;
//
//		case I2C_OAR2:
//			if(bitPosition == 1) bitWidth = 7;
//			break;
//
//		case I2C_DR:
//			bitWidth = 8;
//			break;
//
//		case I2C_SR1:
//			if(bitPosition <= 7) return false;
//			break;
//
//		case I2C_SR2: return false;
//
//		case I2C_CCR:
//			if(bitPosition == 0) bitWidth = 12;
//			break;
//
//		case I2C_TRISE:
//			bitWidth = 6;
//			break;
//
//		case I2C_FLTR:
//			if(bitPosition == 0) bitWidth = 4;
//			break;
//
//		default: return false;
//	}
//
//	if(!isValidI2CBit(bitPosition, bitWidth, regName)) return false;
//
//	volatile uint32_t* regAddr = NULL;
//
//	switch(i2cBus){
//		case _I2C1:
//			regAddr = I2C1_regLookUpTable[regName];
//			break;
//
//		case _I2C2:
//			regAddr = I2C2_regLookUpTable[regName];
//			break;
//
//		case _I2C3:
//			regAddr = I2C3_regLookUpTable[regName];
//			break;
//
//		default: return false;
//	}
//
//	writeI2CBits(regAddr, bitPosition, bitWidth, value);
//	return true;
//}
//
///*
// * @brief	Read a bit from a desired slave's register
// */
//uint32_t readI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition){
//	uint32_t ERROR = 0xFFFFFFFF;
//	uint8_t bitWidth = 1;
//	switch(regName){
//		case I2C_CR1:
//			break;
//
//		case I2C_CR2:
//			if(bitPosition == 0) bitWidth = 6;
//			break;
//
//		case I2C_OAR1:
//			if(bitPosition == 1) bitWidth = 7;
//			else if(bitPosition == 8) bitWidth = 2;
//			break;
//
//		case I2C_OAR2:
//			if(bitPosition == 1) bitWidth = 7;
//			break;
//
//		case I2C_DR:
//			bitWidth = 8;
//			break;
//
//		case I2C_SR1:
//			break;
//
//		case I2C_SR2:
//			if(bitPosition == 8) bitWidth = 8;
//			break;
//
//		case I2C_CCR:
//			if(bitPosition == 0) bitWidth = 12;
//			break;
//
//		case I2C_TRISE:
//			bitWidth = 6;
//			break;
//
//		case I2C_FLTR:
//			if(bitPosition == 0) bitWidth = 4;
//			break;
//
//		default: return ERROR;
//	}
//	if(!isValidI2CBit(bitPosition, bitWidth, regName)) return ERROR;
//
//	volatile uint32_t* reg = NULL;
//	switch(i2cBus){
//		case _I2C1:
//			reg = I2C1_regLookUpTable[regName];
//			break;
//
//		case _I2C2:
//			reg = I2C2_regLookUpTable[regName];
//			break;
//
//		case _I2C3:
//			reg = I2C3_regLookUpTable[regName];
//			break;
//
//		default: return ERROR;
//	}
//
//	return readI2CBits(reg, bitPosition, bitWidth);
//}
//
///*
// * @brief	Helper functions to check if the bit status
// */
//static inline bool I2C_waitBitSet(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t timeout){
//	while((timeout--) != 0){
//		uint32_t result = readI2C(i2cBus, regName, bitPosition);
//		if(result == 0xFFFFFFFF) return false;
//		if(result == 1) return true;
//	}
//	return false;
//}
//
//static inline bool I2C_waitBitClear(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t timeout){
//	while((timeout--) != 0){
//		uint32_t result = readI2C(i2cBus, regName, bitPosition);
//		if(result == 0xFFFFFFFF) return false;
//		if(result == 0) return true;
//	}
//	return false;
//}
//
///*
// * ======================================================
// * Pin Initialization Helpers (SCL and SDA)
// * ======================================================
// */
//bool I2C_sclPinInit(GPIO_Pin_t sclPin, GPIO_PortName_t sclPort, I2C_Name_t i2cBus){
//	switch(i2cBus){
//		case _I2C1:
//			if(!((sclPort == my_GPIOB) && ((sclPin == my_GPIO_PIN_6) || (sclPin == my_GPIO_PIN_8)))){
//				return false;
//			}
//			break;
//
//		case _I2C2:
//			if(!((sclPort == my_GPIOB) && (sclPin == my_GPIO_PIN_10))){
//				return false;
//			}
//			break;
//
//		case _I2C3:
//			if(!((sclPort == my_GPIOA) && (sclPin == my_GPIO_PIN_8))){
//				return false;
//			}
//			break;
//
//		default: return false;
//	}
//	const GPIO_Mode_t afrReg = (sclPin < my_GPIO_PIN_8) ? AFRL : AFRH;
//	const bool hasExternalPullUp = ((sclPin == my_GPIO_PIN_6) && (sclPort == my_GPIOB));
//
//	/* Config SCL pin for I2C purpose */
//	Enable_GPIO_Clock(sclPort);
//	writePin(sclPin, sclPort, MODER, AF_MODE); //Set pin to Alternate Function mode
//	writePin(sclPin, sclPort, OTYPER, OPEN_DRAIN); //Set pin to Open-Drain mode - can only pull down, need pull up to go high
//	writePin(sclPin, sclPort, OSPEEDR, HIGH_SPEED); //Set outpin works at very high speed
//	writePin(sclPin, sclPort, PUPDR, hasExternalPullUp ? FLOATING : PULL_UP);
//	writePin(sclPin, sclPort, afrReg, AF4);
//
//	return true;
//}
//
//
//bool I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus){
//	uint8_t alternateFuncMode = 0xFF;
//
//	switch(i2cBus){
//		case _I2C1:
//			if((sdaPort == my_GPIOB) && ((sdaPin == my_GPIO_PIN_7) || (sdaPin == my_GPIO_PIN_9))){
//				alternateFuncMode = AF4;
//			} else return false;
//			break;
//
//		case _I2C2:
//			if((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_11)){
//				alternateFuncMode = AF4;
//			}
//			else if((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_3)){
//				alternateFuncMode = AF9;
//			} else return false;
//			break;
//
//		case _I2C3:
//			if((sdaPort == my_GPIOC) && (sdaPin == my_GPIO_PIN_9)){
//				alternateFuncMode = AF4;
//			} else if(((sdaPin == my_GPIO_PIN_4) && (sdaPort == my_GPIOB)) ||
//					((sdaPin == my_GPIO_PIN_8) && (sdaPort == my_GPIOB))){
//				alternateFuncMode = AF9;
//			} else return false;
//			break;
//
//		default: return false;
//	}
//
//	/* Config SDA GPIO for I2C purpose */
//	const GPIO_Mode_t afrReg = (sdaPin < my_GPIO_PIN_8) ? AFRL : AFRH;
//	const bool hasExternalPullUp = (sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_9);
//
//	Enable_GPIO_Clock(sdaPort);
//	writePin(sdaPin, sdaPort, MODER, AF_MODE); //Set alternate function mode for the pin
//	writePin(sdaPin, sdaPort, OTYPER, OPEN_DRAIN); //Set pin Open-Drain - can only pull down, need pull up to go high
//	writePin(sdaPin, sdaPort, OSPEEDR, HIGH_SPEED); //Set the pinOut a very high speed
//	writePin(sdaPin, sdaPort, PUPDR, hasExternalPullUp ? FLOATING : PULL_UP); //0b00 = floating, 0b01 = internal pull up resitor
//	writePin(sdaPin, sdaPort, afrReg, alternateFuncMode);
//
//	return true;
//}
//
///*
// * @brief	Init SCL and SDA pins
// */
//bool I2C_GPIO_init(I2C_GPIO_Config_t config){
//	if(config.i2cBus >= I2C_count){
//		return false;
//	}
//	/* Initi scl and sda pins */
//	bool sclInitOk = I2C_sclPinInit(config.sclPin, config.sclPort, config.i2cBus);
//	bool sdaInitOk = I2C_sdaPinInit(config.sdaPin, config.sdaPort, config.i2cBus);
//
//	return sclInitOk && sdaInitOk;
//}
//
///*
// * ======================================================
// * Set Up Frequency for SCL Pin
// * ======================================================
// */
///*
// * @brief	Config and assign a proper value for I2C_CCR[11:0] bit-field
// * 			I2C_CCR is used for configuring speed/frequency of SCL pin
// *
// * @param	sclFreq		Desired SCL frequency in Hz (100K or 400K)
// * @param	fclk1		Peripheral Clock Freq that feeds into I2C bus
// * @param	config		Structure holds I2C pins and I2C bus
// */
//bool I2C_getCCR(I2C_CCR_Mode_t ccrMode,
//				uint32_t sclFreq,
//				uint32_t fclk1,
//				I2C_GPIO_Config_t config){
//
//	if((sclFreq == 0) && (fclk1 == 0)) return false;
//	if((ccrMode == I2C_SM_MODE) && (fclk1 < 2000000u)) return false;
//	if((ccrMode != I2C_SM_MODE) && (fclk1 < 4000000u)) return false;
//
//	uint32_t ccrVal = 0;
//	uint32_t denominator = 0;
//	uint8_t fsMode = 0;
//	uint8_t dutyMode = 0;
//
//	switch(ccrMode){
//		case I2C_SM_MODE:
//			denominator = 2u;
//			break;
//
//		case I2C_FM_MODE_2LOW_1HIGH:
//			denominator = 3u;
//			fsMode = 1;
//			break;
//
//		case I2C_FM_MODE_16LOW_9HIGH:
//			denominator = 25u;
//			fsMode = 1;
//			dutyMode = 1;
//			break;
//
//		default: return false;
//	}
//
//	/* Ceil division so actual sclFreq <= requested */
//	ccrVal = (fclk1 + (denominator*sclFreq - 1)) / (denominator*sclFreq);
//
//	/* Clamp to legal range for the chosen mode */
//	uint8_t minCCR = (fsMode && dutyMode) ? 1u : 4u;
//	if(ccrVal < minCCR) ccrVal = minCCR;
//	if(ccrVal > 0xFFFU) ccrVal = 0xFFFU; //12 bit-field
//
//	//PE must be disable before CCR/TRISE writes
//	volatile uint32_t* cr1 = (config.i2cBus == _I2C1) ? I2C1_GET_REG(I2C_CR1) :
//							 (config.i2cBus == _I2C2) ? I2C2_GET_REG(I2C_CR1) :
//									 	 	 	 	 	I2C3_GET_REG(I2C_CR1);
//	if((*cr1) & 0x1u) return false; //If the PE is set, return false
//
//	/* Program CCR[11:0], DUTY and F/S */
//	if(!writeI2C(config.i2cBus, I2C_CCR, 0, ccrVal)) return false;
//	if(!writeI2C(config.i2cBus, I2C_CCR, 14, dutyMode)) return false;
//	if(!writeI2C(config.i2cBus, I2C_CCR, 15, fsMode)) return false;
//
//	//Program TRISE
//	uint32_t fclk1_MHz = (fclk1 + 500000U) / 1000000U;
//	uint32_t trise = (!fsMode) ? (fclk1_MHz + 1U) : ((fclk1_MHz * 300U)/(1000U + 1u));
//	if(trise > 63) trise = 63;
//
//	if(!writeI2C(config.i2cBus, I2C_TRISE, 0, trise)) return false;
//	return true;
//}
//
///*
// * @brief		Write one byte to a register of a 7-bit address I2C slave
// *
// * @Sequence:
// * 		1. Wait until the bus is idle
// * 		2. Generate a START
// * 		3. Send <SlaveAddr, Write cmd>
// * 		4. Abort on a NACK
// * 		5. Send the target register address
// * 		6. Send the data byte
// * 		7. Generate a STOP bit
// *
// * @param	config			::I2C_GPIO_Config_t, config.i2cBus (_I2C1, I2C2, _I2C3)
// * @param	slaveAddr		Slave device address which is a 7-bit address
// * @param	slaveRegAddr	Desired Register address of that slave that we want to write a value to
// * @param	value			Single Byte Data package is ready to be sent from master to slave device
// */
//bool I2C_writeReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t value){
//	const I2C_Name_t i2cBus = config.i2cBus;
//	uint32_t timeout = 100000u;
//	/* Wait until the bus is free */
//	if(!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout)) return false;
//
//	/* START bit signal generation */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) return false; //Wait until start bit is generated
//
//	/* Send the 7-bit slave address + 1-bit write mode */
//	uint8_t writeMode = 0; //Please adjust this base on the datasheet
//	uint8_t addrByte = (uint8_t)(slaveAddr << 1) | (writeMode << 0); //first 7-bits contain slave address and eighth bit contain write mode bit
//	if(!writeI2C(i2cBus, I2C_DR, 0, addrByte)) return false; //Write the slave address + write mode to the DR holder
//
//	/* Check the status after the first sending & check the failure */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)){
//		if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//			(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //Generate a STOP bit
//			(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET); //Clear AF
//		}
//		return false;
//	}
//
//	/* Dummy read SR1 and then SR2 to clear the ADDR flag */
//	(void)readI2C(i2cBus, I2C_SR1, 0); //Dummy read SR1
//	(void)readI2C(i2cBus, I2C_SR2, 0); //Dummy read SR2
//
//	/* Send the SUBADDR/slave register address */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;//Wait until Data register is empty
//	if(!writeI2C(i2cBus, I2C_DR, 0, slaveRegAddr)) return false; //Send the desired slave register address
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false; //Wait until data byte transfer succeeded
//	if(!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false; //Wait until there is a ACK signal from the slave
//
//	/* Send actual value that we want to write to that register */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;//Wait until Data register is empty
//	if(!writeI2C(i2cBus, I2C_DR, 0, value)) return false; //Send the desired value
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false; //Wait until data byte transfer succeeded
//	if(!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false; //Wait until there is a ACK signal from the slave
//
//	/* Generate a STOP bit */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;
//	return true;
//}
//
//#if 0
//bool I2C_readReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t* outResult){
//	const I2C_Name_t i2cBus = config.i2cBus;
//
//	/* Wait until the bus is free */
//	if(!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, 10000u)) return false;
//
//	/* START bit signal generation */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, 10000u)) return false; //Wait until start bit is generated
//
//	/* Send the 7-bit slave address + 1-bit write mode */
//	uint8_t mode = 0; //Please adjust this base on the datasheet
//	uint8_t addrByte = (uint8_t)(slaveAddr << 1) | (mode << 0); //first 7-bits contain slave address and eighth bit contain write mode bit
//	if(!writeI2C(i2cBus, I2C_DR, 0, addrByte)) return false; //Write the slave address + write mode to the DR holder
//
//	/* Check the status after the first sending & check the failure */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, 10000u)){
//		if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//			(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //Generate a STOP bit
//			(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET); //Clear AF
//		}
//		return false;
//	}
//
//	/* Dummy read SR1 and then SR2 to clear the ADDR flag */
//	(void)readI2C(i2cBus, I2C_SR1, 0); //Dummy read SR1
//	(void)readI2C(i2cBus, I2C_SR2, 0); //Dummy read SR2
//
//	/* Send the SUBADDR/slave register address */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, 10000u)) return false;//Wait until Data register is empty
//	if(!writeI2C(i2cBus, I2C_DR, 0, slaveRegAddr)) return false; //Send the desired slave register address
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, 10000u)) return false; //Wait until data byte transfer succeeded
//
//	/* Detect and handle SAK on subaddress*/
//	if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//		(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
//		(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
//		return false;
//	}
//
//	/* Start reading value/signal from the slave device */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false; //Generate a start bit
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, 10000u)) return false; //Wait for the start bit is generated
//
//	/* Send slave addr + read bit to request slave to start a reading mode */
//	mode = 1; //Indicate it is a read mode
//	addrByte = (slaveAddr << 1) | (mode << 0);
//	if(!writeI2C(i2cBus, I2C_DR, 0, addrByte)) return false;
//
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, 10000u)){
//		if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//			(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
//			(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
//		}
//		return false;
//	}
//
//	/* Single-byte receive: ACK = 0 (NACK), STOP = 1, then clear ADDR */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_ACK_Pos, RESET)) return false; //NACK the next byte
//	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //Generate a STOP bit
//
//	/* Read SR1 and SR2 to clear the bit ADDR in SR1 */
//	(void)readI2C(i2cBus, I2C_SR1, 0);
//	(void)readI2C(i2cBus, I2C_SR2, 0);
//
//	/* Check Receiver Buffer (RxNE) if data is arrived and go to I2C_DR to read it */
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_RXNE_Pos, 10000u)) return false;
//	*outResult = (uint8_t) readI2C(i2cBus, I2C_DR, 0);
//
//	return true;
//}
//
//#else
//bool I2C_readReg8(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t* outResult){
//	const I2C_Name_t i2cBus = config.i2cBus;
//
//	/* Wait until the bus is free */
//	while((readI2C(i2cBus, I2C_SR2, SR2_BUSY_Pos) & 1u) == 1u);
//
//	/* START bit signal generation */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
//	while((readI2C(i2cBus, I2C_SR1, SR1_SB_Pos) & 1u) == 0u); //Wait until start bit is generated
//
//	/* Send the 7-bit slave address + 1-bit write mode */
//	uint8_t addrWrite = (uint8_t)((slaveAddr << 1) | 0u); //first 7-bits contain slave address and eighth bit contain write mode bit
//	if(!writeI2C(i2cBus, I2C_DR, 0, addrWrite)) return false; //Write the slave address + write mode to the DR holder
//
//	/* Check the status after the first sending & check the failure */
//	while((readI2C(i2cBus, I2C_SR1, SR1_ADDR_Pos) & 1u) == 0u){
//		if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//			(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //Generate a STOP bit
//			(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET); //Clear AF
//			return false;
//		}
//	}
//
//	/* Dummy read SR1 and then SR2 to clear the ADDR flag */
//	(void)readI2C(i2cBus, I2C_SR1, 0); //Dummy read SR1
//	(void)readI2C(i2cBus, I2C_SR2, 0); //Dummy read SR2
//
//	/* Send the SUBADDR/slave register address */
//	while((readI2C(i2cBus, I2C_SR1, SR1_TXE_Pos) & 1u) == 0u); //Wait until Transmit Data Reg is empty
//	if(!writeI2C(i2cBus, I2C_DR, 0, slaveRegAddr)) return false; //Send the desired slave register address
//	while((readI2C(i2cBus, I2C_SR1, SR1_BTF_Pos) & 1u) == 0u); //Wait until data byte transfer successfully
//
//	/* Detect and handle SAK on subaddress*/
//	if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//		(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
//		(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
//		return false;
//	}
//
//	/* Start reading value/signal from the slave device */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false; //Generate a start bit
//	while((readI2C(i2cBus, I2C_SR1, SR1_SB_Pos) & 1u) == 0u); //Wait until start bit is generated
//
//	/* Send slave addr + read bit to request slave to start a reading mode */
//	uint8_t addrRead = (uint8_t)((slaveAddr << 1) | 1u);
//	if(!writeI2C(i2cBus, I2C_DR, 0, addrRead)) return false;
//
//	/* Check the status after the first sending & check the failure */
//	while((readI2C(i2cBus, I2C_SR1, SR1_ADDR_Pos) & 1u) == 0u){
//		if((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u){
//			(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //Generate a STOP bit
//			(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET); //Clear AF
//			return false;
//		}
//	}
//
//	/* Single-byte receive: ACK = 0 (NACK), STOP = 1, then clear ADDR */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_ACK_Pos, RESET)) return false; //NACK the next byte
//    if(!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;   /* STOP=1 */
//
//	/* Read SR1 and SR2 to clear the bit ADDR in SR1 */
//	(void)readI2C(i2cBus, I2C_SR1, 0);
//	(void)readI2C(i2cBus, I2C_SR2, 0);
//
//	/* Check Receiver Buffer (RxNE) if data is arrived and go to I2C_DR to read it */
//	while((readI2C(i2cBus, I2C_SR1, SR1_RXNE_Pos) & 1u) == 0u);
//	*outResult = (uint8_t) readI2C(i2cBus, I2C_DR, 0);
//
//	return true;
//}
//
//#endif
//
///*
// * =============================================
// * I2C INITIALIZING
// * =============================================
// */
//void I2C_basicConfigInit(I2C_GPIO_Config_t config, I2C_CCR_Mode_t ccrMode, uint32_t sclFreq, uint32_t fclk1_Hz){
//	const I2C_Name_t i2cBus = config.i2cBus;
//
//	(void)I2C_clockEnable(i2cBus); //Enable I2C Clock
//	if(!I2C_GPIO_init(config)) return;
//
//	/* Program CR2.FREQ = FCLK1 (MHZ) */
//	uint32_t fclk1_MHz = (fclk1_Hz + 500000U) / 1000000U; //Round to the nearest MHz
//	writeI2C(i2cBus, I2C_CR2, 0, fclk1_MHz);
//
//	writeI2C(i2cBus, I2C_CR1, 0, RESET); //Disable I2C Peripheral before configuring it
//	if(!I2C_getCCR(ccrMode, sclFreq, fclk1_Hz, config)) return; //Configuring CCR and TRISE values
//	writeI2C(i2cBus, I2C_CR1, 0, SET); //Enable I2C Peripheral
//}
//

/*
 * @file	i2c.c
 *
 *  Created on: Jul 4, 2025
 *      Author: dobao
 *
 *	The module provides:
 *		Register lookup tables for I2C1-3 (avoids repetitive switch-cases)
 *		Mask tables that mark *reserved* bits so we never write them by mistake
 *		Tiny read/write helpers that perform field-sized RMW operations
 *		Pin initialization helpers for every legal SCL/SDA mapping on STM32F411 including correct pull-up handling
 */
#include "i2c.h"

/*
 * -----------------------------------------------------------------
 * Private Helpers
 * -----------------------------------------------------------------
 */

/*
 * @brief	Bit-mask of **writable** bits for every I2C reg
 * 			A clear bit (0) marks a *reserved* position that **must not** be written.
 *
 * 			Index:  ::I2C_Mode_t
 */
static const uint32_t I2C_VALID_BITS[I2C_REG_COUNT] = {
		[I2C_CR1] = ~((1u << 2) | (1u << 14)),
		[I2C_CR2] = ~((0b11 << 6) | (0b111 << 13)),
		[I2C_OAR1] = ~(0b11111 << 10),
		[I2C_OAR2] = ~(0xFF << 8),
		[I2C_DR] = ~(0xFF << 8),
		[I2C_SR1] = ~((1u << 5) | (1u << 13)),
		[I2C_SR2] = ~(1u << 3),
		[I2C_CCR] = ~(0b11 << 12),
		[I2C_TRISE] = ~(0x3FF << 6),
		[I2C_FLTR] = ~(0x7FF << 5),
};

/*
 * @brief	Lookup table for the I2C1 peripheral register
 *
 * 			This static pointer array maps each value of ::I2C_Mode_t (index) to memory-mapped address
 * 			of the corresponding I2C1 Register. Using the table avoids open-coded 'switch'/'if' blocks
 * 			and makes register access simply
 *
 * 			**I2C_REG_COUNT**	Array Length
 */
#define REG_TABLE_ATTR static volatile uint32_t* const

REG_TABLE_ATTR I2C1RegLookupTable[I2C_REG_COUNT] = {
		[I2C_CR1] = GET_I2C1_REG(I2C_CR1),
		[I2C_CR2] = GET_I2C1_REG(I2C_CR2),
		[I2C_OAR1] = GET_I2C1_REG(I2C_OAR1),
		[I2C_OAR2] = GET_I2C1_REG(I2C_OAR2),
		[I2C_DR] = GET_I2C1_REG(I2C_DR),
		[I2C_SR1] = GET_I2C1_REG(I2C_SR1),
		[I2C_SR2] = GET_I2C1_REG(I2C_SR2),
		[I2C_CCR] = GET_I2C1_REG(I2C_CCR),
		[I2C_TRISE] = GET_I2C1_REG(I2C_TRISE),
		[I2C_FLTR] = GET_I2C1_REG(I2C_FLTR),
};

REG_TABLE_ATTR I2C2RegLookupTable[I2C_REG_COUNT] = {
		[I2C_CR1] = GET_I2C2_REG(I2C_CR1),
		[I2C_CR2] = GET_I2C2_REG(I2C_CR2),
		[I2C_OAR1] = GET_I2C2_REG(I2C_OAR1),
		[I2C_OAR2] = GET_I2C2_REG(I2C_OAR2),
		[I2C_DR] = GET_I2C2_REG(I2C_DR),
		[I2C_SR1] = GET_I2C2_REG(I2C_SR1),
		[I2C_SR2] = GET_I2C2_REG(I2C_SR2),
		[I2C_CCR] = GET_I2C2_REG(I2C_CCR),
		[I2C_TRISE] = GET_I2C2_REG(I2C_TRISE),
		[I2C_FLTR] = GET_I2C2_REG(I2C_FLTR),
};

REG_TABLE_ATTR I2C3RegLookupTable[I2C_REG_COUNT] = {
		[I2C_CR1] = GET_I2C3_REG(I2C_CR1),
		[I2C_CR2] = GET_I2C3_REG(I2C_CR2),
		[I2C_OAR1] = GET_I2C3_REG(I2C_OAR1),
		[I2C_OAR2] = GET_I2C3_REG(I2C_OAR2),
		[I2C_DR] = GET_I2C3_REG(I2C_DR),
		[I2C_SR1] = GET_I2C3_REG(I2C_SR1),
		[I2C_SR2] = GET_I2C3_REG(I2C_SR2),
		[I2C_CCR] = GET_I2C3_REG(I2C_CCR),
		[I2C_TRISE] = GET_I2C3_REG(I2C_TRISE),
		[I2C_FLTR] = GET_I2C3_REG(I2C_FLTR),
};


/*
 * -----------------------------------------------------------------
 * Bit-manipulation Helpers
 * -----------------------------------------------------------------
 */

/*
 * @brief	Check that 'mode' is in range and 'bitPosition' is not reserved
 * 			This function flexibly check if multi-bits are valid
 *
 * @return	true	Inserted bit indice is valid
 * 			false	...invalid or reserved bit
 */
static inline bool isValidI2CBit(uint8_t bitPosition, uint8_t bitWidth, I2C_Mode_t mode){
	if(mode >= I2C_REG_COUNT || bitWidth == 0 || ((bitPosition + bitWidth) > 32)) return false;
	uint32_t mask = ((bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1U) << bitPosition);

	return (I2C_VALID_BITS[mode] & mask) == mask;
}

/*
 * @brief	Generic masked write helper
 *
 * @param	reg		Pointers to the register
 * @param	bitPosition		First bit of the field
 * @param	bitWidth		Field width in bits
 * @param	value			Field value (must fit in @p bitWidth)
 */
static void writeI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	/*
	 * The function leaves the register unchanged if
	 * 		@p bitPosition larger than 31 because shifting by 32 is undefined in C
	 * 		@p value is too large for the field
	 * 		The field would spill past bit 31
	 */
	if(bitPosition > 31 || bitWidth > 32) return;
	if(bitWidth < 32 && value >= (1U << bitWidth)) return;
	if((bitWidth + bitPosition) > 32) return;

	//Mask off the old bit and OR with the new value
	uint32_t mask = (bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1) << bitPosition;
	uint32_t shiftedValue = (value << bitPosition);
	*reg = (*reg & ~mask) | (shiftedValue & mask);
}


/*
 * @brief	Read a field of "bitWidth" bits from a register starting at 'bitPosition'
 *
 * @param	reg (pointer) to the register
 * @param	bitPosition		Starting bit position (0-31)
 * @param	bitWidth		Number of bits/bit size that fit @p value
 */
static uint32_t readI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	if(bitWidth == 32) return *reg; //Full-word: no mask needed

	uint32_t mask = ((1U << bitWidth) - 1U);
	return (*reg >> bitPosition) & mask;
}


/*
 * -----------------------------------------------------------------
 * Clock Helpers
 * -----------------------------------------------------------------
 */

/*
 * @brief	Enable GPIOs' clock
 */
static void enableGPIOClock(GPIO_PortName_t port){
	switch(port){
		case my_GPIOA: my_RCC_GPIOA_CLK_ENABLE(); break;
		case my_GPIOB: my_RCC_GPIOB_CLK_ENABLE(); break;
		case my_GPIOC: my_RCC_GPIOC_CLK_ENABLE(); break;
		case my_GPIOD: my_RCC_GPIOD_CLK_ENABLE(); break;
		case my_GPIOE: my_RCC_GPIOE_CLK_ENABLE(); break;
		case my_GPIOH: my_RCC_GPIOH_CLK_ENABLE(); break;
		default: return;
	}
}


/*
 * -----------------------------------------------------------------
 * Pin Initialization Helpers (SCL and SDA)
 * -----------------------------------------------------------------
 */

/*
 * @brief	A helper function to intialize SCL pin for the selected I2C peripheral
 *
 * The function performs:
 * 		1. Validation of <pin, port> against the requested bus
 * 		2. GPIO setup:
 * 			Alternate function mode
 * 			Open-Drain mode
 * 			Very-high speed
 * 		3. Pull-up handling
 * 			PB6 already has a 4.7kOhm external pull-up resistor
 * 			Use internal pull-up resistor for the rest of the pins
 */
static I2C_Status_t I2C_sclPinInit(GPIO_Pin_t sclPin,
								   GPIO_PortName_t sclPort,
								   I2C_Name_t i2cBus){
	/* Validate pin, port, and bus first */
	switch(i2cBus){
		case my_I2C1:
			if(!(sclPort == my_GPIOB && (sclPin == my_GPIO_PIN_6 || sclPin == my_GPIO_PIN_8))){
				return I2C_INVALID_PIN;
			}
			break;

		case my_I2C2:
			if(!(sclPort == my_GPIOB && sclPin == my_GPIO_PIN_10)){
				return I2C_INVALID_PIN;
			}
			break;

		case my_I2C3:
			if(!(sclPort == my_GPIOA && sclPin == my_GPIO_PIN_8)){
				return I2C_INVALID_PIN;
			}
			break;

		default: return I2C_INVALID_BUS;
	}

	/* Config GPIO for I2C purpose */
	enableGPIOClock(sclPort);
	const GPIO_Mode_t afrReg = (sclPin < my_GPIO_PIN_8) ? AFRL : AFRH;
	const bool hasExtPullUp = (sclPin == my_GPIO_PIN_6 && sclPort == my_GPIOB);

	writePin(sclPin, sclPort, MODER, AF_MODE); //Set pin to Alternate Function Mode
	writePin(sclPin, sclPort, OTYPER, OPEN_DRAIN); //Set pin to Open-Drain mode
	writePin(sclPin, sclPort, OSPEEDR, HIGH_SPEED); //Set a very-high speed output pin
	writePin(sclPin, sclPort, PUPDR, hasExtPullUp ? FLOATING : PULL_UP);
	writePin(sclPin, sclPort, afrReg, AF4); //SCL pins use AF4 on all valid pins of this MCU

	return I2C_OK;
}


/*
 * @brief	Configure the **SDA** pin for the selected I2C peripheral.
 *
 * The function performs:
 * 		1. Validation of <pin, port> against the requested bus
 * 		2. GPIO setup:
 * 			Alternate function mode
 * 			Open-Drain mode
 * 			Very-high speed
 * 		3. Pull-up handling
 * 			PB9 already has a 4.7kOhm external pull-up resistor
 * 			Use internal pull-up resistor for the rest of the pins
 * 		4. AF9 handling for some SDA pins from I2C2 and I2C3 peripherals
 */
static I2C_Status_t I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus){
	uint8_t alternateFuncMode = 0xFF;

	switch(i2cBus){
		case my_I2C1:
			if((sdaPort == my_GPIOB && (sdaPin == my_GPIO_PIN_7 || sdaPin == my_GPIO_PIN_9))){
				alternateFuncMode = AF4;
			} else return I2C_INVALID_PIN;
			break;

		case my_I2C2:
			if(sdaPort == my_GPIOB && sdaPin == my_GPIO_PIN_11){
				alternateFuncMode = AF4;
			}
			else if(sdaPort == my_GPIOB && sdaPin == my_GPIO_PIN_3){
				alternateFuncMode = AF9;
			}
			else return I2C_INVALID_PIN;
			break;

		case my_I2C3:
			if(sdaPort == my_GPIOC && sdaPin == my_GPIO_PIN_9){
				alternateFuncMode = AF4;
			}
			else if((sdaPin == my_GPIO_PIN_4 && sdaPort == my_GPIOB) ||
					(sdaPin == my_GPIO_PIN_8 && sdaPort == my_GPIOB)){
				alternateFuncMode = AF9;
			}
			else return I2C_INVALID_PIN;
			break;

		default: return I2C_INVALID_BUS;
	}

	/* Config GPIO for I2C purpose*/
	enableGPIOClock(sdaPort);
	const GPIO_Mode_t afrReg = (sdaPin < my_GPIO_PIN_8) ? AFRL : AFRH;
	const bool hasExtPullUp = (sdaPort == my_GPIOB && sdaPin == my_GPIO_PIN_9);

	writePin(sdaPin, sdaPort, MODER, AF_MODE); //Set pin to Alternate Function Mode
	writePin(sdaPin, sdaPort, OTYPER, OPEN_DRAIN); //Set pin to Open-Drain mode
	writePin(sdaPin, sdaPort, OSPEEDR, HIGH_SPEED); //Set a very-high speed output pin
	writePin(sdaPin, sdaPort, PUPDR, hasExtPullUp ? FLOATING : PULL_UP); //0b00 = floating, 0b01 = internal pull-up resistor
	writePin(sdaPin, sdaPort, afrReg, alternateFuncMode);

	return I2C_OK;
}


/*
 * @brief	Intialize SCL and SDA pin of selected I2C peripheral
 *
 * @return	I2C_OK on success
 * 			I2C_INVALID_BUS on illegal peripheral index.
 */
static I2C_Status_t I2C_GPIO_init(I2C_GPIO_Config_t config){
	if(config.i2cBus >= my_I2C_COUNT){
		return I2C_INVALID_BUS;
	}
	/* Configure GPIOs */
	I2C_sclPinInit(config.sclPin, config.sclPort, config.i2cBus);
	I2C_sdaPinInit(config.sdaPin, config.sdaPort, config.i2cBus);
	return I2C_OK;
}


/*
 * @brief	Calculate the value that must be written into I2C_CCR register so the bus
 * 			clock (SCL) runs at the request speed, then write that value - together with
 * 			Fast/Standard mode and duty cycle bits to the hardware register
 *
 * @param	mode	::I2C_CCR_Mode_t
 * 					I2C_SM_100K: Standard-mode 100KHz
 * 					I2C_FM_400K_DUTY_2LOW_1HIGH: Fast-mode 400kHz, Tlow/Thigh = 2/1
 * 					I2C_FM_400k_DUTY_16LOW_9HIGH: Fast-mode 400kHz, TLow/Thigh = 16/9
 *
 * @param	sclFreq		Desired SCL frequency in hertz (100k or 400k)
 * @param	sysClkFreq	Peripheral Clock that feeds the I2C unit
 * @param 	config		Structure that hold the I2C pins and i2cBus
 *
 * @retval	uint32_t crr value (0 to 0x0FFF) on success
 * 			ERROR if something wrong.
 */
static I2C_Status_t I2C_getCCR(I2C_CCR_Mode_t mode,
					   uint32_t sclFreq,
					   uint32_t sysClkFreq,
					   I2C_GPIO_Config_t config){

	/*
	 * sclFreq must be non-zero and not greater than 400kHz
	 * sysClkFreq must be at least 2MHz
	 */
	if(sclFreq == 0 || sclFreq > 400000U || sysClkFreq == 0 || sysClkFreq < 2000000U) return I2C_ERROR;

	uint32_t minCcr = 4;
	uint32_t ccr = 0;
	uint8_t fsMode = 0;
	uint8_t dutyMode = 0;

	switch(mode){
		case I2C_SM_100K:
			fsMode = 0;
			dutyMode = 0;
			ccr = sysClkFreq / (2U * sclFreq);
			break;

		case I2C_FM_400K_DUTY_2LOW_1HIGH:
			fsMode = 1;
			dutyMode = 0;
			ccr = sysClkFreq / (3U * sclFreq);
			minCcr = 1; //Fast mode allows minimum CCR value = 1
			break;

		case I2C_FM_400K_DUTY_16LOW_9HIGH:
			fsMode = 1;
			dutyMode = 1;
			ccr = sysClkFreq / (25U * sclFreq);
			minCcr = 1; //Fast mode allows minimum CCR value = 1
			break;

		default: return I2C_ERROR;
	}
	if(ccr < minCcr) ccr = minCcr;
	if(ccr > 0x0FFF) return I2C_ERROR;

	/* Program bit 14 and 15 of I2C_CCR and write CCR val into I2C_CCR */
	writeI2C(15, config.i2cBus, I2C_CCR, fsMode);
	writeI2C(14, config.i2cBus, I2C_CCR, dutyMode);
	writeI2C(0, config.i2cBus, I2C_CCR, ccr);

	return I2C_OK;
}


/*
 * --------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------
 */

/*
 * @brief	write one byte to a register of a 7-bit addressed I2C slave
 *
 * Sequence:
 * 		1. Wait until the bus is idle.
 * 		2. Generate a START
 * 		3. Send <slaveAddr, Write>.
 * 		4. Abort on a NACK (AF flag)
 * 		5. Send the target register address
 * 		6. send the data byte
 * 		7. Generate a STOP
 *
 * @param	config			::I2C_GPIO_Config_t, config.i2cBus (my_I2C1 to my_I2C3)
 * @param	slaveAddr		Slave device address which is a 7-bits address
 * @param	slaveRegAddr	Desired reg addr of that slave device that we want to write the value in
 * @param	value			Single Byte Data packet is ready to be sent from master to slave device.
 */
I2C_Status_t I2C_singleByteWrite(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr, uint8_t value){
	while((readI2C(1, config.i2cBus, I2C_SR2) & 1u) == 1u); //Wait until bus is not busy

	/* Start a transaction */
	writeI2C(8, config.i2cBus, I2C_CR1, SET); //1: Start generation
	while((readI2C(0, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until start condition generated

	/* Send the 7-bit slave address + write bit */
	uint8_t addrByte = (slaveAddr << 1) | 0; //Offset slave addr to start at bit 1 and end at 7 and leave bitPos 0 = 0 which indicates write mode
	writeI2C(0, config.i2cBus, I2C_DR, addrByte); //Write the slave addr + write mode indicator to DR holder
	while((readI2C(1, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until the slave's address is sent

	/* Read SR1 and SR2 to clear the bit ADDR in SR1 */
	(void)readI2C(0, config.i2cBus, I2C_SR1); //Dummy read
	(void)readI2C(0, config.i2cBus, I2C_SR2); //Dummy read
	if((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u){ //AF?
		writeI2C(9, config.i2cBus, I2C_CR1, SET); //STOP
		return I2C_NACK;
	}
	while((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u); //Wait until there is ACK signal from slave

	/* Send the slave's register address (command byte) */
	while((readI2C(7, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data register (TxE) is empty
	writeI2C(0, config.i2cBus, I2C_DR, slaveRegAddr);
	while((readI2C(2, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data byte transfer succeeded
	while((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u); //Wait until there is ACK signal from slave

	/* Send the data byte / value to internal slave reg addr */
	while((readI2C(7, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data register (TxE) is empty
	writeI2C(0, config.i2cBus, I2C_DR, value);
	while((readI2C(2, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data byte transfer succeeded

	/* Generate stop bit */
	writeI2C(9, config.i2cBus, I2C_CR1, SET);

	return I2C_OK;
}

/*
 *
 */
I2C_Status_t I2C_singleByteRead(I2C_GPIO_Config_t config, uint8_t slaveAddr, uint8_t slaveRegAddr){
	while((readI2C(1, config.i2cBus, I2C_SR2) & 1u) == 1u); //Wait until bus is not busy

	/* Start a transaction */
	writeI2C(8, config.i2cBus, I2C_CR1, SET); //1: Start generation
	while((readI2C(0, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until start condition generated

	/* Send the 7-bit slave address + write bit */
	uint8_t addrByte = (slaveAddr << 1) | 0; //Offset slave addr to start at bit 1 and end at 7 and leave bitPos 0 = 0 which indicates write mode
	writeI2C(0, config.i2cBus, I2C_DR, addrByte); //Write slave addr + write mode indicator to DR holder
  	while((readI2C(1, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until the slave's address is sent

	/* Read SR1 and SR2 to clear the bit ADDR in SR1 */
	(void)readI2C(0, config.i2cBus, I2C_SR1); //Dummy read
	(void)readI2C(0, config.i2cBus, I2C_SR2); //Dummy read
	if((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u){ //AF?
		writeI2C(9, config.i2cBus, I2C_CR1, SET); //STOP
		return I2C_NACK;
	}
	while((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u); //Wait until there is ACK signal from slave

	/* Send the slave's register address (command byte) */
	while((readI2C(7, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data register (TxE) is empty
	writeI2C(0, config.i2cBus, I2C_DR, slaveRegAddr);
	while((readI2C(2, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until data byte transfer succeeded
	while((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u); //Wait until there is ACK signal from slave

	/* Start reading value/signal from the slave device */
	writeI2C(8, config.i2cBus, I2C_CR1, 1); //Generate a start bit
	while((readI2C(0, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until start condition generated

	/* Send slave addr + read bit to request slave to start a reading mode	 */
	addrByte = (slaveAddr << 1) | 1; //Offset slave addr which starts at bit 1 and end at bit 7 and leave bitPos 0 = 1 which indicates read mode
	writeI2C(0, config.i2cBus, I2C_DR, addrByte); //Write slave addr + read mode indicator to DR holder
	while((readI2C(1, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until the data byte in DR holder is sent successfully

	/* Read SR1 and SR2 to clear the bit ADDR in SR1 */
	(void)readI2C(0, config.i2cBus, I2C_SR1); //Dummy read
	(void)readI2C(0, config.i2cBus, I2C_SR2); //Dummy read
	while((readI2C(10, config.i2cBus, I2C_SR1) & 1u) == 1u); //Wait until there is ACK signal from slave

	/* Go to Reveiver buffer (RxNE) check if data is arrived and go to I2C_DR to read the data */
	while((readI2C(6, config.i2cBus, I2C_SR1) & 1u) == 0u); //Wait until Data register is full
	uint8_t data = (uint8_t) readI2C(0, config.i2cBus, I2C_DR);

	/* Generate stop bit */
	writeI2C(9, config.i2cBus, I2C_CR1, SET);

	return data;
}

/*
 * @brief	Initialize basic configurations for I2C
 *
 * 			Enable clock for selected I2C bus
 * 			Then, initialize I2C-related pins
 * 			Must select APB frequency that matches setting in RCC.c which is 50MHz
 * 			Have a helper function (I2C_getCCR) that get automatically get CCR value based on sysClkFreq, desired SCL clock and Sm or Fm mode
 * 			Set TRISE to let MCU know the time limit of getting from low to high so it can keep the timing orrect
 * 			Enable I2C peripheral
 *
 * @param 	config	Pin mapping and bus identifier for this I²C instance.
 * @param	mode	Timing profile (100kHz standard mode or one of the 400kHz fast-mode options).
 * @param	sclFreq 	Desired SCL clock in hertz (100000 or 400000).
 * @param	sysClkFreq 	Peripheral clock frequency driving the I²C hardware, in hertz.
 */
void I2C_basicConfigInit(I2C_GPIO_Config_t config,
						 I2C_CCR_Mode_t ccrMode,
						 uint32_t sclFreq,
						 uint32_t sysClkFreq){
	//Flexible enable I2C clock
	switch(config.i2cBus){
		case my_I2C1: my_RCC_I2C1_CLK_ENABLE(); break;
		case my_I2C2: my_RCC_I2C2_CLK_ENABLE(); break;
		case my_I2C3: my_RCC_I2C3_CLK_ENABLE(); break;
		default: return;
	}
	I2C_GPIO_init(config);
	writeI2C(0, config.i2cBus, I2C_CR1, RESET); //Disable I2C peripheral before configuring it
	writeI2C(0, config.i2cBus, I2C_CR2, (sysClkFreq/1000000U)); //Set this I2C's clock freq to 50MHz
	if(I2C_getCCR(ccrMode, sclFreq, sysClkFreq, config) != I2C_OK) return;
//	writeI2C(0, config.i2cBus, I2C_TRISE, 51); //1000ns(from I2C spec), 20ns from 50MHz (TRISE = 1 + (1000/20) = 51)
	writeI2C(0, config.i2cBus, I2C_CR1, SET); //Enable I2C peripheral
}


/*
 * @brief	Write a bit-field to an I2C peripheral register
 *
 * 			Figures out how many bits the setting needs
 * 			Won't touch ay bits the datasheet says are off-limits.
 * 			Changes only the bits you asked for, leaving the rest unchanged.
 * 			Does nothing at all if you pass a bad argument or pick an I2C unit that does not exit
 *
 * @param	bitPosition		The LSB index of the field (0-31)
 * @param	i2cBus			Which peripheral to talk to (my_I2C1, my_I2C2, my_I2C3)
 * @param	mode			Which register of I2C to access (enum @ref I2C_Mode_t)
 *
 * @param	value			The value to write. There is a helper function to check if
 * 							@p value fits into the target field width; if it does not,
 * 							the call ignore
 */
void writeI2C(uint8_t bitPosition, I2C_Name_t i2cBus, I2C_Mode_t mode, uint32_t value){
	//Early sanity
	uint8_t bitWidth = 1;

	switch(mode){
		case I2C_CR1:
			break;

		case I2C_CR2:
			if(bitPosition == 0) bitWidth = 6;
			break;

		case I2C_OAR1:
			if(bitPosition == 1) bitWidth = 7;
			else if(bitPosition == 8) bitWidth = 2;
			break;

		case I2C_OAR2:
			if(bitPosition == 1) bitWidth = 7;
			break;

		case I2C_DR:
			bitWidth = 8;
			break;

		case I2C_SR1:
			break;

		case I2C_SR2:
			if(bitPosition == 8) bitWidth = 8;
			break;

		case I2C_CCR:
			if(bitPosition == 0) bitWidth = 12;
			break;

		case I2C_TRISE:
			bitWidth = 6;
			break;

		case I2C_FLTR:
			if(bitPosition == 0) bitWidth = 4;
			break;

		default: return;
	}

	if(!isValidI2CBit(bitPosition, bitWidth, mode)) return;

	/* Map bus -> Register pointer	 */
	volatile uint32_t* reg = NULL;

	switch(i2cBus){
		case my_I2C1:
			reg = I2C1RegLookupTable[mode];
			break;

		case my_I2C2:
			reg = I2C2RegLookupTable[mode];
			break;

		case my_I2C3:
			reg = I2C3RegLookupTable[mode];
			break;

		default: return;
	}
	if(reg == NULL) return; //Peripheral not present on this part

	//Disallow writes to read-only status reg except write to clear bits
	if((mode == I2C_SR1 || mode == I2C_SR2) && value != 0){
		//Only allow write-to-clear its (set value to 0 to clear)
			return;
	}

	writeI2CBits(reg, bitPosition, bitWidth, value);
}


/*
 * @brief	Read a bit-field from an I2C peripheral register
 *
 * @param	bitPosition		The LSB index of the field (0-31)
 * @param	i2cBus			Which peripheral to talk to (my_I2C1, my_I2C2, my_I2C3)
 * @param	mode			Which register of I2C to access (enum @ref I2C_Mode_t)
 *
 * @param	value			The value to write. There is a helper function to check if
 * 							@p value fits into the target field width; if it does not,
 * 							the call ignore
 *
 * @return	The extracted field value on success.
 * 			If the cal is invalid, the constant @c 0xFFFFFFFF is returned as an ERROR_FLAG
 */
uint32_t readI2C(uint8_t bitPosition, I2C_Name_t i2cBus, I2C_Mode_t mode){
	uint32_t const ERROR_FLAG = 0xFFFFFFFF;
	uint8_t bitWidth = 1;

	switch(mode){
		case I2C_CR1:
			break;

		case I2C_CR2:
			if(bitPosition == 0) bitWidth = 6;
			break;

		case I2C_OAR1:
			if(bitPosition == 1) bitWidth = 7;
			else if(bitPosition == 8) bitWidth = 2;
			break;

		case I2C_OAR2:
			if(bitPosition == 1) bitWidth = 7;
			break;

		case I2C_DR:
			bitWidth = 8;
			break;

		case I2C_SR1:
			break;

		case I2C_SR2:
			if(bitPosition == 8) bitWidth = 8;
			break;

		case I2C_CCR:
			if(bitPosition == 0) bitWidth = 12;
			break;

		case I2C_TRISE:
			bitWidth = 6;
			break;

		case I2C_FLTR:
			if(bitPosition == 0) bitWidth = 0;
			break;

		default: return ERROR_FLAG;
	}

	if(!isValidI2CBit(bitPosition, bitWidth, mode)) return ERROR_FLAG;

	volatile uint32_t* reg = NULL;

	switch(i2cBus){
		case my_I2C1:
			reg = I2C1RegLookupTable[mode];
			break;

		case my_I2C2:
			reg = I2C2RegLookupTable[mode];
			break;

		case my_I2C3:
			reg = I2C3RegLookupTable[mode];
			break;

		default: return ERROR_FLAG;
	}

	if(reg == NULL) return ERROR_FLAG;

	return readI2CBits(reg, bitPosition, bitWidth);
}





