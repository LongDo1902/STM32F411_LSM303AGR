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
//static inline bool isValidI2CBit(uint8_t bitPosition, uint8_t bitWidth, I2C_Reg_t regName){
//	if((regName >= I2C_REG_COUNT) ||
//	   (bitWidth == 0) ||
//	   ((bitPosition + bitWidth) > 32) ||
//	   (bitPosition > 31) ||
//	   (bitWidth > 32)) return false;
//
//	uint32_t mask = ((bitWidth == 32) ? 0xFFFFFFFFu : ((1u << bitWidth) - 1u) << bitPosition);
//	return(I2C_validBits[regName] & mask) == mask;
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
//			bitWidth = 1;
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
//			bitWidth = 1;
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
///*
// * @brief	Read one byte from an I2C 8-bit slave register
// *
// * @Sequences:
// * 		1. Wait bus idle
// * 		2. START
// * 		3. Send <addr|write>
// * 		4. Wait ADDR = 1, clear ADDR
// * 		5. Send slave register addr, wait BTF = 1
// * 		6. Repeated START
// * 		7. Send <addr|read>, wait ADDR = 1, clear ADDR
// * 		8. Wait RXNE = 1, read DR
// * 		9. Issue stop
// */
//bool I2C_readReg8(I2C_GPIO_Config_t config,
//                  uint8_t slaveAddr,
//                  uint8_t slaveRegAddr,
//                  uint8_t* outResult){
//	const I2C_Name_t i2cBus = config.i2cBus;
//	const uint32_t timeout = 10000u;
//
//	/* Ensure bus is idle */
//	if(!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout));
//
//	/* START condition, then wait for SB to assert */
//	if(!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
//	if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) return false;
//
//	/* Send <slaveAddr | Write>, then wait for address phase complete (ADDR = 1) */
//    uint8_t addrWrite = (uint8_t)((slaveAddr << 1) | 0u);
//    if(!writeI2C(i2cBus, I2C_DR, 0, addrWrite)) return false;
//    if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
//
//    /* Clear ADDR by completing the SR1→SR2 read sequence: SR1 was read in the wait helper;
//     * we read SR2 here to finish the clear. */
//    (void)readI2C(i2cBus, I2C_SR2, 0);
//
//    /* Confirm no NACK (AF must be 0). */
//    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false;
//
//    /* Write the sub-register address; wait for BTF (byte transfer finished) */
//    if(!writeI2C(i2cBus, I2C_DR, 0, slaveRegAddr)) return false;
//    if(!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, 10000u)) return false;
//
//    /* Repeated START for the read phase; wait SB again. */
//	if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
//	if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, 10000u)) return false;
//
//	/* Send <slaveAddr | Read>, then wait ADDR=1. */
//	uint8_t addrRead = (uint8_t)((slaveAddr << 1) | 1u);
//	if (!writeI2C(i2cBus, I2C_DR, 0, addrRead)) return false;
//	if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, 10000u)) return false;
//
//	/* Clear ADDR (again, SR1 read occurred inside the wait helper; finish by reading SR2). */
//	(void)readI2C(i2cBus, I2C_SR2, 0);
//
//	/* Confirm no NACK (AF must be 0). */
//	if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, 10000u)) return false;
//
//	/* Wait for data ready (RXNE), then read the single byte from DR. */
//	if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_RXNE_Pos, 10000u)) return false;
//	*outResult = (uint8_t)readI2C(i2cBus, I2C_DR, 0);
//
//	/* STOP condition to release the bus. */
//	if (!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;
//
//	return true;
//}
//
///*
// * =============================================
// * I2C INITIALIZING
// * =============================================
// */
//
//I2C_Timing_t timingParam= {
//		.ccrMode = I2C_SM_MODE, //I2C Standard Mode
//		.sclFreq = 100000U,		//100,000 Hz
//		.fclk1_Hz = SYSCLK_FREQ_16M //Default sys clock speed
//};
//
//static void I2C_getFreq(I2C_GPIO_Config_t config, I2C_CCR_Mode_t ccrMode, uint32_t sclFreq, uint32_t fclk1_Hz){
//	/* Program CR2.FREQ = FCLK1 (MHZ) */
//	uint32_t fclk1_MHz = (fclk1_Hz + 500000U) / 1000000U; //Round to the nearest MHz
//	writeI2C(config.i2cBus, I2C_CR2, 0, fclk1_MHz);
//
//	writeI2C(config.i2cBus, I2C_CR1, 0, RESET); //Disable I2C Peripheral before configuring it
//	if(!I2C_getCCR(ccrMode, sclFreq, fclk1_Hz, config)) return; //Configuring CCR and TRISE values
//	writeI2C(config.i2cBus, I2C_CR1, 0, SET); //Enable I2C Peripheral
//
//}
//
//void I2C_basicConfigInit(I2C_GPIO_Config_t config){
//	const I2C_Name_t i2cBus = config.i2cBus;
//
//	(void)I2C_clockEnable(i2cBus); //Enable I2C Clock
//	if(!I2C_GPIO_init(config)) return;
//
//	I2C_getFreq(config,
//				timingParam.ccrMode,
//				timingParam.sclFreq,
//				timingParam.fclk1_Hz);
//}





/*
 * @file    i2c.c
 * @brief   Low-level I2C helpers (register LUTs, bitfield R/W, GPIO init, timing)
 *
 * Created : Aug 20, 2025
 * Author  : dobao
 *
 * This module provides:
 *  - Register lookup tables for I2C1–3 (avoid repetitive switch-cases)
 *  - Mask tables that mark *reserved* bits so we never write them by mistake
 *  - Tiny read/write helpers that perform field-sized RMW operations
 *  - Pin initialization helpers for every legal SCL/SDA mapping on STM32F411
 *    including correct pull-up handling
 */

#include "i2c.h"

/* ========================================================================== */
/*                              Private: Masks                                 */
/* ========================================================================== */
/*
 * @brief Bit-mask of **writable** bits for every I2C register.
 *        A clear bit (0) marks a *reserved* position that must not be written.
 *        Index: ::I2C_Reg_t
 */
static const uint32_t I2C_validBits[I2C_REG_COUNT] = {
    [I2C_CR1]   = ~((0b1u << 2) | (0b1u << 14)),
    [I2C_CR2]   = ~((0b11u << 6) | (0b111u << 13)),
    [I2C_OAR1]  = ~(0b1111u << 10),
    [I2C_OAR2]  = ~(0b11111111u << 8),
    [I2C_DR]    = ~(0b11111111u << 8),
    [I2C_SR1]   = ~((0b1u << 5) | (0b1u << 13)),
    [I2C_SR2]   = ~(0b1u << 3),
    [I2C_CCR]   = ~(0b11u << 12),
    [I2C_TRISE] = ~(0b1111111111u << 6),
    [I2C_FLTR]  = ~(0b11111111111u << 5)
};

/* ========================================================================== */
/*                       Private: Register Lookup Tables                       */
/* ========================================================================== */
/* Map ::I2C_Reg_t indices to the actual peripheral registers (I2C1..I2C3). */
static volatile uint32_t* I2C1_regLookUpTable[I2C_REG_COUNT] = {
    [I2C_CR1]   = I2C1_GET_REG(I2C_CR1),
    [I2C_CR2]   = I2C1_GET_REG(I2C_CR2),
    [I2C_OAR1]  = I2C1_GET_REG(I2C_OAR1),
    [I2C_OAR2]  = I2C1_GET_REG(I2C_OAR2),
    [I2C_DR]    = I2C1_GET_REG(I2C_DR),
    [I2C_SR1]   = I2C1_GET_REG(I2C_SR1),
    [I2C_SR2]   = I2C1_GET_REG(I2C_SR2),
    [I2C_CCR]   = I2C1_GET_REG(I2C_CCR),
    [I2C_TRISE] = I2C1_GET_REG(I2C_TRISE),
    [I2C_FLTR]  = I2C1_GET_REG(I2C_FLTR)
};

static volatile uint32_t* I2C2_regLookUpTable[I2C_REG_COUNT] = {
    [I2C_CR1]   = I2C2_GET_REG(I2C_CR1),
    [I2C_CR2]   = I2C2_GET_REG(I2C_CR2),
    [I2C_OAR1]  = I2C2_GET_REG(I2C_OAR1),
    [I2C_OAR2]  = I2C2_GET_REG(I2C_OAR2),
    [I2C_DR]    = I2C2_GET_REG(I2C_DR),
    [I2C_SR1]   = I2C2_GET_REG(I2C_SR1),
    [I2C_SR2]   = I2C2_GET_REG(I2C_SR2),
    [I2C_CCR]   = I2C2_GET_REG(I2C_CCR),
    [I2C_TRISE] = I2C2_GET_REG(I2C_TRISE),
    [I2C_FLTR]  = I2C2_GET_REG(I2C_FLTR)
};

static volatile uint32_t* I2C3_regLookUpTable[I2C_REG_COUNT] = {
    [I2C_CR1]   = I2C3_GET_REG(I2C_CR1),
    [I2C_CR2]   = I2C3_GET_REG(I2C_CR2),
    [I2C_OAR1]  = I2C3_GET_REG(I2C_OAR1),
    [I2C_OAR2]  = I2C3_GET_REG(I2C_OAR2),
    [I2C_DR]    = I2C3_GET_REG(I2C_DR),
    [I2C_SR1]   = I2C3_GET_REG(I2C_SR1),
    [I2C_SR2]   = I2C3_GET_REG(I2C_SR2),
    [I2C_CCR]   = I2C3_GET_REG(I2C_CCR),
    [I2C_TRISE] = I2C3_GET_REG(I2C_TRISE),
    [I2C_FLTR]  = I2C3_GET_REG(I2C_FLTR)
};

/* ========================================================================== */
/*                              Private Helpers                                */
/* ========================================================================== */

/*
 * @brief Enable RCC clock for the selected I2C peripheral.
 */
static bool I2C_clockEnable(I2C_Name_t I2Cx) {
    switch (I2Cx) {
        case _I2C1: __HAL_RCC_I2C1_CLK_ENABLE(); return true;
        case _I2C2: __HAL_RCC_I2C2_CLK_ENABLE(); return true;
        case _I2C3: __HAL_RCC_I2C3_CLK_ENABLE(); return true;
        default:    return false;
    }
}

/*
 * @brief Validate that a field (bitPosition..bitWidth) is writable for @p regName.
 *
 * @return true if the entire field is writable (no reserved bits), false otherwise.
 */
static inline bool isValidI2CBit(uint8_t bitPosition, uint8_t bitWidth, I2C_Reg_t regName) {
    if ((regName >= I2C_REG_COUNT) ||
        (bitWidth == 0u) ||
        ((bitPosition + bitWidth) > 32u) ||
        (bitPosition > 31u) ||
        (bitWidth > 32u)) {
        return false;
    }

    const uint32_t mask = (bitWidth == 32u)
                        ? 0xFFFFFFFFu
                        : ((1u << bitWidth) - 1u) << bitPosition;
    return (I2C_validBits[regName] & mask) == mask;
}

/*
 * @brief Generic masked write helper (read-modify-write a field).
 */
static bool writeI2CBits(volatile uint32_t* reg,
                         uint8_t bitPosition,
                         uint8_t bitWidth,
                         uint32_t value) {
    if ((bitWidth < 32u) && (value > ((1u << bitWidth) - 1u))) return false;

    const uint32_t mask         = (bitWidth == 32u) ? 0xFFFFFFFFu : ((1u << bitWidth) - 1u) << bitPosition;
    const uint32_t shiftedValue = value << bitPosition;
    *reg = (*reg & ~mask) | (shiftedValue & mask);
    return true;
}

/*
 * @brief Read a field of @p bitWidth bits from a register starting at @p bitPosition.
 */
static uint32_t readI2CBits(volatile uint32_t* reg,
                            uint8_t bitPosition,
                            uint8_t bitWidth) {
    if (bitWidth == 32u) return *reg;
    const uint32_t mask = (1u << bitWidth) - 1u;
    return (*reg >> bitPosition) & mask;
}

/* ========================================================================== */
/*                              R/W Front-Ends                                 */
/* ========================================================================== */

/*
 * @brief Write a bit-field to an I2C peripheral register.
 *        Determines the field width from @p regName + @p bitPosition.
 */
bool writeI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t value) {
    uint8_t bitWidth = 1u;

    switch (regName) {
        case I2C_CR1:
            break;

        case I2C_CR2:
            if (bitPosition == 0u) bitWidth = 6u;
            break;

        case I2C_OAR1:
            if (bitPosition == 1u)      bitWidth = 7u;
            else if (bitPosition == 8u) bitWidth = 2u;
            break;

        case I2C_OAR2:
            if (bitPosition == 1u) bitWidth = 7u;
            break;

        case I2C_DR:
            bitWidth = 8u;
            break;

        case I2C_SR1:
            if (bitPosition <= 7u) return false;
            break;

        case I2C_SR2:
            return false;

        case I2C_CCR:
            if (bitPosition == 0u) bitWidth = 12u;
            break;

        case I2C_TRISE:
            bitWidth = 6u;
            break;

        case I2C_FLTR:
            if (bitPosition == 0u) bitWidth = 4u;
            break;

        default:
            return false;
    }

    if (!isValidI2CBit(bitPosition, bitWidth, regName)) return false;

    volatile uint32_t* regAddr = NULL;
    switch (i2cBus) {
        case _I2C1: regAddr = I2C1_regLookUpTable[regName]; break;
        case _I2C2: regAddr = I2C2_regLookUpTable[regName]; break;
        case _I2C3: regAddr = I2C3_regLookUpTable[regName]; break;
        default:    return false;
    }

    (void)writeI2CBits(regAddr, bitPosition, bitWidth, value);
    return true;
}

/*
 * @brief Read a bit/field from an I2C peripheral register.
 * @return Extracted field value on success, or I2C_VALUE_ERROR on failure.
 */
uint32_t readI2C(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition) {
    uint8_t bitWidth = 1u;

    switch (regName) {
        case I2C_CR1:
            bitWidth = 1u;
            break;

        case I2C_CR2:
            if (bitPosition == 0u) bitWidth = 6u;
            break;

        case I2C_OAR1:
            if (bitPosition == 1u)      bitWidth = 7u;
            else if (bitPosition == 8u) bitWidth = 2u;
            break;

        case I2C_OAR2:
            if (bitPosition == 1u) bitWidth = 7u;
            break;

        case I2C_DR:
            bitWidth = 8u;
            break;

        case I2C_SR1:
            bitWidth = 1u;
            break;

        case I2C_SR2:
            if (bitPosition == 8u) bitWidth = 8u;
            break;

        case I2C_CCR:
            if (bitPosition == 0u) bitWidth = 12u;
            break;

        case I2C_TRISE:
            bitWidth = 6u;
            break;

        case I2C_FLTR:
            if (bitPosition == 0u) bitWidth = 4u;
            break;

        default:
            return I2C_VALUE_ERROR;
    }

    if (!isValidI2CBit(bitPosition, bitWidth, regName)) return I2C_VALUE_ERROR;

    volatile uint32_t* reg = NULL;
    switch (i2cBus) {
        case _I2C1: reg = I2C1_regLookUpTable[regName]; break;
        case _I2C2: reg = I2C2_regLookUpTable[regName]; break;
        case _I2C3: reg = I2C3_regLookUpTable[regName]; break;
        default:    return I2C_VALUE_ERROR;
    }

    return readI2CBits(reg, bitPosition, bitWidth);
}

/* ========================================================================== */
/*                            Busy-Wait Utilities                              */
/* ========================================================================== */

/*
 * @brief Wait until a specific bit becomes 1.
 */
static inline bool I2C_waitBitSet(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t timeout) {
    while ((timeout--) != 0u) {
        const uint32_t result = readI2C(i2cBus, regName, bitPosition);
        if (result == I2C_VALUE_ERROR) return false;
        if (result == 1u) return true;
    }
    return false;
}

/*
 * @brief Wait until a specific bit becomes 0.
 */
static inline bool I2C_waitBitClear(I2C_Name_t i2cBus, I2C_Reg_t regName, uint8_t bitPosition, uint32_t timeout) {
    while ((timeout--) != 0u) {
        const uint32_t result = readI2C(i2cBus, regName, bitPosition);
        if (result == I2C_VALUE_ERROR) return false;
        if (result == 0u) return true;
    }
    return false;
}

/* ========================================================================== */
/*                         Pin Initialization Helpers                          */
/* ========================================================================== */

bool I2C_sclPinInit(GPIO_Pin_t sclPin, GPIO_PortName_t sclPort, I2C_Name_t i2cBus) {
    switch (i2cBus) {
        case _I2C1:
            if (!((sclPort == my_GPIOB) && ((sclPin == my_GPIO_PIN_6) || (sclPin == my_GPIO_PIN_8)))) {
                return false;
            }
            break;

        case _I2C2:
            if (!((sclPort == my_GPIOB) && (sclPin == my_GPIO_PIN_10))) {
                return false;
            }
            break;

        case _I2C3:
            if (!((sclPort == my_GPIOA) && (sclPin == my_GPIO_PIN_8))) {
                return false;
            }
            break;

        default:
            return false;
    }

    const GPIO_Mode_t afrReg       = (sclPin < my_GPIO_PIN_8) ? AFRL : AFRH;
    const bool        hasExtPullUp = ((sclPin == my_GPIO_PIN_6) && (sclPort == my_GPIOB));

    /* Configure SCL */
    Enable_GPIO_Clock(sclPort);
    writePin(sclPin, sclPort, MODER,   AF_MODE);     /* Alternate Function        */
    writePin(sclPin, sclPort, OTYPER,  OPEN_DRAIN);  /* Open-drain (pull-up req.) */
    writePin(sclPin, sclPort, OSPEEDR, HIGH_SPEED);  /* Very high speed           */
    writePin(sclPin, sclPort, PUPDR,   hasExtPullUp ? FLOATING : PULL_UP);
    writePin(sclPin, sclPort, afrReg,  AF4);

    return true;
}

bool I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus) {
    uint8_t alternateFuncMode = 0xFFu;

    switch (i2cBus) {
        case _I2C1:
            if ((sdaPort == my_GPIOB) && ((sdaPin == my_GPIO_PIN_7) || (sdaPin == my_GPIO_PIN_9))) {
                alternateFuncMode = AF4;
            } else return false;
            break;

        case _I2C2:
            if ((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_11)) {
                alternateFuncMode = AF4;
            } else if ((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_3)) {
                alternateFuncMode = AF9;
            } else return false;
            break;

        case _I2C3:
            if ((sdaPort == my_GPIOC) && (sdaPin == my_GPIO_PIN_9)) {
                alternateFuncMode = AF4;
            } else if (((sdaPin == my_GPIO_PIN_4) && (sdaPort == my_GPIOB)) ||
                       ((sdaPin == my_GPIO_PIN_8) && (sdaPort == my_GPIOB))) {
                alternateFuncMode = AF9;
            } else return false;
            break;

        default:
            return false;
    }

    const GPIO_Mode_t afrReg       = (sdaPin < my_GPIO_PIN_8) ? AFRL : AFRH;
    const bool        hasExtPullUp = (sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_9);

    /* Configure SDA */
    Enable_GPIO_Clock(sdaPort);
    writePin(sdaPin, sdaPort, MODER,   AF_MODE);     /* Alternate Function        */
    writePin(sdaPin, sdaPort, OTYPER,  OPEN_DRAIN);  /* Open-drain (pull-up req.) */
    writePin(sdaPin, sdaPort, OSPEEDR, HIGH_SPEED);  /* Very high speed           */
    writePin(sdaPin, sdaPort, PUPDR,   hasExtPullUp ? FLOATING : PULL_UP);
    writePin(sdaPin, sdaPort, afrReg,  alternateFuncMode);

    return true;
}

/**
 * @brief Init SCL and SDA pins.
 */
bool I2C_GPIO_init(I2C_GPIO_Config_t config) {
    if (config.i2cBus >= I2C_count) {
        return false;
    }

    const bool sclInitOk = I2C_sclPinInit(config.sclPin, config.sclPort, config.i2cBus);
    const bool sdaInitOk = I2C_sdaPinInit(config.sdaPin, config.sdaPort, config.i2cBus);

    return (sclInitOk && sdaInitOk);
}

/* ========================================================================== */
/*                         Timing / Frequency Helpers                          */
/* ========================================================================== */

/*
 * @brief Configure CCR[11:0] (and DUTY/F-S) for desired SCL and program TRISE.
 *
 * @param ccrMode  I2C speed/duty mode.
 * @param sclFreq  Desired SCL frequency in Hz (e.g., 100k or 400k).
 * @param fclk1    APB1 peripheral clock feeding I2C (Hz).
 * @param config   Bus/pins configuration (for bus selection).
 */
bool I2C_getCCR(I2C_CCR_Mode_t ccrMode,
                uint32_t        sclFreq,
                uint32_t        fclk1,
                I2C_GPIO_Config_t config) {

    if ((sclFreq == 0u) && (fclk1 == 0u)) return false;
    if ((ccrMode == I2C_SM_MODE) && (fclk1 < 2000000u)) return false;
    if ((ccrMode != I2C_SM_MODE) && (fclk1 < 4000000u)) return false;

    uint32_t ccrVal      = 0u;
    uint32_t denominator = 0u;
    uint8_t  fsMode      = 0u;
    uint8_t  dutyMode    = 0u;

    switch (ccrMode) {
        case I2C_SM_MODE:
            denominator = 2u;
            break;

        case I2C_FM_MODE_2LOW_1HIGH:
            denominator = 3u;
            fsMode = 1u;
            break;

        case I2C_FM_MODE_16LOW_9HIGH:
            denominator = 25u;
            fsMode = 1u;
            dutyMode = 1u;
            break;

        default:
            return false;
    }

    /* Ceil division so actual sclFreq <= requested */
    ccrVal = (fclk1 + (denominator * sclFreq - 1u)) / (denominator * sclFreq);

    /* Clamp to legal range for the chosen mode */
    const uint8_t minCCR = (fsMode && dutyMode) ? 1u : 4u;
    if (ccrVal < minCCR) ccrVal = minCCR;
    if (ccrVal > 0xFFFu) ccrVal = 0xFFFu; /* 12-bit field */

    /* PE must be disabled before CCR/TRISE writes */
    volatile uint32_t* cr1 = (config.i2cBus == _I2C1) ? I2C1_GET_REG(I2C_CR1) :
                             (config.i2cBus == _I2C2) ? I2C2_GET_REG(I2C_CR1) :
                                                        I2C3_GET_REG(I2C_CR1);
    if ((*cr1) & (1u << CR1_PE_Pos)) return false; /* PE set -> not allowed */

    /* Program CCR[11:0], DUTY, and F/S */
    if (!writeI2C(config.i2cBus, I2C_CCR, I2C_CCR_Pos,  ccrVal))   return false;
    if (!writeI2C(config.i2cBus, I2C_CCR, I2C_DUTY_Pos, dutyMode)) return false;
    if (!writeI2C(config.i2cBus, I2C_CCR, I2C_FS_Pos,   fsMode))   return false;

    /* Program TRISE */
    const uint32_t fclk1_MHz = (fclk1 + 500000u) / 1000000u; /* round-to-nearest MHz */
    uint32_t       trise     = (!fsMode) ? (fclk1_MHz + 1u)
                                         : ((fclk1_MHz * 300u) / (1000u + 1u));
    if (trise > 63u) trise = 63u;

    if (!writeI2C(config.i2cBus, I2C_TRISE, 0u, trise)) return false;
    return true;
}

/* ========================================================================== */
/*                           Register-Level I/O                                */
/* ========================================================================== */

/*
 * @brief Write one byte to a register of a 7-bit I2C slave.
 *
 * Sequence:
 *  1) Wait BUSY=0
 *  2) START
 *  3) Send <SlaveAddr | Write>
 *  4) Abort on NACK
 *  5) Send sub-register address
 *  6) Send data byte
 *  7) STOP
 */
bool I2C_writeReg8(I2C_GPIO_Config_t config,
                   uint8_t           slaveAddr,
                   uint8_t           slaveRegAddr,
                   uint8_t           value) {
    const I2C_Name_t i2cBus  = config.i2cBus;
    uint32_t         timeout = I2C_TIMEOUT_SHORT_US;

    /* Wait until the bus is free */
    if (!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout)) return false;

    /* START condition */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) return false;

    /* Send 7-bit address + write bit (0) */
    const uint8_t writeMode = 0u;
    const uint8_t addrByte  = (uint8_t)((slaveAddr << 1) | (writeMode << 0));
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrByte)) return false;

    /* Check address phase; handle NACK */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) {
        if ((readI2C(i2cBus, I2C_SR1, SR1_AF_Pos) & 1u) == 1u) {
            (void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);   /* STOP */
            (void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos,  RESET);  /* Clear AF */
        }
        return false;
    }

    /* Clear ADDR by SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR1, 0u);
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Send sub-register address */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
    if (!writeI2C(i2cBus, I2C_DR, 0u, slaveRegAddr))            return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false;

    /* Send data byte */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
    if (!writeI2C(i2cBus, I2C_DR, 0u, value))                   return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false;

    /* STOP */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;
    return true;
}

/**
 * @brief Read one byte from an I2C 8-bit sub-register.
 *
 * Sequence:
 *  1) Wait BUSY=0
 *  2) START
 *  3) Send <addr|write>, wait ADDR=1, clear ADDR
 *  4) Send sub-reg, wait BTF=1
 *  5) Repeated START
 *  6) Send <addr|read>, wait ADDR=1, clear ADDR
 *  7) Wait RXNE=1, read DR
 *  8) STOP
 */
bool I2C_readReg8(I2C_GPIO_Config_t config,
                  uint8_t           slaveAddr,
                  uint8_t           slaveRegAddr,
                  uint8_t*          outResult) {
    const I2C_Name_t i2cBus = config.i2cBus;
    const uint32_t   timeout = I2C_TIMEOUT_SHORT_US;

    /* Ensure bus is idle */
    if (!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout));

    /* START then wait for SB */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) return false;

    /* Send <addr | write>, then wait ADDR=1 */
    const uint8_t addrWrite = (uint8_t)((slaveAddr << 1) | 0u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrWrite)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;

    /* Clear ADDR by completing SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Confirm no NACK (AF=0) */
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false;

    /* Send sub-register index; wait BTF */
    if (!writeI2C(i2cBus, I2C_DR, 0u, slaveRegAddr))            return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;

    /* Repeated START; wait SB */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET))         return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout))  return false;

    /* Send <addr | read>, then wait ADDR=1 */
    const uint8_t addrRead = (uint8_t)((slaveAddr << 1) | 1u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrRead))                return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;

    /* Clear ADDR (finish by reading SR2) */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Confirm no NACK (AF=0) */
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)) return false;

    /* Wait RXNE=1 then read DR */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_RXNE_Pos, timeout)) return false;
    *outResult = (uint8_t)readI2C(i2cBus, I2C_DR, 0u);

    /* STOP to release bus */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;

    return true;
}

/* ========================================================================== */
/*                               Initialization                                */
/* ========================================================================== */

/* Default timing parameters */
I2C_Timing_t timingParam = {
    .ccrMode  = I2C_SM_MODE,        /* Standard mode                     */
    .sclFreq  = 100000u,            /* 100 kHz                           */
    .fclk1_Hz = SYSCLK_FREQ_16M     /* Default APB1/sys clock, from HAL  */
};

/* Apply timing to the selected bus:
 * - CR2.FREQ = fclk1 (MHz)
 * - Disable PE, program CCR/DUTY/FS and TRISE, then re-enable PE
 */
static void I2C_getFreq(I2C_GPIO_Config_t config,
                        I2C_CCR_Mode_t    ccrMode,
                        uint32_t          sclFreq,
                        uint32_t          fclk1_Hz) {
    /* Program CR2.FREQ = FCLK1 (MHz, rounded) */
    const uint32_t fclk1_MHz = (fclk1_Hz + 500000u) / 1000000u;
    (void)writeI2C(config.i2cBus, I2C_CR2, CR2_FREQ_Pos, fclk1_MHz);

    /* Disable PE before programming CCR/TRISE */
    (void)writeI2C(config.i2cBus, I2C_CR1, CR1_PE_Pos, RESET);

    /* Configure CCR/TRISE */
    if (!I2C_getCCR(ccrMode, sclFreq, fclk1_Hz, config)) return;

    /* Re-enable PE */
    (void)writeI2C(config.i2cBus, I2C_CR1, CR1_PE_Pos, SET);
}

void I2C_basicConfigInit(I2C_GPIO_Config_t config) {
    const I2C_Name_t i2cBus = config.i2cBus;

    (void)I2C_clockEnable(i2cBus);   /* Enable I2C clock */
    if (!I2C_GPIO_init(config)) return;

    I2C_getFreq(config,
                timingParam.ccrMode,
                timingParam.sclFreq,
                timingParam.fclk1_Hz);
}
