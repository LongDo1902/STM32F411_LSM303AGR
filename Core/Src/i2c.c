/*
 * @file	i2c.c
 *
 *  Created on: Aug 20, 2025
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
 * ===============================================================================
 * Private Helpers
 * ===============================================================================
 */

/*
 * @brief	Bit-mask of **writable** bits for every I2C reg
 * 			A clear bit (0) marks a *reserved* position that **must not** be written.
 *
 * 			Index:  ::I2C_Reg_t
 */
static const uint32_t I2C_validBits[I2C_REG_COUNT] = {
		[I2C_CR1] = ~((0b1u << 2) | (0b1 << 14)),
		[I2C_CR2] = ~((0b11u << 6) | (0b111 << 13)),
		[I2C_OAR1] = ~(0b1111u << 10),
		[I2C_OAR2] = ~(0b11111111u << 8),
		[I2C_DR] = ~(0b11111111u << 8),
		[I2C_SR1] = ~((0b1u << 5) | (0b1u << 13)),
		[I2C_SR2] = ~(0b1u << 3),
		[I2C_CCR] = ~(0b11u << 12),
		[I2C_TRISE] = ~(0b1111111111u << 6),
		[I2C_FLTR] = ~(0b11111111111u << 5)
};

/*
 * @brief	Lookup table for the I2Cx peripheral register
 * 			This static pointer array maps each value in I2C_Reg_t (index) to the
 * 			corresponding I2Cx Register.
 * 			Using lookup table avoiding 'switch' cases and make register access simply.
 */
static volatile uint32_t* I2C1_regLookUpTable[I2C_REG_COUNT] = {
		[I2C_CR1] = I2C1_GET_REG(I2C_CR1),
		[I2C_CR2] = I2C1_GET_REG(I2C_CR2),
		[I2C_OAR1] = I2C1_GET_REG(I2C_OAR1),
		[I2C_OAR2] = I2C1_GET_REG(I2C_OAR2),
		[I2C_DR] = I2C1_GET_REG(I2C_DR),
		[I2C_SR1] = I2C1_GET_REG(I2C_SR1),
		[I2C_SR2] = I2C1_GET_REG(I2C_SR2),
		[I2C_CCR] = I2C1_GET_REG(I2C_CCR),
		[I2C_TRISE] = I2C1_GET_REG(I2C_TRISE),
		[I2C_FLTR] = I2C1_GET_REG(I2C_FLTR)
};

static volatile uint32_t* I2C2_regLookUpTable[I2C_REG_COUNT] = {
		[I2C_CR1] = I2C2_GET_REG(I2C_CR1),
		[I2C_CR2] = I2C2_GET_REG(I2C_CR2),
		[I2C_OAR1] = I2C2_GET_REG(I2C_OAR1),
		[I2C_OAR2] = I2C2_GET_REG(I2C_OAR2),
		[I2C_DR] = I2C2_GET_REG(I2C_DR),
		[I2C_SR1] = I2C2_GET_REG(I2C_SR1),
		[I2C_SR2] = I2C2_GET_REG(I2C_SR2),
		[I2C_CCR] = I2C2_GET_REG(I2C_CCR),
		[I2C_TRISE] = I2C2_GET_REG(I2C_TRISE),
		[I2C_FLTR] = I2C2_GET_REG(I2C_FLTR)
};

static volatile uint32_t* I2C3_regLookUpTable[I2C_REG_COUNT] = {
		[I2C_CR1] = I2C3_GET_REG(I2C_CR1),
		[I2C_CR2] = I2C3_GET_REG(I2C_CR2),
		[I2C_OAR1] = I2C3_GET_REG(I2C_OAR1),
		[I2C_OAR2] = I2C3_GET_REG(I2C_OAR2),
		[I2C_DR] = I2C3_GET_REG(I2C_DR),
		[I2C_SR1] = I2C3_GET_REG(I2C_SR1),
		[I2C_SR2] = I2C3_GET_REG(I2C_SR2),
		[I2C_CCR] = I2C3_GET_REG(I2C_CCR),
		[I2C_TRISE] = I2C3_GET_REG(I2C_TRISE),
		[I2C_FLTR] = I2C3_GET_REG(I2C_FLTR)
};

/*
 * @brief	Flexibly enable RCC for I2Cx
 */
static bool I2C_clockEnable(I2C_Name_t I2Cx){
	switch(I2Cx){
		case _I2C1: __HAL_RCC_I2C1_CLK_ENABLE(); return true;
		case _I2C2: __HAL_RCC_I2C2_CLK_ENABLE(); return true;
		case _I2C3: __HAL_RCC_I2C3_CLK_ENABLE(); return true;
		default: return false;
	}
}

/*
 * @brief	Check that 'reg' is in range and 'bitPosition' is not reserved
 * 			This function flexibly check if multi-bits are valid
 *
 * @return	true	Inserted bit indice is valid
 * 			false	...invalid or reserved bit
 */
static inline bool isValidI2CBit(uint8_t bitPosition, uint8_t bitWidth, I2C_Reg_t reg){
	if((reg >= I2C_REG_COUNT) ||
	   (bitWidth == 0) ||
	   ((bitPosition + bitWidth) > 32) ||
	   (bitPosition > 31) ||
	   (bitWidth > 32)) return false;

	uint32_t mask = ((bitWidth == 32) ? 0xFFFFFFFFu : ((1u << bitWidth) - 1u) << bitPosition);
	return(I2C_validBits[reg] & mask) == mask;
}

/*
 * @brief	Generic masked write helper
 *
 * @param	reg		Pointers to the register
 * @param	bitPosition		First bit of the field
 * @param	bitWidth		Field width in bits
 * @param	value			Field value (must fit in @p bitWidth)
 */
static bool writeI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth, uint32_t value){
	if((bitWidth < 32) && (value > ((1U << bitWidth) - 1U))) return false;

	uint32_t mask = (bitWidth == 32) ? 0xFFFFFFFFu : ((1U << bitWidth) - 1) << bitPosition;
	uint32_t shiftedValue = value << bitPosition;
	*reg = (*reg & ~mask) | (shiftedValue & mask);
	return true; //Write process successfully
}

/*
 * @brief	Read a field of "bitWidth" bits from a register starting from "bitPosition"
 *
 * @param	reg		Pointer to the register
 * @param	bitPosition		First bit of the field
 * @param	bitWidth		Field width
 */
static uint32_t readI2CBits(volatile uint32_t* reg, uint8_t bitPosition, uint8_t bitWidth){
	if(bitWidth == 32) return *reg;

	uint32_t mask = ((1U << bitWidth) - 1U);
	return(*reg >> bitPosition) & mask;
}

/*
 * @brief	Write a bit-field to an I2C peripheral register
 * 			Figures how many bits the setting needs
 * 			Changes only the bis you asked for, leaving the rest unchanged
 */
bool writeI2C(uint8_t bitPosition, I2C_Name_t i2cBus, I2C_Reg_t regName, uint32_t value){
	uint8_t bitWidth = 1;
	switch(regName){
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

		case I2C_SR2: return false;

		case I2C_CCR:
			if(bitPosition == 0) bitWidth = 12;
			break;

		case I2C_TRISE:
			bitWidth = 6;
			break;

		case I2C_FLTR:
			if(bitPosition == 0) bitWidth = 4;
			break;

		default: return false;
	}

	if(!isValidI2CBit(bitPosition, bitWidth, regName)) return false;

	volatile uint32_t* regAddr = NULL;

	switch(i2cBus){
		case _I2C1:
			regAddr = I2C1_regLookUpTable[regName];
			break;

		case _I2C2:
			regAddr = I2C2_regLookUpTable[regName];
			break;

		case _I2C3:
			regAddr = I2C3_regLookUpTable[regName];
			break;

		default: return false;
	}

	if((regName == I2C_SR1) && (bitPosition < 8)) return false; //These bits are read only

	writeI2CBits(regAddr, bitPosition, bitWidth, value);
	return true;
}


/*
 * ======================================================
 * Pin Initialization Helpers (SCL and SDA)
 * ======================================================
 */
bool I2C_sclPinInit(GPIO_Pin_t sclPin, GPIO_PortName_t sclPort, I2C_Name_t i2cBus){
	switch(i2cBus){
		case _I2C1:
			if(!((sclPort == my_GPIOB) && ((sclPin == my_GPIO_PIN_6) || (sclPin == my_GPIO_PIN_8)))){
				return false;
			}
			break;

		case _I2C2:
			if(!((sclPort == my_GPIOB) && (sclPin == my_GPIO_PIN_10))){
				return false;
			}
			break;

		case _I2C3:
			if(!((sclPort == my_GPIOA) && (sclPin == my_GPIO_PIN_8))){
				return false;
			}
			break;

		default: return false;
	}
	const GPIO_Mode_t afrReg = (sclPin < my_GPIO_PIN_8) ? AFRL : AFRH;
	const bool hasExternalPullUp = ((sclPin == my_GPIO_PIN_6) && (sclPort == my_GPIOB));

	/* Config SCL pin for I2C purpose */
	Enable_GPIO_Clock(sclPort);
	writePin(sclPin, sclPort, MODER, AF_MODE); //Set pin to Alternate Function mode
	writePin(sclPin, sclPort, OTYPER, OPEN_DRAIN); //Set pin to Open-Drain mode - can only pull down, need pull up to go high
	writePin(sclPin, sclPort, OSPEEDR, HIGH_SPEED); //Set outpin works at very high speed
	writePin(sclPin, sclPort, PUPDR, hasExternalPullUp ? FLOATING : PULL_UP);
	writePin(sclPin, sclPort, afrReg, AF4);

	return true;
}


bool I2C_sdaPinInit(GPIO_Pin_t sdaPin, GPIO_PortName_t sdaPort, I2C_Name_t i2cBus){
	uint8_t alternateFuncMode = 0xFF;

	switch(i2cBus){
		case _I2C1:
			if((sdaPort == my_GPIOB) && ((sdaPin == my_GPIO_PIN_7) || (sdaPin == my_GPIO_PIN_9))){
				alternateFuncMode = AF4;
			} else return false;
			break;

		case _I2C2:
			if((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_11)){
				alternateFuncMode = AF4;
			}
			else if((sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_3)){
				alternateFuncMode = AF9;
			} else return false;
			break;

		case _I2C3:
			if((sdaPort == my_GPIOC) && (sdaPin == my_GPIO_PIN_9)){
				alternateFuncMode = AF4;
			} else if(((sdaPin == my_GPIO_PIN_4) && (sdaPort == my_GPIOB)) ||
					((sdaPin == my_GPIO_PIN_8) && (sdaPort == my_GPIOB))){
				alternateFuncMode = AF9;
			} else return false;
			break;

		default: return false;
	}

	/* Config SDA GPIO for I2C purpose */
	const GPIO_Mode_t afrReg = (sdaPin < my_GPIO_PIN_8) ? AFRL : AFRH;
	const bool hasExternalPullUp = (sdaPort == my_GPIOB) && (sdaPin == my_GPIO_PIN_9);

	Enable_GPIO_Clock(sdaPort);
	writePin(sdaPin, sdaPort, MODER, AF_MODE); //Set alternate function mode for the pin
	writePin(sdaPin, sdaPort, OTYPER, OPEN_DRAIN); //Set pin Open-Drain - can only pull down, need pull up to go high
	writePin(sdaPin, sdaPort, OSPEEDR, HIGH_SPEED); //Set the pinOut a very high speed
	writePin(sdaPin, sdaPort, PUPDR, hasExternalPullUp ? FLOATING : PULL_UP); //0b00 = floating, 0b01 = internal pull up resitor
	writePin(sdaPin, sdaPort, afrReg, alternateFuncMode);

	return true;
}

bool I2C_GPIO_init(I2C_GPIO_Config_t config){
	if(config.i2cBus >= I2C_count){
		return false;
	}
	/* Initi scl and sda pins */
	bool sclInitOk = I2C_sclPinInit(config.sclPin, config.sclPort, config.i2cBus);
	bool sdaInitOk = I2C_sdaPinInit(config.sdaPin, config.sdaPort, config.i2cBus);

	return sclInitOk && sdaInitOk;
}

/*
 * @brief	Config and assign a proper value for I2C_CCR[11:0] bit-field
 * 			I2C_CCR is used for configuring speed/frequency of SCL pin
 *
 * @param	sclFreq		Desired SCL frequency in Hz (100K or 400K)
 * @param	fclk1		Peripheral Clock Freq that feeds into I2C bus
 * @param	config		Structure holds I2C pins and I2C bus
 */
bool I2C_getCCR(I2C_CCR_Mode_t ccrMode,
				uint32_t sclFreq,
				uint32_t fclk1,
				I2C_GPIO_Config_t config){

	if((sclFreq == 0) && (fclk1 == 0)) return false;
	if((ccrMode == I2C_SM_MODE) && (fclk1 < 2000000u)) return false;
	if((ccrMode != I2C_SM_MODE) && (fclk1 < 4000000u)) return false;

	uint32_t ccrVal = 0;
	uint32_t denominator = 0;
	uint8_t fsMode = 0;
	uint8_t dutyMode = 0;

	switch(ccrMode){
		case I2C_SM_MODE:
			denominator = 2u;
			break;

		case I2C_FM_MODE_2LOW_1HIGH:
			denominator = 3u;
			fsMode = 1;
			break;

		case I2C_FM_MODE_16LOW_9HIGH:
			denominator = 25u;
			fsMode = 1;
			dutyMode = 1;
			break;

		default: return false;
	}

	/* Ceil division so actual sclFreq <= requested */
	ccrVal = (fclk1 + (denominator*sclFreq - 1)) / denominator*sclFreq;

	/* Clamp to legal range for the chosen mode */
	uint8_t minCCR = (fsMode && dutyMode) ? 1u : 4u;
	if(ccrVal < minCCR) ccrVal = minCCR;
	if(ccrVal > 0xFFFU) ccrVal = 0xFFFU; //12 bit-field

	//PE must be disable before CCR/TRISE writes
	volatile uint32_t* cr1 = (config.i2cBus == _I2C1) ? I2C1_GET_REG(I2C_CR1) :
							 (config.i2cBus == _I2C2) ? I2C2_GET_REG(I2C_CR1) :
									 	 	 	 	 	I2C3_GET_REG(I2C_CR1);
	if((*cr1) & 0x1u) return false; //If the PE is set, return false

	/* Program CCR[11:0], DUTY and F/S */
	if(!writeI2C(0, config.i2cBus, I2C_CCR, ccrVal)) return false;
	if(!writeI2C(14, config.i2cBus, I2C_CCR, dutyMode)) return false;
	if(!writeI2C(15, config.i2cBus, I2C_CCR, fsMode)) return false;

	//Program TRISE
	uint32_t pclk1_MHz = (fclk1 + 500000U) / 1000000U;
	uint32_t trise = (!fsMode) ? (pclk1_MHz + 1) : ((fclk1 * 300U + 999U)/(1000U + 1u));
	if(trise > 63) trise = 63;

	if(!writeI2C(0, config.i2cBus, I2C_TRISE, trise)) return false;
	return true;
}















