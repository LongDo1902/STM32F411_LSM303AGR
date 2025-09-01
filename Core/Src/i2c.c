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
 * @brief Write a single byte to an 8-bit sub-register of a 7-bit I2C slave.
 *
 * Sequence:
 * 		Wait until BUSY = 0
 * 		START
 * 		Send <SlaveAddr|W>, wait ADDR = 1 (Abort on AF/NACK)
 * 		Clear ADDR by reading SR1 then SR2
 * 		Send sub-register address (SUB) and wait for TXE/BTF (Abort on AF/NACK)
 * 		Send data byte, wait TXE/BTF (Abort on AF/NACK)
 * 		STOP
 *
 * @param	config			I2C bus/pin selection
 * @param	slaveAddr		7-bit UNSHIFTED slave device address
 * @param	slaveRegAddr	8-bit sub-register address written to (SUB)
 * @param	value			Single data byte to write
 *
 * @Notes: This helper does not send multiple bytes. Please use I2C_writeBurst to send multiple bytes
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
    const uint8_t addrByte  = (uint8_t)((slaveAddr << 1) | 0u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrByte)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Clear ADDR by SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Send sub-register address; wait BTF */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
    if (!writeI2C(i2cBus, I2C_DR, 0u, slaveRegAddr))            return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Send data byte */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
    if (!writeI2C(i2cBus, I2C_DR, 0u, value))                   return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* STOP */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;
    return true;
}

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
                    uint8_t*          dataBuf,
					uint16_t		  len) {

	if((len == 0u) | (dataBuf == NULL)) return false;

    const I2C_Name_t i2cBus  = config.i2cBus;
    uint32_t         timeout = I2C_TIMEOUT_SHORT_US;

    /* Wait until the bus is free */
    if (!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout)) return false;

    /* START condition */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) return false;

    /* Send 7-bit address + write bit (0) */
    const uint8_t addrByte  = (uint8_t)((slaveAddr << 1) | 0u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrByte)) return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Clear ADDR by SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Send sub-register address; wait BTF */
    uint8_t sentByte = (uint8_t)(startRegAddr | autoIncreBitSet);
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
    if (!writeI2C(i2cBus, I2C_DR, 0u, sentByte))           		return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Send data byte */
    for(uint32_t i = 0; i < len; i++){
		if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_TXE_Pos, timeout)) return false;
		if (!writeI2C(i2cBus, I2C_DR, 0u, dataBuf[i]))              return false;
		if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;

		/* If AF set -> abort */
	    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
	    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
	    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
	    	return false;
	    }
    }

    /* STOP */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET)) return false;
    return true;
}


/*
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
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Clear ADDR by completing SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Send sub-register index; wait BTF */
    if (!writeI2C(i2cBus, I2C_DR, 0u, slaveRegAddr))            return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Repeated START; wait SB */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET))         return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout))  return false;

    /* Send <addr | read>, then wait ADDR=1 */
    const uint8_t addrRead = (uint8_t)((slaveAddr << 1) | 1u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrRead))                return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }
    /* Clear ADDR (finish by reading SR2) */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Wait RXNE=1 then read DR */
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_RXNE_Pos, timeout)) return false;
    *outResult = (uint8_t)readI2C(i2cBus, I2C_DR, 0u);

    /* STOP to release bus */
    (void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    (void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);

    return true;
}

bool I2C_readBurst(I2C_GPIO_Config_t config,
				   uint8_t           slaveAddr,
				   uint8_t           startRegAddr,
				   uint8_t			 autoIncreBitSet,
				   uint16_t			 len,
				   uint8_t* 		 outResultBuf){
    const I2C_Name_t i2cBus = config.i2cBus;
    const uint32_t   timeout = I2C_TIMEOUT_SHORT_US;

    /* Ensure bus is idle */
    if (!I2C_waitBitClear(i2cBus, I2C_SR2, SR2_BUSY_Pos, timeout));

    /* START then wait for SB */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET)) 		return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout)) 	return false;

    /* Send <addr | write>, then wait ADDR=1 */
    const uint8_t addrWrite = (uint8_t)((slaveAddr << 1) | 0u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrWrite)) 				 return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Clear ADDR by completing SR1->SR2 read sequence */
    (void)readI2C(i2cBus, I2C_SR2, 0u);

    /* Send sub-register index; wait BTF */
    const uint8_t sentByte = (uint8_t)(startRegAddr | autoIncreBitSet);
    if (!writeI2C(i2cBus, I2C_DR, 0u, sentByte))            	return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_BTF_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    /* Repeated START; wait SB */
    if (!writeI2C(i2cBus, I2C_CR1, CR1_START_Pos, SET))         return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_SB_Pos, timeout))  return false;

    /* Send <addr | read>, then wait ADDR=1 */
    const uint8_t addrRead = (uint8_t)((slaveAddr << 1) | 1u);
    if (!writeI2C(i2cBus, I2C_DR, 0u, addrRead))                 return false;
    if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_ADDR_Pos, timeout)) return false;
    if (!I2C_waitBitClear(i2cBus, I2C_SR1, SR1_AF_Pos, timeout)){
    	(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET);
    	(void)writeI2C(i2cBus, I2C_SR1, SR1_AF_Pos, RESET);
    	return false;
    }

    (void)readI2C(i2cBus, I2C_SR2, 0u); /* Clear ADDR (finish by reading SR2) */

	/* MAK (Master AK) all but last */
	(void)writeI2C(i2cBus, I2C_CR1, CR1_ACK_Pos, SET);

	bool last = false;

    for(uint32_t i = 0; i < len; i++){
    	if(i == (len - 1)) last = true;
    	if(last){
    		(void)writeI2C(i2cBus, I2C_CR1, CR1_ACK_Pos, RESET); //NMACK
    		(void)writeI2C(i2cBus, I2C_CR1, CR1_STOP_Pos, SET); //STOP
    	}

		/* Wait RXNE=1 then read DR */
		if (!I2C_waitBitSet(i2cBus, I2C_SR1, SR1_RXNE_Pos, timeout)) return false;
		outResultBuf[i] = (uint8_t)readI2C(i2cBus, I2C_DR, 0u);
    }

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
