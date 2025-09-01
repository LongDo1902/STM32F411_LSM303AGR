/*
 * lsm303agr.c
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */
#include "i2c.h"
#include "lsm303agr.h"

/*
 * ==========================================================
 * 					PIN & PIN & I2C ADDR
 * ==========================================================
 */
/* I2C1 on PB6 (SCL) / PB9 (SDA) */
static const I2C_GPIO_Config_t i2cConfig = {
    .i2cBus  = _I2C1,

    .sclPin  = my_GPIO_PIN_6,  /* PB6 as SCL */
    .sclPort = my_GPIOB,

    .sdaPin  = my_GPIO_PIN_9,  /* PB9 as SDA */
    .sdaPort = my_GPIOB
};

static const LSM303AGR_t lsm303agrConfig = {
		.i2c = i2cConfig,
		.addrAcc = LSM303AGR_I2C_ADDR_ACC, //0x19U
		.addrMag = LSM303AGR_I2C_ADDR_MAG //0x1EU
};

/*
 * ==================================================================
 * 					INTERNAL HELPERS OF ACCELEROMETER
 * ==================================================================
 */
static inline bool writeAcc(const LSM303AGR_t* dev, uint8_t addrReg, uint8_t value){
	return I2C_writeReg8(dev -> i2c, dev -> addrAcc, addrReg, value);
}

static inline bool readAcc(const LSM303AGR_t* dev, uint8_t addrReg, uint8_t* outResult){
	return I2C_readReg8(dev -> i2c, dev -> addrAcc, addrReg, outResult);
}


/*
 * ==================================================================
 * 					INTERNAL HELPERS OF MAGNOMETER
 * ==================================================================
 */
static inline bool writeMag(const LSM303AGR_t* dev, uint8_t addrReg, uint8_t value){
	return I2C_writeReg8(dev -> i2c, dev -> addrMag, addrReg, value);
}



static inline bool readMag(const LSM303AGR_t* dev, uint8_t addrReg, uint8_t* outResult){
	return I2C_readReg8(dev -> i2c, dev -> addrMag, addrReg, outResult);
}


/*
 * ==========================================================
 * 					PUBLIC HELPERS
 * ==========================================================
 */
bool lsm303agr_whoAmI(const LSM303AGR_t* dev, uint8_t* whoAcc, uint8_t* whoMag){
	if((dev == NULL) | (whoAcc == NULL) | (whoMag == NULL)) return false;

	if(!readAcc(dev, LSM303AGR_WHO_AM_I_ACC, whoAcc)) return false;
	if(!readMag(dev, LSM303AGR_WHO_AM_I_MAG, whoMag)) return false;
	return true;
}

bool lsm303agr_isPresent(){
	uint8_t whoAcc = 0u;
	uint8_t whoMag = 0u;
	if(!lsm303agr_whoAmI(&lsm303agrConfig, &whoAcc, &whoMag)) return false;
	return ((whoAcc == LSM303AGR_WHO_AM_I_ACC_VAL) && (whoMag == LSM303AGR_WHO_AM_I_MAG_VAL));
}





















