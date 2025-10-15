/*
 * @file    main.c
 *
 * Created on Aug 31, 2025
 * 		Author dobao
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <main.h>

#include "i2c.h"
#include "led.h"
#include "lsm303agr.h"
/*
 * =============================================================
 * 					PIN & PIN & I2C ADDR
 * =============================================================
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
 * ===============================================================
 * 					PARAMETERS DECLARATIONS
 * ===============================================================
 */

/*
 * Temperature related parameter declaration
 */
float lsm303agr_temperature = 0.0;

/*
 * Raw, offset and converted parameters of Accelerometer
 */
#define CALIBRATE_SAMPLE 6
int16_t rawAccX[CALIBRATE_SAMPLE], rawAccY[CALIBRATE_SAMPLE], rawAccZ[CALIBRATE_SAMPLE];
int32_t offsetRawX, offsetRawY, offsetRawZ;
float outAcc_XYZ[3];

/*
 * ================================================================
 * 						MAIN THINGS HAPPEN
 * ================================================================
 */
int main(void){
	HAL_Init();
	ledsInit();

	/* Initialize I2C pins and its configurations to communicate to LSM303AGR */
	(void)I2C_GPIO_init(i2cConfig); //Init SCL and SDA pins
	(void)I2C_Init(i2cConfig); //Init CCR mode and SCL frequency
	HAL_Delay(5); //Small delay ensure everything boot up fine before configure others

	/* Reset the sensor registers to its default values */
	LSM303AGR_softReset(&lsm303agrConfig);
	HAL_Delay(5);

	/* Set the Output Data Rate to 400Hz */
	if(!LSM303AGR_setODR_acc(&lsm303agrConfig, _400Hz)) return false;
	HAL_Delay(5);

	/* Enable BDU and Temperature before reading temperature */
	if(!LSM303AGR_enableBDU_acc(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	if(!LSM303AGR_enableTemperature(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	/* Set the sensor to Normal Power Mode */
	if(!LSM303AGR_setPowerMode(&lsm303agrConfig, NORMAL_POWER_MODE)) return false;
	HAL_Delay(5);

	/* Enabling XYZ axes */
	if(!LSM303AGR_XYZ_enable(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	/* Configure Full Scale */
	if(!LSM303AGR_setFullScale(&lsm303agrConfig, _4g)) return false;
	HAL_Delay(5);

	if(!LSM303AGR_accCalibrate(&lsm303agrConfig, CALIBRATE_SAMPLE, rawAccX, rawAccY, rawAccZ, &offsetRawX, &offsetRawY, &offsetRawZ)) return false;

	while(1){
		if(!LSM303AGR_readAcc_mg(&lsm303agrConfig, outAcc_XYZ)) return false;
		HAL_Delay(100);
	}
}
