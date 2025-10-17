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

static LSM303AGR_State_t lsm303agrState = {
	.offsetAccX = 0,
	.offsetAccY = 0,
	.offsetAccZ = 0,
	.isCalibrated = false,

	/* These settings are default. They will be reconfigured in main.c */
	.ODR_sel = _10Hz,
	.fullScaleSel = _2g,
	.powerModeSel = NORMAL_POWER_MODE,
	.accSensitivity = 0.0f,
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
#define CALIBRATE_SAMPLE 200
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

	/* Check if LSM303AGR is presenting */
	if(!LSM303AGR_isPresent(&lsm303agrConfig)) return false;

	/* GREEN LED indicates device found and starts configuring */
	for(uint8_t i = 0; i < 2; i++){
		for(uint8_t j = 0; j < 3; j++){
			ledControl(LED_GREEN, ON);
			HAL_Delay(70);
			ledControl(LED_GREEN, OFF);
			HAL_Delay(70);
		}
		HAL_Delay(300);
	}

	/* Reset the sensor registers to its default values */
	LSM303AGR_softReset(&lsm303agrConfig);
	HAL_Delay(5);

	/* Set the Output Data Rate to 400Hz */
	if(!LSM303AGR_setODR(&lsm303agrConfig, _400Hz)) return false;
	HAL_Delay(5);

	/* Enable BDU */
	if(!LSM303AGR_enableBDU(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	/* Set the sensor to Normal Power Mode */
	if(!LSM303AGR_setPowerMode(&lsm303agrConfig, NORMAL_POWER_MODE)) return false;
	HAL_Delay(5);

	/* Enabling XYZ axes */
	if(!LSM303AGR_enableXYZ(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	/* Configure Full Scale */
	if(!LSM303AGR_setFullScale(&lsm303agrConfig, _4g)) return false;
	HAL_Delay(5);

	if(!LSM303AGR_accCalibrate(&lsm303agrConfig, &lsm303agrState, CALIBRATE_SAMPLE)) return false;

	/* BLUE LED indicates the configuring completed */
	for(uint8_t i = 0; i < 2; i++){
		for(uint8_t j = 0; j < 3; j++){
			ledControl(LED_BLUE, ON);
			HAL_Delay(70);
			ledControl(LED_BLUE, OFF);
			HAL_Delay(70);
		}
		HAL_Delay(300);
	}

	while(1){
		if(!LSM303AGR_readAcc_mg(&lsm303agrConfig, &lsm303agrState, outAcc_XYZ)) return false;
		HAL_Delay(250);
	}
}
