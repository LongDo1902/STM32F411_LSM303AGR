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
uint8_t value1 = 0;
uint8_t value2 = 0;
int8_t lsm303agr_temperature = 0;

uint8_t dataOutBuf[6];

bool isRead1Ok = false;
bool isRead2Ok = false;
bool isMultiReadOk = false;

int main(void){
	HAL_Init();
	ledsInit();

	(void)I2C_GPIO_init(i2cConfig);
	(void)I2C_Init(i2cConfig);

	LSM303AGR_rebootAcc(&lsm303agrConfig);
	LSM303AGR_softReset(&lsm303agrConfig);

	LSM303AGR_multiReadAcc(&lsm303agrConfig, LSM303AGR_CTRL_REG1_ACC, sizeof(dataOutBuf), dataOutBuf);

	(void)LSM303AGR_readAcc(&lsm303agrConfig, LSM303AGR_TEMP_CFG_REG_ACC, &value2);
	LSM303AGR_enableTemperature(&lsm303agrConfig);
	(void)LSM303AGR_readAcc(&lsm303agrConfig, LSM303AGR_TEMP_CFG_REG_ACC, &value2);

	while(1){
		if(isRead1Ok){
			ledControl(LED_GREEN, ON);
		}
		if(isRead2Ok){
			ledControl(LED_ORANGE, ON);
		}
		if(isMultiReadOk){
			ledControl(LED_RED, ON);
		}
	}
}
