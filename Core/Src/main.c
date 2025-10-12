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
uint8_t tempAddrVal = 0;
uint8_t bduAddrVal = 0;
float lsm303agr_temperature = 0.0;
uint8_t ctrlReg2 = 0;
uint8_t referenceDataCapture = 0;

uint8_t dataOutBuf[6];

bool isRead1Ok = false;
bool isRead2Ok = false;
bool isMultiReadOk = false;


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

	/* Set the Output Data Rate */
	if(!LSM303AGR_setODR_acc(&lsm303agrConfig, _400Hz)) return false;
	HAL_Delay(5);

	/* Enable BDU and Temperature */
	if(!LSM303AGR_enableTemperature(&lsm303agrConfig)) return false;
	HAL_Delay(5);
	if(!LSM303AGR_enableBDU_acc(&lsm303agrConfig)) return false;
	HAL_Delay(5);

	while(1){
		LSM303AGR_getTemperature(&lsm303agrConfig, &lsm303agr_temperature);
		HAL_Delay(250);
	}
}
