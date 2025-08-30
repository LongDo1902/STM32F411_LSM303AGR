//#include <stdio.h>
//#include <stdint.h>
//#include <stdbool.h>
//
//#include "i2c.h"
//#include "led.h"
//
//I2C_GPIO_Config_t config = {
//		.i2cBus = _I2C1,
//
//		/* PB6 as SCL */
//		.sclPin = my_GPIO_PIN_6,
//		.sclPort = my_GPIOB,
//
//		/* PB9 as SDA */
//		.sdaPin = my_GPIO_PIN_9,
//		.sdaPort = my_GPIOB
//};
//
//uint8_t outResult = 0;
//bool flag = false;
//
//int main(void){
//	HAL_Init();
//	ledGreenInit();
//	I2C_basicConfigInit(config, I2C_SM_MODE, 100000U, HAL_RCC_GetPCLK1Freq());
//	if(I2C_readReg8(config, 0b0011001, 0x0F, &outResult)) flag = true; //Read Acceleration sensor
//
//	while(1){
//		if(flag){
//			ledControl(LED_GREEN, ON);
//		}
//	}
//}

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"
#include "led.h"
#include "stm32PeripheralAddr.h"
#include "gpioWriteRead.h"
#include "timer.h"
#include "exti.h"
#include "rcc.h"
#include "i2c.h"

int main(void){
	initTimer(my_TIM1);

	I2C_GPIO_Config_t i2cConfig = {
			.i2cBus = my_I2C1,

			/* PB6 as SCL */
			.sclPin = my_GPIO_PIN_6,
			.sclPort = my_GPIOB,

			/* PB9 as SDA */
			.sdaPin = my_GPIO_PIN_9,
			.sdaPort = my_GPIOB,
	};

	RCC_init();
	I2C_basicConfigInit(i2cConfig, I2C_SM_100K, 100000, 50000000); //Standard mode 100kHz and 50MHz APB peripheral
	uint8_t dataRead = (uint8_t) I2C_singleByteRead(i2cConfig, 0b0011001, 0x0F);
	I2C_singleByteWrite(i2cConfig, 0b0011001, 0x1F, 0b11000000);
	uint8_t tempCfgRegRead = (uint8_t) I2C_singleByteRead(i2cConfig, 0b0011001, 0x1F);

	while(1){
	}
}
