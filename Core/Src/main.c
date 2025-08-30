#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "i2c.h"
#include "led.h"

I2C_GPIO_Config_t config = {
		.i2cBus = _I2C1,

		/* PB6 as SCL */
		.sclPin = my_GPIO_PIN_6,
		.sclPort = my_GPIOB,

		/* PB9 as SDA */
		.sdaPin = my_GPIO_PIN_9,
		.sdaPort = my_GPIOB
};

uint8_t outResult = 0;
bool flag = false;

int main(void){
	HAL_Init();
	ledsInit();
	I2C_basicConfigInit(config, I2C_SM_MODE, 100000U, SYSCLK_FREQ_16M);
	if(I2C_readReg8(config, 0b0011001, 0xF, &outResult)) flag = true;
	uint8_t data = outResult;
	while(1){
		if(flag == true){
			ledControl(LED_GREEN, ON);
		}
	}
}






































