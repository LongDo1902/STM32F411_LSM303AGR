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

#include "i2c.h"
#include "led.h"
#include "lsm303agr.h"

int _delay = 500;
bool flag = false;
int main(void)
{
    HAL_Init();
    ledsInit();
    if(!lsm303agr_isPresent()) flag = true;

    for (;;) {
    	if(flag){
    		ledControl(LED_GREEN, ON);
    		HAL_Delay(_delay);
    		ledControl(LED_GREEN, OFF);
    		HAL_Delay(_delay);
    	}
    }
}
