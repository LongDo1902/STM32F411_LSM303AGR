/*
 * led.c
 *
 *  Created on: Jul 16, 2025
 *      Author: dobao
 */
#include "led.h"

void ledGreenInit(){
	Enable_GPIO_Clock(my_GPIOD);
	writePin(LED_GREEN, my_GPIOD, OSPEEDR, HIGH_SPEED);
	writePin(LED_GREEN, my_GPIOD, MODER, OUTPUT_MODE);

}

void ledOrangeInit(){
	Enable_GPIO_Clock(my_GPIOD);
	writePin(LED_GREEN, my_GPIOD, OSPEEDR, HIGH_SPEED);
	writePin(LED_ORANGE, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledRedInit(){
	Enable_GPIO_Clock(my_GPIOD);
	writePin(LED_RED, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledBlueInit(){
	Enable_GPIO_Clock(my_GPIOD);
	writePin(LED_BLUE, my_GPIOD, MODER, OUTPUT_MODE);
}

void ledsInit(){
	ledGreenInit();
	ledOrangeInit();
	ledRedInit();
	ledBlueInit();
}

void ledControl(LED_Color_t ledColor, LED_State_t on_off){
	if(on_off == ON){
		writePin(ledColor, my_GPIOD, BSRR, SET);
	}
	else if(on_off == OFF){
		writePin(ledColor, my_GPIOD, BSRR, RESET);
	}
	else return;
}


