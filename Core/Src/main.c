/*
 * @file    main.c
 * @brief   Minimal demo: init I2C1 (PB6/PB9), write then read a register, show status on LEDs.
 *
 * Behavior:
 *  - RED  LED ON  => write succeeded
 *  - GREEN LED ON => read  succeeded
 *
 * Notes:
 *  - Uses a 7-bit, unshifted slave address (0x19).
 *  - Keeps the same control flow and result flags as your original code.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#include "i2c.h"
#include "led.h"

/* -------------------------------------------------------------------------- */
/*                        Application constants (no magic)                     */
/* -------------------------------------------------------------------------- */

/* I2C target device and register */
#define DEV_ADDR_7BIT          0x19u     /* 0b0011001 (unshifted 7-bit address) */
#define REG_CTRL               0x1Fu
#define REG_CTRL_VALUE         0x03u     /* 0b00000011 */

/* I2C1 on PB6 (SCL) / PB9 (SDA) */
static const I2C_GPIO_Config_t i2cConfig = {
    .i2cBus  = _I2C1,

    .sclPin  = my_GPIO_PIN_6,  /* PB6 as SCL */
    .sclPort = my_GPIOB,

    .sdaPin  = my_GPIO_PIN_9,  /* PB9 as SDA */
    .sdaPort = my_GPIOB
};

/* Result storage and flags (preserved behavior) */
static uint8_t outResult = 0u;
static bool    readFlag  = false;
static bool    writeFlag = false;

int main(void)
{
    HAL_Init();
    ledsInit();

    /* Configure GPIOs + I2C timing and enable peripheral (per your i2c.c) */
    I2C_basicConfigInit(i2cConfig);

    /* Write REG_CTRL_VALUE to REG_CTRL, then read it back */
    if (I2C_writeReg8(i2cConfig, DEV_ADDR_7BIT, REG_CTRL, REG_CTRL_VALUE)) {
        writeFlag = true;
    }
    if (I2C_readReg8(i2cConfig, DEV_ADDR_7BIT, REG_CTRL, &outResult)) {
        readFlag = true;
    }

    /* Indicate status on LEDs (same logic, just tidied) */
    for (;;) {
        if (readFlag)  { ledControl(LED_GREEN, ON); }
        if (writeFlag) { ledControl(LED_RED,   ON); }
        /* (Optional) Add a tiny delay here if you want to avoid a tight loop. */
    }
}
