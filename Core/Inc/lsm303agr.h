/*
 * lsm303agr.h
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */

#ifndef INC_LSM303AGR_H_
#define INC_LSM303AGR_H_

#include "stdio.h"
#include "stdint.h"

#include "i2c.h"

/*
 * =======================================================================
 * 						I2C ADDRESSES & WHO AM I
 * =======================================================================
 */
/* Unshifted I2C addresses (ACC and MAG) */
#define I2C_ADDR_ACC		0x19U
#define I2C_ADDR_MAG		0x1EU

/* WHO_AM_I expected values */
#define WHO_AM_I_ACC_VAL	0x33U
#define WHO_AM_I_MAG_VAL	0x40U

/* Auto-increment bit (set MSB of register address for multi-byte ops) */
#define ADDR_AUTO_INC		0x80U

/*
 * =======================================================================
 * 						ACCELEROMETER REGISTER ADDRESS
 * =======================================================================
 */
#define STATUS_REG_AUX_ACC	0x07U
#define OUT_TEMP_L_ACC		0x0CU
#define OUT_TEMP_H_ACC		0x0DU
#define INT_COUNTER_REG_ACC	0x0EU
#define WHO_AM_I_ACC		0x0FU

#define TEMP_CFG_REG_ACC	0x1FU

#define CTRL_REG1_ACC		0x20U
#define CTRL_REG2_ACC		0x21U
#define CTRL_REG3_ACC		0x22U
#define CTRL_REG4_ACC		0x23U
#define CTRL_REG5_ACC		0x24U
#define CTRL_REG6_ACC		0x25U

#define REFERENCE_ACC		0x26U
#define STATUS_REG_ACC		0x27U

#define OUT_X_L_ACC			0x28U
#define OUT_X_H_ACC			0x29U

#define OUT_Y_L_ACC			0x2AU
#define OUT_Y_H_ACC			0x2BU

#define OUT_Z_L_ACC			0x2CU
#define OUT_Z_H_ACC			0x2DU

#define FIFO_CTRL_REG_ACC	0x2EU
#define FIFO_SRC_REG_ACC	0x2FU

#define INT1_CFG_ACC		0x30U
#define INT1_SRC_ACC		0x31U
#define INT1_THS_ACC		0x32U
#define INT1_DURATION_ACC	0x33U

#define INT2_CFG_ACC		0x34U
#define INT2_SRC_ACC		0x35U
#define INT2_THS_ACC		0x36U
#define INT2_DURATION_ACC	0x37U

#define CLICK_CFG_ACC		0x38U
#define CLICK_SRC_ACC		0x39U
#define CLICK_THS_ACC		0x3AU

#define TIME_LIMIT_ACC		0x3BU
#define TIME_LATENCY_ACC	0x3CU
#define TIME_WINDOW_ACC		0x3DU

#define ACT_THS_ACC			0x3EU
#define ACT_DUR_ACC			0x3FU

/*
 * =======================================================================
 * 						MAGNOMETOR REGISTER ADDRESS
 * =======================================================================
 */
#define OFFSET_X_REG_L_MAG	0x45U
#define OFFSET_X_REG_H_MAG	0x46U

#define OFFSET_Y_REG_L_MAG	0x47U
#define OFFSET_Y_REG_H_MAG	0x48U

#define OFFSET_Z_REG_L_MAG	0x49U
#define OFFSET_Z_REG_H_MAG	0x4AU

#define WHO_AM_I_MAG		0x4FU

#define CFG_REG1_MAG		0x60U
#define CFG_REG2_MAG		0x61U
#define CFG_REG3_MAG		0x62U

#define INT_CTRL_REG_MAG	0x63U
#define INT_SOURCE_REG_MAG	0x64U
#define INT_THS_L_REG_MAG	0x65U
#define INT_THS_H_REG_MAG	0x66U

#define STATUS_REG_MAG		0x67U

#define OUT_X_L_MAG			0x68U
#define OUT_X_H_MAG			0x69U

#define OUT_Y_L_MAG			0x6AU
#define OUT_Y_H_MAG			0x6BU

#define OUT_Z_L_MAG			0x6CU
#define OUT_Z_H_MAG			0x6DU

/*
 * ======================================================================
 * 					FUNCTION DECLARATIONS
 * ======================================================================
 */
bool lsm303agr_whoAmI(uint8_t* whoAcc, uint8_t* whoMag);
bool lsm303agr_isPresent();

bool lsm303agr_writeAcc();
bool lsm303agr_readAAcc();

bool lsm303agr_writeMag();
bool lsm303agr_readMag();

bool lsm303agr_readAccRaw();
bool lsm303agr_readMagRaw();
bool lsm303agr_readTempRaw();


#endif /* INC_LSM303AGR_H_ */





