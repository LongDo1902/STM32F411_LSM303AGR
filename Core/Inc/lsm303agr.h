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
#define LSM303AGR_I2C_ADDR_ACC		0x19U
#define LSM303AGR_I2C_ADDR_MAG		0x1EU

/* WHO_AM_I expected values */
#define LSM303AGR_WHO_AM_I_ACC_VAL	0x33U
#define LSM303AGR_WHO_AM_I_MAG_VAL	0x40U

/* Auto-increment bit (set MSB of register address for	 multi-byte ops) */
#define LSM303AGR_ADDR_AUTO_INC		0x80U

/*
 * =======================================================================
 * 						ACCELEROMETER REGISTER ADDRESS
 * =======================================================================
 */
#define LSM303AGR_STATUS_REG_AUX_ACC	0x07U
#define LSM303AGR_OUT_TEMP_L_ACC		0x0CU
#define LSM303AGR_OUT_TEMP_H_ACC		0x0DU
#define LSM303AGR_INT_COUNTER_REG_ACC	0x0EU
#define LSM303AGR_WHO_AM_I_ACC			0x0FU

#define LSM303AGR_TEMP_CFG_REG_ACC		0x1FU

#define LSM303AGR_CTRL_REG1_ACC		0x20U
#define LSM303AGR_CTRL_REG2_ACC		0x21U
#define LSM303AGR_CTRL_REG3_ACC		0x22U
#define LSM303AGR_CTRL_REG4_ACC		0x23U
#define LSM303AGR_CTRL_REG5_ACC		0x24U
#define LSM303AGR_CTRL_REG6_ACC		0x25U

#define LSM303AGR_REFERENCE_ACC		0x26U
#define LSM303AGR_STATUS_REG_ACC	0x27U

#define LSM303AGR_OUT_X_L_ACC		0x28U
#define LSM303AGR_OUT_X_H_ACC		0x29U

#define LSM303AGR_OUT_Y_L_ACC		0x2AU
#define LSM303AGR_OUT_Y_H_ACC		0x2BU

#define LSM303AGR_OUT_Z_L_ACC		0x2CU
#define LSM303AGR_OUT_Z_H_ACC		0x2DU

#define LSM303AGR_FIFO_CTRL_REG_ACC	0x2EU
#define LSM303AGR_FIFO_SRC_REG_ACC	0x2FU

#define LSM303AGR_INT1_CFG_ACC		0x30U
#define LSM303AGR_INT1_SRC_ACC		0x31U
#define LSM303AGR_INT1_THS_ACC		0x32U
#define LSM303AGR_INT1_DURATION_ACC	0x33U

#define LSM303AGR_INT2_CFG_ACC		0x34U
#define LSM303AGR_INT2_SRC_ACC		0x35U
#define LSM303AGR_INT2_THS_ACC		0x36U
#define LSM303AGR_INT2_DURATION_ACC	0x37U

#define LSM303AGR_CLICK_CFG_ACC		0x38U
#define LSM303AGR_CLICK_SRC_ACC		0x39U
#define LSM303AGR_CLICK_THS_ACC		0x3AU

#define LSM303AGR_TIME_LIMIT_ACC	0x3BU
#define LSM303AGR_TIME_LATENCY_ACC	0x3CU
#define LSM303AGR_TIME_WINDOW_ACC	0x3DU

#define LSM303AGR_ACT_THS_ACC		0x3EU
#define LSM303AGR_ACT_DUR_ACC		0x3FU

/*
 * =======================================================================
 * 						MAGNOMETOR REGISTER ADDRESS
 * =======================================================================
 */
#define LSM303AGR_OFFSET_X_REG_L_MAG	0x45U
#define LSM303AGR_OFFSET_X_REG_H_MAG	0x46U

#define LSM303AGR_OFFSET_Y_REG_L_MAG	0x47U
#define LSM303AGR_OFFSET_Y_REG_H_MAG	0x48U

#define LSM303AGR_OFFSET_Z_REG_L_MAG	0x49U
#define LSM303AGR_OFFSET_Z_REG_H_MAG	0x4AU

#define LSM303AGR_WHO_AM_I_MAG			0x4FU

#define LSM303AGR_CFG_REG1_MAG			0x60U
#define LSM303AGR_CFG_REG2_MAG			0x61U
#define LSM303AGR_CFG_REG3_MAG			0x62U

#define LSM303AGR_INT_CTRL_REG_MAG		0x63U
#define LSM303AGR_INT_SOURCE_REG_MAG	0x64U
#define LSM303AGR_INT_THS_L_REG_MAG		0x65U
#define LSM303AGR_INT_THS_H_REG_MAG		0x66U

#define LSM303AGR_STATUS_REG_MAG		0x67U

#define LSM303AGR_OUT_X_L_MAG			0x68U
#define LSM303AGR_OUT_X_H_MAG			0x69U

#define LSM303AGR_OUT_Y_L_MAG			0x6AU
#define LSM303AGR_OUT_Y_H_MAG			0x6BU

#define LSM303AGR_OUT_Z_L_MAG			0x6CU
#define LSM303AGR_OUT_Z_H_MAG			0x6DU

/*
 * ==========================================================
 *					BUS & PIN SELECTION
 * ==========================================================
 */

typedef struct {
	I2C_GPIO_Config_t	i2c;	//Bus/pin selection
	uint8_t 			addrAcc;	//Accelerometer 7-bit address (default: 0x19U)
	uint8_t 			addrMag;	//Magnometer 7-bit address (default: 0x1EU)
}LSM303AGR_t;

/*
 * ======================================================================
 * 					FUNCTION DECLARATIONS
 * ======================================================================
 */
bool lsm303agr_whoAmI(const LSM303AGR_t* dev, uint8_t* whoAcc, uint8_t* whoMag);
bool lsm303agr_isPresent();

bool lsm303agr_writeAcc();
bool lsm303agr_readAAcc();

bool lsm303agr_writeMag();
bool lsm303agr_readMag();

bool lsm303agr_readAccRaw();
bool lsm303agr_readMagRaw();
bool lsm303agr_readTempRaw();

#endif /* INC_LSM303AGR_H_ */




