/*
 * lsm303agr.h
 *
 *  Created on: Aug 20, 2025
 *      Author: dobao
 */

#ifndef INC_LSM303AGR_H_
#define INC_LSM303AGR_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "i2c.h"

#define ARRLEN(x) (sizeof(x)/sizeof((x)[0]))

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
 * 						MAGNETOMETOR REGISTER ADDRESS
 * =======================================================================
 */
#define LSM303AGR_OFFSET_X_REG_L_MAG	0x45U
#define LSM303AGR_OFFSET_X_REG_H_MAG	0x46U

#define LSM303AGR_OFFSET_Y_REG_L_MAG	0x47U
#define LSM303AGR_OFFSET_Y_REG_H_MAG	0x48U

#define LSM303AGR_OFFSET_Z_REG_L_MAG	0x49U
#define LSM303AGR_OFFSET_Z_REG_H_MAG	0x4AU

#define LSM303AGR_WHO_AM_I_MAG			0x4FU

#define LSM303AGR_CFG_REG_A_MAG			0x60U
#define LSM303AGR_CFG_REG_B_MAG			0x61U
#define LSM303AGR_CFG_REG_C_MAG			0x62U

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
 * ==============================================================
 * 					REGISTER BIT POSITION DEFINES
 * ==============================================================
 */
/* TEMP_CFG_REG_ACC */
#define TEMP_ACC_ENABLE_Pos		6

/* CTRL_REG1_ACC */
#define REG1_ACC_XEN_Pos		0
#define REG1_ACC_YEN_Pos		1
#define REG1_ACC_ZEN_Pos		2
#define	REG1_ACC_LPEN_Pos		3
#define REG1_ACC_ODR_Pos		4

/* CTRL_REG2_ACC */
#define REG2_ACC_HPIS1_Pos		0
#define REG2_ACC_HPIS2_Pos		1
#define REG2_ACC_HPCLICK_Pos	2
#define REG2_ACC_FDS_Pos		3
#define REG2_ACC_HPCF_Pos		4
#define REG2_ACC_HPM_Pos		6

/* CTRL_REG4_ACC */
#define REG4_ACC_SPI_EN_Pos		0
#define REG4_ACC_ST_Pos			1
#define REG4_ACC_HR_Pos			3
#define REG4_ACC_FS_Pos			4
#define REG4_ACC_BLE_Pos		6
#define REG4_ACC_BDU_Pos		7

/* CTRL_REG5_ACC */
#define REG5_ACC_BOOT_Pos		7

/* STATUS_REG_A */
#define XDA_Pos					0
#define YDA_Pos					1
#define ZDA_Pos					2
#define XYZDA_Pos				3
#define XOR_Pos					4
#define YOR_Pos					5
#define ZOR_Pos					6
#define XYZOR_Pos				7

/* CFG_REG_A_MAG */
#define REG_A_MD_Pos			0
#define REG_A_ODR_Pos			2
#define REG_A_LP_Pos			4
#define REG_A_SOFT_RST_Pos		5
#define REG_A_REBOOT_Pos		6
#define REG_A_COMP_TEMP_EN_Pos	7

/*
 * ===============================================================
 * 					CONFIGURATION TABLES
 * ===============================================================
 */
typedef enum{
	POWER_DOWN_MODE = 0x0,
	_1HZ = 0x1,
	_10Hz = 0x2,
	_25Hz = 0x3,
	_50Hz = 0x4,
	_100Hz = 0x5,
	_200Hz = 0x6,
	_400Hz = 0x7,
	_1K620Hz = 0x8, //Low-power mode
	_1344HR_5376LP = 0x9 //HR/Normal mode or Low power mode
}ODR_Sel_t;

typedef enum{
	_2g = 0x0u,
	_4g = 0x1u,
	_8g = 0x2u,
	_16g = 0x3u
}FullScale_t;

/*
 * ===============================================================
 * 				DEFAULT VALUES FOR MANUAL RESET
 * ===============================================================
 */
typedef enum{
	TEMP_CFG_REG_A_DF = 0x0,

	CTRL_REG1_A_DF = 0x7,
	CTRL_REG2_A_DF = 0x0,
	CTRL_REG3_A_DF = 0x0,
	CTRL_REG4_A_DF = 0x0,
	CTRL_REG5_A_DF = 0x0,
	CTRL_REG6_A_DF = 0x0,

	REFERENCE_A_DF = 0x0,
	FIFO_CTRL_REG_A_DF = 0x0,

	INT1_CFG_A_DF = 0x0,
	INT1_THS_A_DF = 0X0,
	INT1_DURATION_A_DF = 0x0,

	INT2_CFG_A_DF = 0x0,
	INT2_THS_A_DF = 0x0,
	INT2_DURATION_A_DF = 0x0,

	CLICK_CFG_A_DF = 0x0,
	CLICK_THS_A_DF = 0x0,

	TIME_LIMIT_A_DF = 0x0,
	TIME_LATENCY_A_DF = 0x0,
	TIME_WINDOW_A_DF = 0x0,

	ACT_THS_A_DF = 0x0,
	ACT_DUR_A_DF = 0x0,

	//----------------------------
	OFFSET_X_REG_L_M_DF = 0x0,
	OFFSET_X_REG_H_M_DF = 0x0,
	OFFSET_Y_REG_L_M_DF = 0x0,
	OFFSET_Y_REG_H_M_DF = 0x0,
	OFFSET_Z_REG_L_M_DF = 0x0,
	OFFSET_Z_REG_H_M_DF = 0x0,

	CFG_REG_A_M_DF = 0x3,
	CFG_REG_B_M_DF = 0x0,
	CFG_REG_C_M_DF = 0x0,
	INT_CTRL_REG_M_DF = 0xE,
	INT_THS_L_REG_M_DF = 0x0,
	INT_THS_H_REG_M_DF = 0x0
}DefaultValue_t;

typedef struct{
	uint8_t startReg;
	uint8_t len;
}RegSpan_t;

/*
 * ===============================================================
 * 					POWER MODE SELECTIONS
 * ===============================================================
 */
typedef enum{
	//Note: These are not the bit values, it just a random number from 0 to 2
	LOW_POWER_MODE = 0,
	NORMAL_POWER_MODE = 1,
	HIGH_RES_POWER_MODE = 2
}PowerMode_t;

typedef struct {
	I2C_GPIO_Config_t	i2c;	//Bus/pin selection
	uint8_t 			addrAcc;	//Accelerometer 7-bit address (default: 0x19U)
	uint8_t 			addrMag;	//Magnometer 7-bit address (default: 0x1EU)
}LSM303AGR_t;

typedef struct{
	int32_t 	offsetAccX;
	int32_t 	offsetAccY;
	int32_t 	offsetAccZ;
	bool 		isCalibrated;

	ODR_Sel_t	ODR_sel;
	FullScale_t fullScaleSel;
	PowerMode_t powerModeSel;
	float 		accSensitivity;

	bool	isODRSet;
	bool	isFullScaleSet;
	bool	isPowerModeSet;
	bool	isAccSensitivitySet;
}LSM303AGR_State_t;

/*
 * ==========================================================
 * 				STATUS REG A
 * ==========================================================
 */
typedef enum{
	Y_AVAILABLE,
	Z_AVAILABLE,
	XYZ_AVAILABLE,
	X_OVERRUN,
	Y_OVERRUN,
	Z_OVERRUN,
	XYZ_OVERRUN
}XYZ_Status_t;

/*
 * ==========================================================
 * 				HIGH-PASS CONFIGURATION STRUCT
 * ==========================================================
 */
typedef enum{
	HPM_NORMAL_REF_RESET = 0x00, //0b00
	HPM_REFERENCE_SIGNAL = 0x01, //0b01
	HPM_NORMAL_NO_RESET = 0X02, //0b10
	HPM_AUTORESET_INT = 0x03 //0b11
}HighPassMode_t;

typedef enum{
	HP_DATA_BYPASS,
	HP_DATA_OUTREG_FIFO
}FDS_t;

typedef struct{
	uint16_t ODR_HZ;
	float CUTOFF_HZ[4];
}HPFRow_t;

/*
 * ======================================================================
 * 						WRITE AND READ FUNCTIONS
 * ======================================================================
 */

bool LSM303AGR_writeAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value);
bool LSM303AGR_readAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult);
bool LSM303AGR_multiWriteAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg);
bool LSM303AGR_multiReadAcc(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf);

bool LSM303AGR_writeMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t value);
bool LSM303AGR_readMag(const LSM303AGR_t* lsm303agrStruct, uint8_t regAddr, uint8_t* outResult);
bool LSM303AGR_multiWriteMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, const uint8_t* dataBuf, uint16_t quantityOfReg);
bool LSM303AGR_multiReadMag(const LSM303AGR_t* lsm303agrStruct, uint8_t startRegAddr, uint16_t quantityOfReg, uint8_t* outResultBuf);

/*
 * ================================================================================
 * 							IDENTIFICATION HELPERS
 * ================================================================================
 */
bool LSM303AGR_whoAmI(const LSM303AGR_t* lsm303agrStruct, uint8_t* whoAcc, uint8_t* whoMag);
bool LSM303AGR_isPresent(const LSM303AGR_t* lsm303agrStruct);

/*
 * ================================================================================
 * 							WHOLE SENSOR RESET
 * ================================================================================
 */
bool LSM303AGR_softReset(const LSM303AGR_t* lsm303agrStruct);

/*
 * ================================================================================
 * 							TEMPERATURE CONTROL
 * ================================================================================
 */
bool LSM303AGR_disableTemperature(const LSM303AGR_t* lsm303agrStruct);
bool LSM303AGR_enableTemperature(const LSM303AGR_t* lsm303agrStruct);
bool LSM303AGR_getTemperature(const LSM303AGR_t* lsm303agrStruct, float* outTempC);

/*
 * ================================================================================
 * 						SETUP AND VERIFY CONFIGURATIONS OF ACC
 * ================================================================================
 */
bool LSM303AGR_enableBDU(const LSM303AGR_t* lsm303agrStruct);
bool LSM303AGR_setODR(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, ODR_Sel_t odr);
bool LSM303AGR_setPowerMode(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, PowerMode_t powerMode);
bool LSM303AGR_getPowerMode(const LSM303AGR_t* lsm303agrStruct, PowerMode_t* outMode);
bool LSM303AGR_isXYZ_available(const LSM303AGR_t* lsm303agrStruct);
bool LSM303AGR_enableXYZ(const LSM303AGR_t* lsm303agrStruct);
bool LSM303AGR_setFullScale(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, FullScale_t selectFullScale);
bool LSM303AGR_getFullScale(const LSM303AGR_t* lsm303agrStruct, FullScale_t* whichFullScale);

/*
 * ================================================================================
 * 							GET ACCELEROMETER MEASUREMENTS
 * ================================================================================
 */
bool LSM303AGR_readRawAcc(const LSM303AGR_t* lsm303agrStruct, int16_t rawAccBuf[3]);
bool LSM303AGR_accCalibrate(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, uint32_t sample);
bool LSM303AGR_readAcc_mg(const LSM303AGR_t* lsm303agrStruct, LSM303AGR_State_t* lsm303agrState, float outAcc_XYZ[3]);

#endif /* INC_LSM303AGR_H_ */




