/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file lsm330dlc_driver.h
*
* This file is the driver for the integrated 3axis gyroscope and accelerometer LSM330DLC
*/

#ifndef LSM330DLC_DRIVER_H_
#define LSM330DLC_DRIVER_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>

#define GY_X 0									///< Define the gyroscope X_axis for using in array
#define GY_Y 1									///< Define the gyroscope Y_axis for using in array
#define GY_Z 2									///< Define the gyroscope Z_axis for using in array

#define LSM330_ACC_SLAVE_ADDRESS  0b0011000		///< Define the Accelerometer Address, as a slave on the i2c bus
#define LSM330_GYRO_SLAVE_ADDRESS 0b1101010		///< Define the Gyroscope Address, as a slave on the i2c bus

#define LSM_GYRO_DEN_PIN AVR32_PIN_PD23			///< Define the microcontroller pin to disable(?) the gyroscope

/**
 * \brief Structure containing the configuration data of the accelerometer sensor
*/
typedef struct lsm330dlc_acc_conf_t
{
	uint8_t start_address;						///< Define the start Address of the accelerometer sensor
	uint8_t ctrl_reg_a[5];						///< Define an array containing the Control register
} lms330dlc_acc_conf_t;		

/**
 * \brief Structure containing the configuration data of the accelerometer sensor
*/
typedef struct lsm330dlc_gyro_conf_t
{
	uint8_t start_address;						///< Define the start Address of the accelerometer sensor
	uint8_t	ctrl_reg_g[5];						///< Define an array containing the Control register
} lsm330dlc_gyro_conf_t;

///< CTRL_REG_A_1
#define LSM_ACC_DATARATE_OFF		0x00		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1HZ		0x10		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_10Hz		0x20		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_25Hz		0x30		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_50Hz		0x40		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_100Hz		0x50		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_200Hz		0x60		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_400Hz		0x70		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1620Hz		0x80		///< Define the frequency of the Accelerometer data rate
#define LSM_ACC_DATARATE_1344_5376Hz 0x90		///< Define the frequency of the Accelerometer data rate

#define LSM_ACC_X_EN	0x01					///< Enable the Accelerometer X_axis
#define LSM_ACC_Y_EN	0x02					///< Enable the Accelerometer Y_axis
#define LSM_ACC_Z_EN	0x04					///< Enable the Accelerometer Z_axis
#define LSM_ACC_ALL_EN	0x07					///< Enable the Accelerometer 3 axis

#define LSM_ACC_LOW_POWER_EN 0x08				///< Enable the acclerometer low power mode

///< CTRL_REG_A_2
#define LSM_ACC_HPIS1	0x01					///< Define the Accelerometer control register A2
#define LSM_ACC_HPIS2	0x02					///< Define the Accelerometer control register A2
#define LSM_ACC_HPCLICK	0x04					///< Define the Accelerometer control register A2
#define LSM_ACC_FDS		0x08					///< Define the Accelerometer control register A2
#define LSM_ACC_HPCF1	0x10					///< Define the Accelerometer control register A2
#define LSM_ACC_HPCF2	0x20					///< Define the Accelerometer control register A2
#define LSM_ACC_HPM0	0x40					///< Define the Accelerometer control register A2
#define LSM_ACC_HPM1	0x80					///< Define the Accelerometer control register A2

///< CTRL_REG_A_3
#define LSM_ACC_OVERRUN_INT	0x02				///< Define the Accelerometer control register A3
#define LSM_ACC_FIFO_WM_INT	0x04				///< Define the Accelerometer control register A3
#define LSM_ACC_DRDY2_INT	0x08				///< Define the Accelerometer control register A3
#define LSM_ACC_DRDY1_INT	0x10				///< Define the Accelerometer control register A3
#define LSM_ACC_AOI_INT		0x40				///< Define the Accelerometer control register A3
#define LSM_ACC_CLICK_INT	0x80				///< Define the Accelerometer control register A3

///< CTRL_REG_A_4
#define LSM_ACC_SPI_MODE	0x01				///< Define the Accelerometer control register A4
#define LSM_ACC_HIGH_RES	0x08				///< Define the Accelerometer control register A4

#define LSM_ACC_FULL_SCALE_2G	0x00			///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_4G	0x10			///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_8G	0x20			///< Define the Accelerometer control register A4
#define LSM_ACC_FULL_SCALE_16G	0x30			///< Define the Accelerometer control register A4

#define LSM_ACC_BIG_ENDIAN	0x40				///< Define the Accelerometer control register A4

///< CTRL_REG_A_5
#define LSM_ACC_D4D_INT	0x04					///< Define the Accelerometer control register A5
#define LSM_ACC_LIR_INT	0x08					///< Define the Accelerometer control register A5
#define LSM_ACC_FIFO_EN	0x40					///< Define the Accelerometer control register A5
#define LSM_ACC_BOOT	0x80					///< Define the Accelerometer control register A5

///< CTRL_REG1_G
#define LSM_GYRO_DATARATE_95HZ		0x00		///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_190HZ		0x40		///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_380Hz		0x80		///< Define the frequency of the gyroscope data rate
#define LSM_GYRO_DATARATE_760Hz		0xC0		///< Define the frequency of the gyroscope data rate

///< note: actual bandwidth depends on datarate - specified for 380Hz. Consult Datasheet.
#define LSM_GYRO_BANDWIDTH_20Hz		0x40		///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_25Hz		0x50		///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_50Hz		0x60		///< Define the gyroscope bandwith corresponding to a given data rate
#define LSM_GYRO_BANDWIDTH_100Hz	0x70		///< Define the gyroscope bandwith corresponding to a given data rate

#define LSM_GYRO_X_EN	0x01					///< Enable the gyroscope X_axis
#define LSM_GYRO_Y_EN	0x02					///< Enable the gyroscope Y_axis
#define LSM_GYRO_Z_EN	0x04					///< Enable the gyroscope Z_axis
#define LSM_GYRO_ALL_EN	0x07					///< Enable the gyroscope 3 axis
#define LSM_GYRO_POWER_ON	0x08				///< Power on the gyroscope X_axis

///< CTRL_REG2_G
#define LSM_GYRO_HPCF0	0x01					///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF1	0x02					///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF2	0x04					///< Define the gyroscope control register G2
#define LSM_GYRO_HPCF3	0x08					///< Define the gyroscope control register G2
#define LSM_GYRO_HPM0	0x10					///< Define the gyroscope control register G2
#define LSM_GYRO_HPM1	0x20					///< Define the gyroscope control register G2
#define LSM_GYRO_LVL_EN	0x40					///< Enable the gyroscope control register G2
#define LSM_GYRO_EXTREN	0x80					///< Define the gyroscope control register G2

///< CTRL_REG3_G
#define LSM_GYRO_FIFO_EMPTY_INT	0x01			///< Define the gyroscope control register G3
#define LSM_GYRO_FIFO_OVRUN_INT	0x02			///< Define the gyroscope control register G3
#define LSM_GYRO_FIFO_WM_INT	0x04			///< Define the gyroscope control register G3
#define LSM_GYRO_DRDY_INT		0x08			///< Define the gyroscope control register G3
#define LSM_GYRO_PP_OD			0x10			///< Define the gyroscope control register G3
#define LSM_GYRO_H_L_ACT		0x20			///< Define the gyroscope control register G3
#define LSM_GYRO_I1_BOOT		0x40			///< Define the gyroscope control register G3
#define LSM_GYRO_I1_INT1		0x80			///< Define the gyroscope control register G3

///< CTRL_REG4_G
#define LSM_GYRO_SPI_MODE	0x01

#define LSM_GYRO_FULL_SCALE_250		0x00		///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_500		0x10		///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_2000_1	0x20		///< Define the range of the gyroscope sensor
#define LSM_GYRO_FULL_SCALE_2000_2	0x30		///< Define the range of the gyroscope sensor

#define LSM_GYRO_BIG_ENDIAN	0x40				///< Define the gyroscope sensor endianness
#define LSM_GYRO_BLOCK_DATA	0x80				///< Define the block transmission mode of the gyroscope

///< CTRL_REG5_G
#define LSM_OUT_SEL0		0x01				///< Define the gyroscope control register G5
#define LSM_OUT_SEL1		0x02				///< Define the gyroscope control register G5
#define LSM_INT_SEL0		0x04				///< Define the gyroscope control register G5
#define LSM_INT_SEL1		0x08				///< Define the gyroscope control register G5
#define LSM_GYRO_HP_EN		0x10				///< Define the gyroscope control register G5
#define LSM_GYRO_FIFO_EN	0x40				///< Enable the FIFO of the gyroscope, part of control register G5
#define LSM_GYRO_BOOT		0x80				///< Define the gyroscope control register G5

#define LSM_ACC_OUT_ADDRESS 0x28				///< Define the writing address of the accelerometer

#define LSM_ACC_FIFO_CTRL_ADDRESS 0x2E			///< Define the address of the FIFO control register, for the accelerometer
#define LSM_ACC_FIFO_SRC_ADDRESS 0x2F			///< Define the address of the FIFO source(?) register, for the accelerometer

#define LSM_GYRO_OUT_ADDRESS 0x26				///< Define the writing address of the gyroscope

#define LSM_AUTO_INCREMENT 0x80					///< Define the auto incrementation of the LSM330DLC sensor

/**
 * \brief Define the configuration of the accelerometer
*/
static const lms330dlc_acc_conf_t lsm_acc_default_config=
{
	.start_address=0x20 | LSM_AUTO_INCREMENT,
	.ctrl_reg_a=
	{	
		LSM_ACC_DATARATE_400Hz | LSM_ACC_ALL_EN ,	///< CTRL_REG_G_1
		0,											///< CTRL_REG_G_2
		0,											///< CTRL_REG_G_3
		LSM_ACC_HIGH_RES | LSM_ACC_FULL_SCALE_8G  |LSM_ACC_BIG_ENDIAN,					///< CTRL_REG_G_4
		LSM_ACC_FIFO_EN								///< CTRL_REG_G_5
	}
};

/**
 * \brief Define the configuration of the gyroscope
*/
static const lsm330dlc_gyro_conf_t lsm_gyro_default_config=
{
	.start_address = 0x20| LSM_AUTO_INCREMENT,
	.ctrl_reg_g=
	{	
		LSM_GYRO_POWER_ON | LSM_GYRO_DATARATE_760Hz | LSM_GYRO_BANDWIDTH_50Hz | LSM_GYRO_ALL_EN,	///< CTRL_REG_A_1
		0,											///< CTRL_REG_A_2
		0,											///< CTRL_REG_A_3
		LSM_GYRO_FULL_SCALE_2000_2|LSM_GYRO_BIG_ENDIAN,	///< CTRL_REG_A_4
		0 ///< LSM_GYRO_FIFO_EN
	}
};

/**
 * \brief Declare the configuration of the FIFO
*/
static const uint8_t fifo_config[2] = {LSM_ACC_FIFO_CTRL_ADDRESS, 0x80};

/**
 * \brief Structure containing the gyroscope's data
*/
typedef struct
{
	int8_t temperature;				///< Define the temperature of the sensor
	uint8_t status_register;		///< Define the status register of the gyroscope
	int16_t axes[3];				/// Define an array containing the 3 axis of the gyroscope
} lsm_gyro_data_t;

/**
 * \brief Structure containing the gyroscope FIFO's data
*/
typedef struct
{
	int8_t temperature;				///< Define the temperature of the sensor
	uint8_t status_register;		///< Define the status register of the FIFO of the gyroscope
	int16_t axes[32 * 3];				/// Define an array containing the 3 axis of the gyroscope in the FIFO
} lsm_gyro_fifo_t;

/**
 * \brief Structure containing the accelerometer's data
*/
typedef struct
{
	uint8_t status_register;		///< Define the status register of the accelerometer
	int16_t axes[3];				/// Define an array containing the 3 axis of the accelerometer
} lsm_acc_data_t;

/**
 * \brief Structure containing the accelerometer FIFO's data
*/
typedef struct
{
	//uint8_t status_register;
	uint8_t axes[32 * 6];				/// Define an array containing the 3 axis of the accelerometer in the FIFO
} lsm_acc_fifo_t;

/**
 * \brief Initialize the LSM330 accelerometer+gyroscope sensor
*/
void lsm330dlc_driver_init(void);

/**
 * \brief Return the gyroscope's data
 *
 * \return a pointer to the gyroscope data structure
*/
lsm_gyro_data_t* lsm330dlc_driver_get_gyro_data(void);

/**
 * \brief Return the accelerometer's data
 *
 * \return a pointer to the accelerometer data structure
*/
lsm_acc_data_t* lsm330dlc_driver_get_acc_data(void);

#ifdef __cplusplus
	}
#endif

#endif 