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

/**
 * \brief Structure containing the configuration data of the accelerometer sensor
*/
typedef struct
{
	uint8_t start_address;			///< Define the start Address of the accelerometer sensor
	uint8_t	ctrl_reg_g[5];			///< Define an array containing the Control register
} lsm330dlc_gyro_conf_t;

/**
 * \brief Structure containing the configuration data of the accelerometer sensor
*/
typedef struct
{
	uint8_t start_address;			///< Define the start Address of the accelerometer sensor
	uint8_t ctrl_reg_a[5];			///< Define an array containing the Control register
} lsm330dlc_acc_conf_t;

/**
 * \brief	Structure containing the accelerometer's data
*/
typedef struct
{
	int16_t axes[3];				///< Define an array containing the 3 axis of the accelerometer
} lsm_acc_data_t;

/**
 * \brief	Structure containing the gyroscope's data
*/
typedef struct
{
	int8_t temperature;				///< Define the temperature of the sensor
	int16_t axes[3];				///< Define an array containing the 3 axis of the gyroscope
} lsm_gyro_data_t;


/**
 * \brief	Initialize the LSM330 accelerometer+gyroscope sensor
*/
void lsm330dlc_driver_init(void);

/**
 * \brief	Return the gyroscope's data
 *
 * \return	A pointer to the gyroscope data structure
*/
lsm_gyro_data_t* lsm330dlc_driver_get_gyro_data(void);

/**
 * \brief	Return the accelerometer's data
 *
 * \return	A pointer to the accelerometer data structure
*/
lsm_acc_data_t* lsm330dlc_driver_get_acc_data(void);

#ifdef __cplusplus
	}
#endif

#endif 