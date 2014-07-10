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
#include "gyro.h"
#include "accelero.h"		

/**
 * \brief	Structure containing the accelerometer's data
*/
typedef struct
{
	//int16_t axes[3];				///< Define an array containing the 3 axis of the accelerometer
	accelero_data_t *acceleroData;
} lsm_acc_data_t;

/**
 * \brief	Structure containing the gyroscope's data
*/
typedef struct
{
	gyro_data_t *gyroData;
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
void lsm330dlc_gyro_update(gyro_data_t *lsm_gyro_outputs);

/**
 * \brief	Return the accelerometer's data
 *
 * \return	A pointer to the accelerometer data structure
*/
void lsm330dlc_acc_update(accelero_data_t *lsm_acc_outputs);

#ifdef __cplusplus
	}
#endif

#endif 