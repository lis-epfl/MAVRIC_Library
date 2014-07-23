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
* \file lsm330dlc.h
*
* This file is the driver for the integrated 3axis gyroscope and accelerometer LSM330DLC
*/

#ifndef LSM330DLC_H_
#define LSM330DLC_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "gyroscope.h"
#include "accelerometer.h"		

/**
 * \brief	Structure containing the accelerometer's data
*/
typedef struct
{
	//int16_t axes[3];				///< Define an array containing the 3 axis of the accelerometer
	accelerometer_t *raw_accelero;
} lsm_acc_t;

/**
 * \brief	Structure containing the gyroscope's data
*/
typedef struct
{
	gyroscope_t *raw_gyro;
} lsm_gyro_t;


/**
 * \brief	Initialize the LSM330 accelerometer+gyroscope sensor
*/
void lsm330dlc_init(void);

/**
 * \brief	Return the gyroscope's data
 *
 * \return	A pointer to the gyroscope data structure
*/
void lsm330dlc_gyro_update(gyroscope_t *lsm_gyro_outputs);

/**
 * \brief	Return the accelerometer's data
 *
 * \return	A pointer to the accelerometer data structure
*/
void lsm330dlc_acc_update(accelerometer_t *lsm_acc_outputs);

#ifdef __cplusplus
	}
#endif

#endif 