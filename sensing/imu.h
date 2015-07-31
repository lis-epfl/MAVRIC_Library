/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

/*******************************************************************************
 * \file imu.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 *   
 * \brief This file implements the IMU data structure
 *
 ******************************************************************************/


#ifndef IMU_H_
#define IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "gyroscope.h"
#include "accelerometer.h"
#include "magnetometer.h"
#include "quaternions.h"
#include "scheduler.h"
#include "state.h"

#define GYRO_LPF 0.1f						///< The gyroscope linear pass filter gain
#define ACC_LPF 0.05f						///< The accelerometer linear pass filter gain
#define MAG_LPF 0.1f						///< The magnetometer linear pass filter gain


/**
 * \brief Structure containing the accelero, gyro and magnetometer sensors' gains
 */
typedef struct
{
	float bias[3];							///< The biais of the sensor
	float scale_factor[3];					///< The scale factors of the sensor
	float orientation[3];					///< The orientation of the sensor
	uint8_t axis[3];						///< The axis number (X,Y,Z) referring to the sensor datasheet
	
	float max_oriented_values[3];			///< Values uses for calibration
	float min_oriented_values[3];			///< Values uses for calibration
	bool calibration;						///< In calibration mode, true
} sensor_calib_t;


/**
 * \brief Structure containing the configuration 
 * accelero, gyro and magnetometer sensors' gains
 */
typedef struct
{
	float bias[3];							///< The biais of the sensor
	float scale_factor[3];					///< The scale factors of the sensor
	float orientation[3];					///< The orientation of the sensor
	uint8_t axis[3];						///< The axis number (X,Y,Z) referring to the sensor datasheet
} sensor_config_t;


/**
 * \brief The configuration IMU structure
 */
typedef struct
{
	sensor_config_t accelerometer;		   ///< The gyroscope configuration structure
	sensor_config_t gyroscope;			   ///< The accelerometer configuration structure
	sensor_config_t magnetometer;		   ///< The compass configuration structure
} imu_conf_t;


/**
 * \brief The IMU structure
 */
typedef struct
{
	sensor_calib_t 	 calib_gyro;			///< The gyroscope calibration structure
	sensor_calib_t   calib_accelero;		///< The accelerometer calibration structure
	sensor_calib_t   calib_compass;			///< The compass calibration structure
	
	gyroscope_t      raw_gyro;				///< The gyroscope raw values structure
	gyroscope_t      oriented_gyro;			///< The gyroscope oriented values structure
	gyroscope_t      scaled_gyro;			///< The gyroscope scaled values structure
	
	accelerometer_t  raw_accelero;			///< The accelerometer raw values structure
	accelerometer_t  oriented_accelero;		///< The accelerometer oriented values structure
	accelerometer_t  scaled_accelero;		///< The accelerometer scaled values structure
	
	magnetometer_t   raw_compass;			///< The compass raw values structure
	magnetometer_t   oriented_compass;		///< The compass oriented values structure
	magnetometer_t   scaled_compass;		///< The compass scaled values structure
	
	float dt;								///< The time interval between two IMU updates
	uint32_t last_update;					///< The time of the last IMU update in ms

	state_t* state;							///< The pointer to the state structure
} imu_t;


/** 
 * \brief	Initialize the IMU module
 *
 * \param	imu						The pointer to the IMU structure
 * \param	conf_imu				The pointer to the configuration IMU structure
 * \param	state					The pointer to the state structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool imu_init (imu_t *imu, imu_conf_t *conf_imu, state_t* state);


/**
 * \brief	To calibrate the gyros at startup (not used know)
 *
 * \param	imu		The pointer to the IMU structure
 */
void imu_calibrate_gyros(imu_t *imu);


/**
 * \brief	Updates the scaled sensors values from raw measurements
 *
 * \param	imu		The pointer to the IMU structure
 */
void imu_update(imu_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
