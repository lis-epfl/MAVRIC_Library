/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file imu.h
 *
 * This file implements the IMU data structure
 */


#ifndef IMU_H_
#define IMU_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "conf_platform.h"
#include "gyro.h"
#include "accelero.h"
#include "compass.h"
#include "quaternions.h"
#include "scheduler.h"

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
	float axis[3];							///< The axis number (X,Y,Z) referring to the sensor datasheet
}sensor_calib_t;


/**
 * \brief The IMU structure
 */
typedef struct
{
	sensor_calib_t 	 calib_gyro;			///< The gyroscope calibration structure
	sensor_calib_t   calib_accelero;		///< The accelerometer calibration structure
	sensor_calib_t   calib_compass;			///< The compass calibration structure
	
	gyro_data_t      raw_gyro;				///< The gyroscope raw values structure
	gyro_data_t      oriented_gyro;			///< The gyroscope oriented values structure
	gyro_data_t      scaled_gyro;			///< The gyroscope scaled values structure
	
	accelero_data_t  raw_accelero;			///< The accelerometer raw values structure
	accelero_data_t  oriented_accelero;		///< The accelerometer oriented values structure
	accelero_data_t  scaled_accelero;		///< The accelerometer scaled values structure
	
	compass_data_t   raw_compass;			///< The compass raw values structure
	compass_data_t   oriented_compass;		///< The compass oriented values structure
	compass_data_t   scaled_compass;		///< The compass scaled values structure
	
	float dt;								///< The time interval between two IMU updates
	uint32_t last_update;					///< The time of the last IMU update in ms
	uint8_t calibration_level;				///< The level of calibration
	
	const mavlink_stream_t* mavlink_stream;		///< The pointer to the mavlink stream
} imu_t;


/** 
 * \brief	Initialize the IMU module
 *
 * \param	imu						The pointer to the IMU structure
 */
void imu_init (imu_t *imu, const mavlink_stream_t* mavlink_stream);

/**
 * \brief	To calibrate the gyros at startup (not used know)
 *
 * \param	imu						The pointer to the IMU structure
 */
void imu_calibrate_gyros(imu_t *imu);

/**
 * \brief	Updates the scaled sensors values from raw measurements
 *
 * \param	imu						The pointer to the IMU structure
 */
void imu_update(imu_t *imu);

/**
 * \brief	Relevels the imu
 *
 * \param	imu						The pointer to the IMU structure
 */
void imu_relevel(imu_t *imu);


/**
 * \brief	Task to send the mavlink scaled IMU message
 * 
 * \param	imu						The pointer to the IMU structure
 *
 * \return	The status of execution of the task
 */
task_return_t imu_send_scaled(imu_t* imu);

/**
 * \brief	Task to send the mavlink raw IMU message
 * 
 * \param	imu						The pointer to the IMU structure
 *
 * \return	The status of execution of the task
 */
task_return_t imu_send_raw(imu_t* imu);


#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
