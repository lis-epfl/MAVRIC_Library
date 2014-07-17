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
 * This file implements the code to read the IMU data
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

#define GYRO_LPF 0.1f					///< The gyroscope linear particle filter gain
#define ACC_LPF 0.05f					///< The accelerometer linear particle filter gain
#define MAG_LPF 0.1f					///< The magnetometer linear particle filter gain

#define IMU_AXES 6						///< The number of axis of the device

/**
 * \brief Structure containing the accelero, gyro and magnetometer sensors' gains
 */
typedef struct
{
	float bias[3];
	float scale_factor[3];
	float orientation[3];
}sensor_calib_t;

/**
 * \brief Structure containing the Attitude and Heading Reference System
 */
typedef struct
{
	UQuat_t		qe;						///< quaternion defining the Attitude estimation of the platform
	
	float		angular_speed[3];					///< Gyro rates
	float		linear_acc[3];					///< Acceleration WITHOUT gravity
	
	float		heading;				///< The heading of the platform
	UQuat_t		up_vec;					///< The quaternion of the up vector
	UQuat_t		north_vec;				///< The quaternion of the north vector
	
	uint32_t	last_update;			///< The time of the last IMU update in ms
	float		dt;						///< The time interval between two IMU updates
}AHRS_t;

/**
 * \brief The IMU structure
 */
typedef struct
{
	gyro_data_t raw_gyro, oriented_gyro, scaled_gyro;
	accelero_data_t raw_accelero, oriented_accelero, scaled_accelero;
	compass_data_t raw_compass, oriented_compass, scaled_compass;
	
	sensor_calib_t calib_gyro, calib_accelero, calib_compass;
	
	float dt;							///< The time interval between two IMU updates
	uint32_t last_update;				///< The time of the last IMU update in ms
} Imu_Data_t;

bool imu_last_update_init;				///< Variable to initialize the IMU

/** 
 * \brief	Initialize the IMU module
 *
 * \param	imu		the pointer to the IMU structure
 */
void imu_init(Imu_Data_t *imu);

/**
 * \brief	The function to be called to access the raw data of the IMU
 *
 * \param	imu		the pointer to the IMU structure
 */
void imu_get_raw_data(Imu_Data_t *imu);

/**
 * \brief	To calibrate the gyros at startup (not used know)
 *
 * \param	imu		the pointer to the IMU structure
 */
void imu_calibrate_gyros(Imu_Data_t *imu);

/**
 * \brief	Computes the attitude estimation, do the position estimation and the position correction
 *
 * \param	imu		the pointer structure of the IMU
 * \param	pos_est		the pointer to the position estimation structure
 * \param	barometer	the pointer to the barometer structure
 * \param	gps			the pointer to the GPS structure
 */
void imu_update(Imu_Data_t *imu);

void imu_relevel(Imu_Data_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
