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
#include "qfilter.h"
#include "bmp085.h"
#include "position_estimation.h"
#include "gps_ublox.h"
#include "conf_platform.h"
#include "gyro.h"
#include "accelero.h"
#include "compass.h"

#define GYRO_LPF 0.1f					///< The gyroscope linear particle filter gain
#define ACC_LPF 0.05f					///< The accelerometer linear particle filter gain
#define MAG_LPF 0.1f					///< The magnetometer linear particle filter gain

#define IMU_AXES 6						///< The number of axis of the device

/**
 * \brief The IMU structure
 */
typedef struct
{
	Quat_Attitude_t attitude;			///< Attitude structure of the platform
	float raw_channels[9];				///< The array where the raw value of the IMU and compass are parsed
	float raw_bias[9];					///< The biaises of the IMU and compass
	float raw_scale[9];					///< The scales of the IMU and compass
	uint32_t last_update;				///< The time of the last IMU update in ms
	//uint8_t valid;						///< True if the message is valid (TODO: is it sill used?)
	float dt;							///< The time interval between two IMU updates
	//int8_t ready;							///< Is the IMU ready (TODO: is it still used?)
	gyro_data_t gyroData;
	accelero_data_t acceleroData;
	compass_data_t compassData;
} Imu_Data_t;

bool imu_last_update_init;				///< Variable to initialize the IMU

/** 
 * \brief	Initialize the IMU module
 *
 * \param	imu1		the pointer to the IMU structure
 */
void imu_init(Imu_Data_t *imu1);

/**
 * \brief	The function to be called to access the raw data of the IMU
 *
 * \param	imu1		the pointer to the IMU structure
 */
void imu_get_raw_data(Imu_Data_t *imu1);

/**
 * \brief	To calibrate the gyros at startup (not used know)
 *
 * \param	imu1		the pointer to the IMU structure
 */
void imu_calibrate_gyros(Imu_Data_t *imu1);

/**
 * \brief	Computes the attitude estimation, do the position estimation and the position correction
 *
 * \param	imu1		the pointer structure of the IMU
 * \param	pos_est		the pointer to the position estimation structure
 * \param	barometer	the pointer to the barometer structure
 * \param	gps			the pointer to the GPS structure
 */
void imu_update(Imu_Data_t *imu1, position_estimator_t *pos_est, pressure_data *barometer, gps_Data_type *gps);

/**
 * \brief	Computes the transition from raw values to scaled values
 *
 * \param	attitude	the pointer structure of the attitude
 * \param	rates		the array of angular rates (IMU), accelerations and magnetometer
 */
void imu_raw2scale(Quat_Attitude_t *attitude, float rates[9]);

#ifdef __cplusplus
}
#endif

#endif /* IMU_H_ */
