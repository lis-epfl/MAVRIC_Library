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
 * \file qfilter.h
 *
 * This file implements a complementary filter for the attitude estimation
 */


#ifndef QFILTER_H_
#define QFILTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "compiler.h"
#include "coord_conventions.h"

#define GYRO_LPF 0.1				///< The gyroscope linear particle filter gain
#define ACC_LPF 0.05				///< The accelerometer linear particle filter gain
#define MAG_LPF 0.1					///< The magnetometer linear particle filter gain


#define GRAVITY 9.81				///< The gravity constant

/**
 * \brief The calibration level of the filter
 */
enum calibration_mode
{
	OFF,							///< Calibration level: No calibration 
	LEVELING,						///< Calibration level: leveling 
	LEVEL_PLUS_ACCEL				///< Calibration level: leveling plus acceleration
}; 

/**
 * \brief The structure for the quaternion-based attitude estimation
 */
typedef struct
{
	UQuat_t qe;						///< The quaternion of the attitude estimation
	UQuat_t up_vec;					///< The quaternion of the up vector
	UQuat_t north_vec;				///< The quaternion of the north vector
	
	float be[9];					///< The biais of the IMU and compass
	float sf[9];					///< The scale factors of the IMU and compass
	
	float om[3];					///< The 3D angular rates vector omega
	float a[3];						///< The 3D linear acceleration vector
	float mag[3];					///< The 3D magnetometer vector
	
	float kp;						///< The proportional gain for the acceleration correction of the angular rates
	float ki;						///< The integral gain for the acceleration correction of the biais
	float kp_mag;					///< The proportional gain for the magnetometer correction of the angular rates
	float ki_mag;					///< The integral gain for the magnetometer correction of the angular rates
	
	float raw_mag_mean[3];			///< The raw magnetometer values to compute the initial heading of the platform
	
	uint8_t calibration_level;		///< The level of calibration
	float heading;					///< The heading of the platform
	float acc_bf[3];				///< The 3D acceleration vector in body frame
} Quat_Attitude_t;

/**
 * \brief	Initialize the attitude estimation module
 *
 * \param	attitude		The pointer to the attitude structure
 * \param	scalefactor		The pointer to the scale factors structure of the IMU
 * \param	biais			The pointer to the biaises structure of the IMU
 */
void qfInit(Quat_Attitude_t *attitude, float *scalefactor, float *bias);

/**
 * \brief	Initialize the quaternion for the attitude estimation
 *
 * \param	attitude		The pointer to the attitude structure
 */
void initQuat(Quat_Attitude_t *attitude);

/**
 * \brief	Performs the attitude estimation via a complementary filter
 *
 * \param	attitude		The pointer to the attitude structure
 * \param	rates			The raw rates from the IMU
 * \param	dt				The time interval between two estimation loops
 */
void qfilter(Quat_Attitude_t *attitude, float *rates, float dt);

#ifdef __cplusplus
}
#endif

#endif /* QFILTER_H_ */
