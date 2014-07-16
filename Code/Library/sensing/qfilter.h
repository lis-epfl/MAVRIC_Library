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

#include <stdint.h>
#include "coord_conventions.h"
#include "imu.h"


#define GRAVITY 9.81f				///< The gravity constant

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
	Imu_Data_t *imu1;
	AHRS_t *attitude_estimation;
	
	float kp;						///< The proportional gain for the acceleration correction of the angular rates
	float ki;						///< The integral gain for the acceleration correction of the biais
	float kp_mag;					///< The proportional gain for the magnetometer correction of the angular rates
	float ki_mag;					///< The integral gain for the magnetometer correction of the angular rates
	
	float raw_mag_mean[3];			///< The raw magnetometer values to compute the initial heading of the platform
	
	uint8_t calibration_level;		///< The level of calibration
} qfilter_t;

/**
 * \brief	Initialize the attitude estimation module
 *
 * \param	attitude		The pointer to the attitude structure
 * \param	scalefactor		The pointer to the scale factors structure of the IMU
 * \param	biais			The pointer to the biaises structure of the IMU
 */
void qfilter_init(qfilter_t *attitude_filter, Imu_Data_t *imu1, AHRS_t *attitude_estimation);

/**
 * \brief	Initialize the quaternion for the attitude estimation
 *
 * \param	attitude		The pointer to the attitude structure
 */
void qfilter_init_quaternion(qfilter_t *attitude_filter);

/**
 * \brief	Performs the attitude estimation via a complementary filter
 *
 * \param	attitude		The pointer to the attitude structure
 * \param	dt				The time interval between two estimation loops
 */
void qfilter_attitude_estimation(qfilter_t *attitude_filter, float dt);

#ifdef __cplusplus
}
#endif

#endif /* QFILTER_H_ */
