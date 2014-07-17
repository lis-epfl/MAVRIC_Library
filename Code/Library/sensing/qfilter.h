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
#include "imu.h"


/**
 * \brief The calibration level of the filter
 */
typedef enum
{
	OFF,							///< Calibration level: No calibration 
	LEVELING,						///< Calibration level: leveling 
	LEVEL_PLUS_ACCEL				///< Calibration level: leveling plus acceleration
} calibration_mode_t;


/**
 * \brief The structure for the quaternion-based attitude estimation
 */
typedef struct
{
	Imu_Data_t *imu;				///< Pointer to inertial sensors readout
	AHRS_t *attitude_estimation;	///< Pointer to estimated attiude
	
	float kp;						///< The proportional gain for the acceleration correction of the angular rates
	float ki;						///< The integral gain for the acceleration correction of the biais
	float kp_mag;					///< The proportional gain for the magnetometer correction of the angular rates
	float ki_mag;					///< The integral gain for the magnetometer correction of the angular rates
} qfilter_t;


/**
 * \brief	Initialize the attitude estimation module
 *
 * \param	attitude		The pointer to the attitude structure
 * \param	scalefactor		The pointer to the scale factors structure of the IMU
 * \param	biais			The pointer to the biaises structure of the IMU
 */
void qfilter_init(qfilter_t *attitude_filter, Imu_Data_t *imu, AHRS_t *attitude_estimation);


/**
 * \brief	Performs the attitude estimation via a complementary filter
 *
 * \param	qf		The pointer to the qfilter structure
 */
void qfilter_update(qfilter_t *qf);


#ifdef __cplusplus
}
#endif

#endif /* QFILTER_H_ */
