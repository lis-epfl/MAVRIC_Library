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
 * \file ahrs.h
 *
 * This file implements attitude estimation data structure
 */


#ifndef AHRS_H_
#define AHRS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "quaternions.h"
#include "scheduler.h"


/**
 * \brief Structure containing the Attitude and Heading Reference System
 */
typedef struct
{
	UQuat_t		qe;							///< quaternion defining the Attitude estimation of the platform
	
	float		angular_speed[3];			///< Gyro rates
	float		linear_acc[3];				///< Acceleration WITHOUT gravity
	
	float		heading;					///< The heading of the platform
	UQuat_t		up_vec;						///< The quaternion of the up vector
	UQuat_t		north_vec;					///< The quaternion of the north vector
	
	uint32_t	last_update;				///< The time of the last IMU update in ms
	float		dt;							///< The time interval between two IMU updates
	
	const mavlink_stream_t* mavlink_stream;		///< The pointer to the mavlink stream
} ahrs_t;


/**
 * \brief   Initialiases the ahrs structure
 * 
 * \param  ahrs 				Pointer to ahrs structure
 * \param  mavlink_stream 	Pointer to mavlin kstream structure
 */
void ahrs_init(ahrs_t* ahrs, mavlink_stream_t* mavlink_stream);


/**
 * \brief	Task to send the mavlink attitude message
 * 
 * \param	ahrs		The pointer to the attitude estimation
 *
 * \return	The status of execution of the task
 */
task_return_t ahrs_send_attitude(ahrs_t* ahrs);


/**
 * \brief	Task to send the mavlink quaternion attitude message
 * 
 * \param	ahrs		The pointer to the attitude estimation
 *
 * \return	The status of execution of the task
 */
task_return_t ahrs_send_attitude_quaternion(ahrs_t* ahrs);


#ifdef __cplusplus
}
#endif

#endif /* AHRS_H_ */
