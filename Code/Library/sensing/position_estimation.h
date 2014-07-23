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
 * \file position_estimation.h
 *
 * This file performs the 3D position estimation, either by direct integration or with correction with the GPS and barometer
 */


#ifndef POSITION_ESTIMATION_H__
#define POSITION_ESTIMATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "imu.h"
#include "ahrs.h"
#include "bmp085.h"
#include "gps_ublox.h"
#include "coord_conventions.h"
#include "mavlink_communication.h"
#include "tasks.h"

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0f
#define POS_DECAY 0.0f

/**
 * \brief The position estimator structure
 */
typedef struct
{
	float kp_vel_gps[3];							///< The gain to correct the velocity estimation from the GPS
	float kp_pos_gps[3];							///< The gain to correct the position estimation from the GPS
	float kp_alt_baro;								///< The gain to correct the Z position estimation from the barometer
	float kp_vel_baro;								///< The gain to correct the position estimation from the barometer

	uint32_t time_last_gps_msg;						///< The time at which we received the last GPS message in ms
	uint32_t time_last_barometer_msg;				///< The time at which we received the last barometer message in ms
	bool init_gps_position;							///< The boolean flag ensuring that the GPS was initialized
	bool init_barometer;							///< The boolean flag ensuring that the barometer was initialized
	
	float vel_bf[3];								///< The 3D velocity in body frame
	float vel[3];									///< The 3D velocity in global frame

	float last_alt;									///< The value of the last altitude estimation
	float last_vel[3];								///< The last 3D velocity

	local_coordinates_t local_position;				///< The local position
	local_coordinates_t lastGpsPos;					///< The coordinates of the last GPS position
	
	float gravity;
	
	pressure_data_t*  	 	barometer;				///< The pointer to the barometer structure
	const gps_t* 	gps;					///< The pointer to the GPS structure
	const ahrs_t* 		 	ahrs;					///< The pointer to the attitude estimation structure
	const imu_t* 		 	imu;					///< The pointer to the IMU structure
	const mavlink_stream_t* mavlink_stream;			///< The pointer to the mavlink stream structure

	bool* waypoint_set;								///< The pointer to the waypoint set flag
} position_estimator_t;


/**
 * \brief	Initialize the position estimation module
 *
 * \param	pos_est					The pointer to the position estimation structure
 * \param	barometer				The pointer to the barometer structure
 * \param	gps						The pointer to the GPS structure
 * \param	ahrs		The pointer to the attitude estimation structure
 * \param	imu						The pointer to the IMU structure
 * \param   mavlink_stream			The pointer to the mavlink stream structure
 * \param	waypoint_set			The pointer to the flag telling if there is a flight plan loaded
 * \param	mavlink_handler			The pointer to the mavlink message handler
 * \param	home_lat				The value of the hard coded home latitude position
 * \param	home_lon				The value of the hard coded home longitude position
 * \param	home_alt				The value of the hard coded home altitude position
 * \param	gravity					The value of the gravity
 */
void position_estimation_init(position_estimator_t *pos_est, pressure_data_t *barometer, gps_t *gps, ahrs_t *ahrs, imu_t *imu, const mavlink_stream_t* mavlink_stream, bool* waypoint_set, mavlink_message_handler_t *mavlink_handler, float home_lat, float home_lon, float home_alt, float gravity);


/**
 * \brief	Reset the home position and altitude
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
void position_estimation_reset_home_altitude(position_estimator_t *pos_est);


/**
 * \brief	Position estimation update step, performing position estimation then position correction (function to be used)
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
void position_estimation_update(position_estimator_t *pos_est);


/**
 * \brief	Task to send the mavlink position estimation message
 *
 * \param	pos_est					The pointer to the position estimation structure
 * 
 * \return	The status of execution of the task
 */
task_return_t position_estimation_send_position(position_estimator_t* pos_est);


/**
 * \brief	Task to send the mavlink GPS global position message
 * 
 * \param	pos_est					The pointer to the position estimation structure
 *
 * \return	The status of execution of the task
 */
task_return_t position_estimation_send_global_position(position_estimator_t* pos_est);


#ifdef __cplusplus
}
#endif

#endif // POSITION_ESTIMATION_H__