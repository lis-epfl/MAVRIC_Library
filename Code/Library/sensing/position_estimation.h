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
#include "bmp085.h"
#include "gps_ublox.h"
#include "coord_conventions.h"

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0f
#define POS_DECAY 0.0f

/**
 * \brief The position estimator structure
 */
typedef struct
{
	float kp_vel[3];					///< The gain to correct the velocity estimation from the GPS
	float kp_pos[3];					///< The gain to correct the position estimation from the GPS
	float kp_alt;						///< The gain to correct the Z position estimation from the GPS
	float kp_vel_baro;					///< The gain to correct the position estimation from the barometer

	uint32_t time_last_gps_msg;			///< The time at which we received the last GPS message in ms
	uint32_t time_last_barometer_msg;	///< The time at which we received the last barometer message in ms
	bool init_gps_position;				///< The boolean flag ensuring that the GPS was initialized
	bool init_barometer;				///< The boolean flag ensuring that the barometer was initialized
	
	float vel_bf[3];					///< The 3D velocity in body frame
	float vel[3];						///< The 3D velocity in global frame

	float pos_correction[3];			///< The 3D array to store the difference between the position estimation and the GPS estimated position
	float last_alt;						///< The value of the last altitude estimation
	float last_vel[3];					///< The last 3D velocity

	local_coordinates_t localPosition;	///< The local position
	local_coordinates_t lastGpsPos;		///< The coordinates of the last GPS position
	
	float gravity;
	
	pressure_data_t *barometer;
	gps_Data_type_t *gps;
	AHRS_t *attitude_estimation;
	Imu_Data_t* imu;
	local_coordinates_t *sim_local_position;
	bool *waypoint_set;

} position_estimator_t;


/**
 * \brief	Initialize the position estimation module
 *
 * \param	pos_est					The pointer to the position estimation structure
 * \param	barometer				The pointer to the barometer structure
 * \param	gps						The pointer to the GPS structure
 * \param	attitude_estimation		The pointer to the attitude estimation structure
 * \param	sim_local_position		The local position of the simulator
 */
void position_estimation_init(position_estimator_t *pos_est, pressure_data_t *barometer, gps_Data_type_t *gps, AHRS_t *attitude_estimation, Imu_Data_t *imu, local_coordinates_t *sim_local_position, bool* waypoint_set, mavlink_communication_t *mavlink_communication, float home_lat, float home_lon, float home_alt, float gravity);


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


#ifdef __cplusplus
}
#endif

#endif // POSITION_ESTIMATION_H__