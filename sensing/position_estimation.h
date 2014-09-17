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
 * \file position_estimation.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief This file performs the 3D position estimation, either by direct 
 * integration or with correction with the GPS and pos_est->barometer
 *
 ******************************************************************************/


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
#include "state.h"
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
	local_coordinates_t last_gps_pos;				///< The coordinates of the last GPS position
	
	float gravity;
	
	barometer_t* barometer;							///< The pointer to the barometer structure
	const gps_t* gps;								///< The pointer to the GPS structure
	const ahrs_t* ahrs;								///< The pointer to the attitude estimation structure
	const imu_t* imu;								///< The pointer to the IMU structure
	state_t* state;									///< The pointer to the state structure

	bool* nav_plan_active;							///< The pointer to the waypoint set flag
} position_estimator_t;


/**
 * \brief	Initialize the position estimation module
 *
 * \param	pos_est					The pointer to the position estimation structure
 * \param	state					The pointer to the state structure
 * \param	barometer				The pointer to the barometer structure
 * \param	gps						The pointer to the GPS structure
 * \param	ahrs					The pointer to the attitude estimation structure
 * \param	imu						The pointer to the IMU structure
 * \param	nav_plan_active			The pointer to the flag telling if there is a flight plan loaded
 * \param	home_lat				The value of the hard coded home latitude position
 * \param	home_lon				The value of the hard coded home longitude position
 * \param	home_alt				The value of the hard coded home altitude position
 * \param	gravity					The value of the gravity
 */
void position_estimation_init(position_estimator_t *pos_est,state_t* state, barometer_t *barometer, const gps_t *gps, const ahrs_t *ahrs, const imu_t *imu, bool* nav_plan_active, float home_lat, float home_lon, float home_alt, float gravity);


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