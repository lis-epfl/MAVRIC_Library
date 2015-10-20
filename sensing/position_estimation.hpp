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


#include "state.hpp"
#include "gps_ublox.hpp"
#include "bmp085.hpp"
#include "data_logging.hpp"
#include "sonar.hpp"

extern "C" 
{
	#include <stdbool.h>
	#include "ahrs.h"
	#include "coord_conventions.h"
	#include "constants.h"
}

// leaky velocity integration as a simple trick to emulate drag and avoid too large deviations (loss per 1 second)
#define VEL_DECAY 0.0f
#define POS_DECAY 0.0f


/**
 * \brief The position estimator structure
 */
typedef struct
{
	global_position_t origin;	///<	Global coordinates of the local frame's origin (ie. local (0, 0, 0) expressed in the global frame)
	float gravity;				///<	value of the Gravity for position estimation correction
	bool fence_set;				
} position_estimation_conf_t;


/**
 * \brief The position estimator structure
 */
typedef struct
{
	float kp_vel_gps[3];					///< Gain to correct the velocity estimation from the GPS
	float kp_pos_gps[3];					///< Gain to correct the position estimation from the GPS
	float kp_alt_baro;						///< Gain to correct the Z position estimation from the barometer
	float kp_vel_baro;						///< Gain to correct the Z velocity estimation from the barometer
	float kp_alt_sonar;						///< Gain to correct the Z position estimation from the sonar
	float kp_vel_sonar;						///< Gain to correct the Z velocity estimation from the sonar

	uint32_t time_last_gps_posllh_msg;		///< Time at which we received the last GPS POSLLH message in ms
	uint32_t time_last_gps_velned_msg;		///< Time at which we received the last GPS VELNED message in ms
	uint32_t time_last_barometer_msg;		///< Time at which we received the last barometer message in ms
	bool init_gps_position;					///< Boolean flag ensuring that the GPS was initialized
	bool init_barometer;					///< Boolean flag ensuring that the barometer was initialized
	
	float vel_bf[3];						///< 3D velocity in body frame
	float vel[3];							///< 3D velocity in global frame

	float last_alt;							///< Value of the last altitude estimation
	float last_vel[3];						///< Last 3D velocity

	local_position_t local_position;		///< Local position
	local_position_t last_gps_pos;			///< Coordinates of the last GPS position
	
	bool fence_set;							///< Indicates if fence is set
	local_position_t fence_position;		///< Position of the fence

	float gravity;							///< Value of the gravity
	
	Barometer* barometer;					///< Pointer to the barometer structure
	const Sonar* sonar;						///< Pointer to the sonar structure
	const gps_t* gps;						///< Pointer to the GPS structure
	const ahrs_t* ahrs;						///< Pointer to the attitude estimation structure
	state_t* state;							///< Pointer to the state structure
	data_logging_t* stat_logging;			///< Pointer to the stat logging structure

	bool* nav_plan_active;					///< Pointer to the waypoint set flag
} position_estimation_t;


/**
 * \brief	Initialize the position estimation module
 *
 * \param	pos_est			Pointer to the position estimation structure
 * \param	config			Configuration for default home position and gravity value
 * \param	state			Pointer to the state structure
 * \param	barometer		Pointer to the barometer structure
 * \param	sonar 			Pointer to the sonar structure
 * \param	gps				Pointer to the GPS structure
 * \param	ahrs			Pointer to the attitude estimation structure
 * \param	stat_logging	Pointer to the stat logging structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool position_estimation_init(position_estimation_t* pos_est, const position_estimation_conf_t config, state_t* state, Barometer* barometer, const Sonar* sonar, const gps_t *gps, const ahrs_t *ahrs, data_logging_t* stat_logging);


/**
 * \brief	Reset the home position and altitude
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
void position_estimation_reset_home_altitude(position_estimation_t *pos_est);


/**
 * \brief	Position estimation update step, performing position estimation then position correction (function to be used)
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
void position_estimation_update(position_estimation_t *pos_est);

/**
 * \brief	Reset the origin of the fence (e.g. common for many entities or when armed)
 *
 * \param	pos_est					The pointer to the position estimation structure
 */
void position_estimation_set_new_fence_origin(position_estimation_t* pos_est);

#endif // POSITION_ESTIMATION_H__