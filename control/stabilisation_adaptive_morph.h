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
 * \file stabilisation_adaptive_morph.h
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#ifndef STABILISATION_ADAPTIVE_MORPH_H_
#define STABILISATION_ADAPTIVE_MORPH_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation.h"
#include "imu.h"
#include "position_estimation.h"
#include "servos.h"
#include "mavlink_waypoint_handler.h"
#include "servos_mix_adaptive_morph.h"
#include "airspeed_analog.h"



/**
 * \brief Structure containing the stacked controller
 */
typedef struct 
{
	stabiliser_t rate_stabiliser;								///< The rate controller structure
	stabiliser_t attitude_stabiliser;							///< The attitude controller structure
	stabiliser_t velocity_stabiliser;							///< The velocity controller structure
	stabiliser_t position_stabiliser;							///< The position controller structure
} stabiliser_stack_adaptive_morph_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
	stabiliser_stack_adaptive_morph_t stabiliser_stack;					///< The pointer to the PID parameters values for the stacked controller 
	control_command_t* controls;								///< The pointer to the control structure
	const imu_t* imu;											///< The pointer to the IMU structure
	const ahrs_t* ahrs;											///< The pointer to the attitude estimation structure
	const position_estimation_t* pos_est;						///< The pointer to the position estimation structure
	const airspeed_analog_t* airspeed_analog;					///< The pointer to the analog airspeed sensor structure
	servos_t* servos;											///< The pointer to the servos structure
	servo_mix_adaptive_morph_t* servo_mix;								///< The pointer to the servos mixer
	float thrust_apriori;										///< A priori on the thrust for velocity control
	int32_t tuning;												///< Are we tuning the controllers?		0: nothing		1: rate		2: attitude
	int32_t tuning_axis;										///< Which axis are we tuning ?			0: roll			1: pitch
	int32_t tuning_steps;										///< Is the user allowed to create steps with the remote ?
	float pitch_up;												///< Up value for the steps in pitch
	float pitch_down;											///< Down value for the steps in pitch
	float roll_right;											///< Right value for the steps in roll
	float roll_left;											///< Left value for the steps in roll
} stabilisation_adaptive_morph_t;

/**
 * \brief Structure containing the configuration data
 */
typedef struct  
{
	float thrust_apriori;										///< A priori thrust
	stabiliser_stack_adaptive_morph_t stabiliser_stack;					///< The pointer to the PID parameters values and output for the stacked controller
	int32_t tuning;												///< Are we tuning the controllers?
	int32_t tuning_axis;										///< Which axis are we tuning ?
	int32_t tuning_steps;										///< Is the user allowed to create steps with the remote ?
	float pitch_up;												///< Up value for the steps in pitch
	float pitch_down;											///< Down value for the steps in pitch
	float roll_right;											///< Right value for the steps in roll
	float roll_left;											///< Left value for the steps in roll
} stabilisation_adaptive_morph_conf_t;

/**
 * \brief							Initialize module stabilization
 *
 * \param	stabilisation_adaptive_morph		The pointer to the stabilisation adaptive_morph structure
 * \param	stabiliser_conf			The pointer to structure with all PID controllers
 * \param	control_input			The pointer to the controlling inputs
 * \param	imu						The pointer to the IMU structure
 * \param	ahrs					The pointer to the attitude estimation structure
 * \param	pos_est					The pointer to the position estimation structure
 * \param	airspeed_analog			The pointer to the analog airspeed sensor structure
 * \param	servos					The pointer to the array of servos command values
 * \param	servo_mix				The pointer to the servo mix structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool stabilisation_adaptive_morph_init(stabilisation_adaptive_morph_t* stabilisation_adaptive_morph, stabilisation_adaptive_morph_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimation_t* pos_est, const airspeed_analog_t* airspeed_analog, servos_t* servos, servo_mix_adaptive_morph_t* servo_mix);

/**
 * \brief						Main Controller for controlling and stabilizing the adaptive_morph
 *
 * \param	stabilisation_adaptive_morph	The stabilisation structure
 */
void stabilisation_adaptive_morph_cascade_stabilise(stabilisation_adaptive_morph_t* stabilisation_adaptive_morph);



#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_ADAPTIVE_MORPH_H_ */