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
 * \file stabilisation_copter.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#ifndef STABILISATION_COPTER_H_
#define STABILISATION_COPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation.h"
#include "imu.h"
#include "position_estimation.h"
#include "imu.h"
#include "servos.h"
#include "mavlink_waypoint_handler.h"

/**
 * \brief Structure containing the stacked controller
 */
typedef struct 
{
	stabiliser_t rate_stabiliser;								///< The rate controller structure
	stabiliser_t attitude_stabiliser;							///< The attitude controller structure
	stabiliser_t velocity_stabiliser;							///< The velocity controller structure
	stabiliser_t position_stabiliser;							///< The position controller structure
	float yaw_coordination_velocity;							///< the yaw coordination value in velocity control mode
} stabiliser_stack_copter_t;

/**
 * \brief Structure containing the pointers to the data needed in this module
 */
typedef struct
{
	stabiliser_stack_copter_t stabiliser_stack;		///< The pointer to the PID parameters values for the stacked controller 
	control_command_t* controls;					///< The pointer to the control structure
	const imu_t* imu;								///< The pointer to the IMU structure
	const ahrs_t* ahrs;								///< The pointer to the attitude estimation structure
	const position_estimator_t* pos_est;			///< The pointer to the position estimation structure
	servos_t* servos;								///< The pointer to the servos structure
} stabilise_copter_t;

/**
 * \brief Structure containing the configuration data
 */
typedef struct  
{
	stabiliser_stack_copter_t stabiliser_stack;					///< The pointer to the PID parameters values and output for the stacked controller
}stabilise_copter_conf_t;

/**
 * \brief							Initialize module stabilization
 *
 * \param	stabilisation_copter	The pointer to the stabilisation copter structure
 * \param	stabiliser_conf			The pointer to structure with all PID controllers
 * \param	control_input			The pointer to the controlling inputs
 * \param	imu						The pointer to the IMU structure
 * \param	ahrs		The pointer to the attitude estimation structure
 * \param	pos_est					The pointer to the position estimation structure
 * \param	servos					The pointer to the array of servos command values
 * \param	mavlink_stream			The pointer to the mavlink stream
 */
void stabilisation_copter_init(stabilise_copter_t* stabilisation_copter, stabilise_copter_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimator_t* pos_est,servos_t* servos, const mavlink_stream_t* mavlink_stream);

/**
 * \brief							Main Controller for controlling and stabilizing the quad in position (not using velocity control)
 *
 * \param	stabilisation_copter	The stabilisation structure
 * \param	input					The control command structure
 * \param	waypoint_handler		The waypoint handler structure, to get hold_position coordinates
 * \param	position_estimator		The position estimator structure to compute position error
 */
void stabilisation_copter_position_hold(stabilise_copter_t* stabilisation_copter, const control_command_t* input, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator);

/**
 * \brief							Main Controller for controlling and stabilizing the quad
 *
 * \param	stabilisation_copter	The stabilisation structure
 */
void stabilisation_copter_cascade_stabilise(stabilise_copter_t* stabilisation_copter);

/**
 * \brief							Mix to servo for quad configuration diagonal
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_diag_quad(control_command_t *control, servos_t* servos);

/**
 * \brief							Mix to servo for quad configuration cross
 *
 * \param	control					Pointer to controlling inputs
 * \param	servos					The array of servos structure
 */
void stabilisation_copter_mix_to_servos_cross_quad(control_command_t *control, servos_t* servos);

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_COPTER_H_ */