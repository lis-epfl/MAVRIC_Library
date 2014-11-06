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
 * \file velocity_controller_copter.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief A velocity controller for copters.
 *
 ******************************************************************************/

#include "velocity_controller_copter.h"
#include "quaternions.h"
#include "coord_conventions.h"
#include "maths.h"
#include "constants.h"
#include "vectors.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Converts velocity command from local to global frame 
 * 
 * \param controller 	Pointer to data structure
 * \param command[3] 	Velocity command in global frame (output)
 */
static void get_velocity_command_from_local_to_global(const velocity_controller_copter_t* controller, float command[3]);


/**
 * \brief Converts velocity command from semi-local frame to global frame
 * 
 * \details Semi local frame is global rotated around the vertical axis to match
 * the X axis with the current heading of the UAV
 * 
 * \param controller 	Pointer to data structure
 * \param command[3] 	Velocity command in global frame (output) 
 * 
 */
static void get_velocity_command_from_semilocal_to_global(const velocity_controller_copter_t* controller, float command[3]);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void get_velocity_command_from_local_to_global(const velocity_controller_copter_t* controller, float command[3])
{
	quaternions_rotate_vector( 	quaternions_inverse( controller->ahrs->qe ), 
								controller->velocity_command->xyz, 
								command);
}


static void get_velocity_command_from_semilocal_to_global(const velocity_controller_copter_t* controller, float command[3])
{
	aero_attitude_t semilocal_frame_rotation;
	quat_t q_semilocal;

	// Get current heading
	float heading = coord_conventions_quat_to_aero(controller->ahrs->qe).rpy[YAW];
	semilocal_frame_rotation.rpy[ROLL]  = 0.0f;
	semilocal_frame_rotation.rpy[PITCH] = 0.0f;
	semilocal_frame_rotation.rpy[YAW]   = heading;		

	// Get rotation quaternion from semilocal frame to global frame
	q_semilocal = coord_conventions_quaternion_from_aero( semilocal_frame_rotation );

	// Rotate command from semilocal to global
	quaternions_rotate_vector( 	quaternions_inverse( q_semilocal ), 	// TODO: Check why this is not "quaternions_inverse( q_semilocal )" here
	// quaternions_rotate_vector( 	q_semilocal, 							
								controller->velocity_command->xyz, 
								command );
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void velocity_controller_copter_init(velocity_controller_copter_t* controller, const velocity_controller_copter_conf_t* config, const ahrs_t* ahrs, const position_estimation_t* pos_est, const velocity_command_t* velocity_command, attitude_command_t* attitude_command, thrust_command_t* thrust_command)
{
	// Init dependencies
	controller->velocity_command 	= velocity_command;
	controller->attitude_command 	= attitude_command;
	controller->thrust_command 		= thrust_command;
	controller->ahrs 				= ahrs;
	controller->pos_est 			= pos_est;

	// Init hover point
	controller->thrust_hover_point = config->thrust_hover_point;

	// Init PID gains
	pid_controller_init(&controller->pid[X], &config->pid_config[X]);
	pid_controller_init(&controller->pid[Y], &config->pid_config[Y]);
	pid_controller_init(&controller->pid[Z], &config->pid_config[Z]);
}


void velocity_controller_copter_update(velocity_controller_copter_t* controller)
{
	float velocity_command_global[3];
	float errors[3];
	float thrust_vector[3];
	float thrust_norm;
	float thrust_dir[3];

	// Get the command velocity in global frame
	switch( controller->velocity_command->mode )
	{
		case VELOCITY_COMMAND_MODE_LOCAL:
			get_velocity_command_from_local_to_global( 	controller,
														velocity_command_global );
		break;

		case VELOCITY_COMMAND_MODE_SEMI_LOCAL:
			get_velocity_command_from_semilocal_to_global( 	controller,
															velocity_command_global );
		break;

		case VELOCITY_COMMAND_MODE_GLOBAL:
			velocity_command_global[X] = controller->velocity_command->xyz[X];
			velocity_command_global[Y] = controller->velocity_command->xyz[Y];
			velocity_command_global[Z] = controller->velocity_command->xyz[Z];
		break;
		
		default:
			velocity_command_global[X] = 0.0f;
			velocity_command_global[Y] = 0.0f;
			velocity_command_global[Z] = 0.0f;
		break;
	}

	// Compute errors
	errors[X] = velocity_command_global[X] - controller->pos_est->vel[X];
	errors[Y] = velocity_command_global[Y] - controller->pos_est->vel[Y];
	errors[Z] = velocity_command_global[Z] - controller->pos_est->vel[Z];		// WARNING: it was multiplied by (-1) in stabilisation_copter.c

	// Update PID
	thrust_vector[X] = pid_controller_update_dt( &controller->pid[X], errors[X], controller->ahrs->dt );				// should be multiplied by mass
	thrust_vector[Y] = pid_controller_update_dt( &controller->pid[Y], errors[Y], controller->ahrs->dt );				// should be multiplied by mass
	// thrust_vector[Z] = - GRAVITY + pid_controller_update_dt( &controller->pid[Z], errors[Z], controller->ahrs->dt );	// should be multiplied by mass
	thrust_vector[Z] = pid_controller_update_dt( &controller->pid[Z], errors[Z], controller->ahrs->dt );	// should be multiplied by mass

	// Compute the norm of the thrust that should be applied
	thrust_norm = vectors_norm(thrust_vector);

	// Compute the direction in which thrust should be apply
	thrust_dir[X] = thrust_vector[X] / thrust_norm;
	thrust_dir[Y] = thrust_vector[Y] / thrust_norm;
	thrust_dir[Z] = thrust_vector[Z] / thrust_norm;

	// Map thrust dir to attitude
	controller->attitude_command->mode 		 = ATTITUDE_COMMAND_MODE_RPY;
	controller->attitude_command->rpy[ROLL]  = maths_clip(thrust_vector[Y], 1);
	controller->attitude_command->rpy[PITCH] = - maths_clip(thrust_vector[X], 1);
	controller->attitude_command->rpy[YAW]   = 0.0f;

	// Map PID output to thrust
	// float max_thrust = 30.0f;			// 10 Newton max thrust
	// controller->thrust_command->thrust 	= maths_clip(thrust_norm/max_thrust, 1.0f) * 2.0f - 1; 
	// controller->thrust_command->thrust 	= maths_clip(thrust_norm, 1.0f) * 2.0f - 1; 
	// controller->thrust_command->thrust 	= - 1.0 + 2 * thrust_norm/max_thrust; 

	// // Map PID output to attitude
	// controller->attitude_command->mode 		 = ATTITUDE_COMMAND_MODE_RPY;
	// controller->attitude_command->rpy[ROLL]  = maths_clip(thrust_vector[Y], 1);
	// controller->attitude_command->rpy[PITCH] = - maths_clip(thrust_vector[X], 1);
	// controller->attitude_command->rpy[YAW]   = 0.0f;

	// Map PID output to thrust
	controller->thrust_command->thrust 	= controller->thrust_hover_point - thrust_vector[Z];
}
