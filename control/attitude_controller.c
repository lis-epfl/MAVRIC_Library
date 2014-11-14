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
 * \file attitude_controller.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A cascaded controller for attitude & rate control.
 *
 ******************************************************************************/


#include "attitude_controller.h"
#include "constants.h"
#include "time_keeper.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief 				Performs inner stabilisation loop on angular rate
 * 
 * \param controller	Pointer to attitude controller
 */
static void attitude_controller_rate_loop(attitude_controller_t* controller);


/**
 * \brief 				Performs outer stabilisation loop on attitude angles
 * 
 * \param controller	Pointer to attitude controller
 */
static void attitude_controller_angle_loop(attitude_controller_t* controller);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void attitude_controller_rate_loop(attitude_controller_t* controller)
{
	float errors[3];

	// Get errors on rate
	errors[ROLL]  = controller->rate_command->xyz[ROLL]  - controller->ahrs->angular_speed[ROLL];
	errors[PITCH] = controller->rate_command->xyz[PITCH] - controller->ahrs->angular_speed[PITCH];
	errors[YAW]   = controller->rate_command->xyz[YAW]   - controller->ahrs->angular_speed[YAW];

	// Update PIDs
	controller->torque_command->xyz[ROLL]  = pid_controller_update_dt(&(controller->rate_pid[ROLL]),  errors[ROLL],  controller->ahrs->dt);
	controller->torque_command->xyz[PITCH] = pid_controller_update_dt(&(controller->rate_pid[PITCH]), errors[PITCH], controller->ahrs->dt);
	controller->torque_command->xyz[YAW]   = pid_controller_update_dt(&(controller->rate_pid[YAW]),   errors[YAW], 	 controller->ahrs->dt);
}


static void attitude_controller_angle_loop(attitude_controller_t* controller)
{
	float errors[3];

	// Get attitude command
	switch ( controller->attitude_command->mode )
	{
		case ATTITUDE_COMMAND_MODE_QUATERNION:
			attitude_error_estimator_set_quat_ref(	&controller->attitude_error_estimator,
													controller->attitude_command->quat );
			break;

		case ATTITUDE_COMMAND_MODE_RPY:
			attitude_error_estimator_set_quat_ref_from_rpy( &controller->attitude_error_estimator,
															controller->attitude_command->rpy );
			break;
	}

	// Get local angular errors
	attitude_error_estimator_update( &controller->attitude_error_estimator );
	errors[ROLL] 	= controller->attitude_error_estimator.rpy_errors[ROLL];
	errors[PITCH] 	= controller->attitude_error_estimator.rpy_errors[PITCH];
	errors[YAW] 	= controller->attitude_error_estimator.rpy_errors[YAW];	

	// Update PIDs
	controller->rate_command->xyz[ROLL]  = pid_controller_update_dt(&(controller->angle_pid[ROLL]),  errors[ROLL], 	controller->ahrs->dt);
	controller->rate_command->xyz[PITCH] = pid_controller_update_dt(&(controller->angle_pid[PITCH]), errors[PITCH], controller->ahrs->dt);
	controller->rate_command->xyz[YAW] 	 = pid_controller_update_dt(&(controller->angle_pid[YAW]),   errors[YAW], 	controller->ahrs->dt);
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void attitude_controller_init(attitude_controller_t* controller, const attitude_controller_conf_t* config, const ahrs_t* ahrs, attitude_command_t* attitude_command, rate_command_t* rate_command, torque_command_t* torque_command)
{
	// Init dependencies
	controller->attitude_command 	= attitude_command;
	controller->rate_command 		= rate_command;
	controller->torque_command 		= torque_command;
	controller->ahrs 				= ahrs;

	// Init mode
	controller->mode = ATTITUDE_CONTROLLER_MODE_DEFAULT;

	// Init attitude error estimator
	attitude_error_estimator_init(&controller->attitude_error_estimator, ahrs);

	// Init rate gains
	pid_controller_init(&controller->rate_pid[ROLL],  &config->rate_pid_config[ROLL]);
	pid_controller_init(&controller->rate_pid[PITCH], &config->rate_pid_config[PITCH]);
	pid_controller_init(&controller->rate_pid[YAW],   &config->rate_pid_config[YAW]);
	
	// Init angle gains
	pid_controller_init(&controller->angle_pid[ROLL],  &config->angle_pid_config[ROLL]);
	pid_controller_init(&controller->angle_pid[PITCH], &config->angle_pid_config[PITCH]);
	pid_controller_init(&controller->angle_pid[YAW],   &config->angle_pid_config[YAW]);
}


void attitude_controller_update(attitude_controller_t* controller)
{
	switch( controller->mode )
	{
		case ATTITUDE_CONTROLLER_MODE_DEFAULT:
			attitude_controller_angle_loop(controller);
			attitude_controller_rate_loop(controller);
		break;

		case ATTITUDE_CONTROLLER_MODE_RATE_ONLY:
			attitude_controller_rate_loop(controller);
		break;
	}	
}