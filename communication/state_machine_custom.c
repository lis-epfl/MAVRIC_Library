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
 * \file state_machine.c
 *
 * \author MAV'RIC Team
 * \author Dylan Bourgeois
 *   
 * \brief Handles transitions between states defined in paper :
 * 
 *		Automatic Re-Initialization and Failure Recovery for Aggressive
 *		Flight with a Monocular Vision-Based Quadrotor
 *		M. Faessler, F. Fontana, C. Forster, D. Scaramuzza
 *		IEEE International Conference on Robotics and Automation (ICRA), Seattle, 2015.
 * 		http://rpg.ifi.uzh.ch/docs/ICRA15_Faessler.pdf
 *
 ******************************************************************************/

#include "stabilisation_copter_default_config.h"

#include "state_machine_custom.h"
#include "launch_detection_default_config.h"
#include "print_util.h"

#define RP_THRESHOLD 0.08f
#define ANGULAR_SPEED_THRESHOLD 0.5f

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_machine_custom_init(state_machine_custom_t * state_machine, remote_t * remote, imu_t * imu, ahrs_t * ahrs, position_estimation_t * pos_est, stabilisation_copter_t * stabilisation_copter, navigation_t * navigation)
{
	bool init_success = true;

	state_machine->state = STATE_IDLE;
	state_machine->enabled = 0;
	state_machine->debug = 0;

	state_machine->stabilisation_copter_conf = &stabilisation_copter_default_config;

	state_machine->remote = remote;
	state_machine->imu = imu;
	state_machine->ahrs = ahrs;
	state_machine->pos_est = pos_est;
	state_machine->stabilisation_copter = stabilisation_copter;
	state_machine->navigation = navigation;

	launch_detection_init(&state_machine->ld, &launch_detection_default_config);

	return init_success;
}

task_return_t state_machine_custom_update(state_machine_custom_t * state_machine, control_command_t * controls)
{
	bool switch_enabled = state_machine->debug ? 1 : ((int32_t)(state_machine->remote->channels[CHANNEL_AUX1] + 1.0f) > 0);
	bool is_armed = state_machine->debug ? 1 : state_machine->imu->state->mav_mode.ARMED == ARMED_ON;

	switch (state_machine->state) 
	{
		case STATE_IDLE:
			if (is_armed && switch_enabled && (state_machine->enabled==0))
			{
				state_machine->enabled = 1;
				state_machine->state = STATE_LAUNCH_DETECTION;
			} 
		break;

		case STATE_LAUNCH_DETECTION:

			launch_detection_update(&state_machine->ld, (state_machine->imu->scaled_accelero.data));

			if (state_machine->ld.status == LAUNCHING)
			{
				state_machine->state = STATE_ATTITUDE_CONTROL;
			}
		break;

		case STATE_ATTITUDE_CONTROL:
			state_machine->stabilisation_copter_conf = &stabilisation_copter_custom_config;
			controls->rpy[ROLL] = 0.0f;
			controls->rpy[PITCH] = 0.0f;
			controls->thrust = state_machine->stabilisation_copter_conf->thrust_hover_point;
			controls->control_mode = ATTITUDE_COMMAND_MODE;
			controls->yaw_mode = YAW_RELATIVE;

			bool roll_predicate = state_machine->ahrs->qe.v[0] < RP_THRESHOLD;
			bool pitch_predicate = state_machine->ahrs->qe.v[1] < RP_THRESHOLD;
			bool angular_speed_x_predicate = state_machine->ahrs->angular_speed[0] < ANGULAR_SPEED_THRESHOLD;
			bool angular_speed_y_predicate = state_machine->ahrs->angular_speed[1] < ANGULAR_SPEED_THRESHOLD;

			if (roll_predicate && pitch_predicate && angular_speed_x_predicate && angular_speed_y_predicate)
			{
				state_machine->state = STATE_VERTICAL_VELOCITY;
			}
		break;

		case STATE_VERTICAL_VELOCITY:
			state_machine->stabilisation_copter_conf = &stabilisation_copter_default_config;

			controls->tvel[X] = 0.0f;
			controls->tvel[Y] = 0.0f;
			controls->tvel[Z] = 0.0f;
			controls->control_mode = VELOCITY_COMMAND_MODE;

			bool est_speed_predicate = state_machine->pos_est->vel[2] < 0.3f;
			bool err_predicate = state_machine->stabilisation_copter->stabiliser_stack.velocity_stabiliser.thrust_controller.error < 0.3f;
			bool acc_predicate = state_machine->imu->scaled_accelero.data[Z] < 0.3f;
			
			if (est_speed_predicate && err_predicate && acc_predicate)
			{
				state_machine->state = STATE_HEIGHT_CONTROL;

				state_machine->navigation->state->in_the_air = true;
				state_machine->navigation->auto_landing_behavior = HEIGHT_CONTROL;
				state_machine->navigation->auto_landing = true;
				state_machine->navigation->auto_landing_next_state = false;
			}
		break;

		case STATE_HEIGHT_CONTROL:
			controls->control_mode = VELOCITY_COMMAND_MODE;
			controls->yaw_mode = YAW_RELATIVE;

			// bool altitude_predicate = abs(state_machine->pos_est->local_position.pos[2]) < 3.0f;
			// if (altitude_predicate)
			// {
			// 	state_machine->state = STATE_HORIZONTAL_VELOCITY;
			// }
		break;

		case STATE_HORIZONTAL_VELOCITY:
			// controls->tvel[X] = 0.0f;
			// controls->tvel[Y] = 0.0f;

			// controls->control_mode = VELOCITY_COMMAND_MODE;
		break;

		case STATE_POSITION_LOCKING:
		break;
	}

	return TASK_RUN_SUCCESS;
}

void state_machine_custom_reset(state_machine_custom_t * state_machine)
{
	state_machine->state = STATE_IDLE;
	state_machine->enabled = 0;
	state_machine->ld.status = 0;

	pid_controller_reset_integrator(&state_machine->stabilisation_copter->stabiliser_stack.velocity_stabiliser.thrust_controller);
}