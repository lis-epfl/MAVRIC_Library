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
 * \file throw_recovery_state_machine.c
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

#include "throw_recovery_state_machine.h"
#include "launch_detection_default_config.h"
#include "print_util.h"

#define RP_THRESHOLD 0.08f 				///< Roll and Pitch threshold for STATE_ATTITUDE_CONTROL
#define ANGULAR_SPEED_THRESHOLD 0.5f	///< Angular speed threshold for STATE_ATTITUDE_CONTROL

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool throw_recovery_state_machine_init(throw_recovery_state_machine_t * state_machine, const remote_t * remote, const imu_t * imu, const ahrs_t * ahrs, const position_estimation_t * pos_est, stabilisation_copter_t * stabilisation_copter, navigation_t * navigation)
{
	bool init_success = true;

	// Init state machine dependencies
	state_machine->stabilisation_copter_conf = &stabilisation_copter_default_config;
	state_machine->remote = remote;
	state_machine->imu = imu;
	state_machine->ahrs = ahrs;
	state_machine->pos_est = pos_est;
	state_machine->stabilisation_copter = stabilisation_copter;
	state_machine->navigation = navigation;
	launch_detection_init(&state_machine->ld, &launch_detection_default_config);

	// Init state machine
	state_machine->state = STATE_IDLE;
	state_machine->enabled = true;
	state_machine->debug = false;
	state_machine->is_initialised = false;

	return init_success;
}

task_return_t throw_recovery_state_machine_update(throw_recovery_state_machine_t * state_machine, control_command_t * controls)
{
	bool switch_enabled = throw_recovery_state_machine_switch_enabled(state_machine);
	bool is_armed = state_machine->debug ? true : state_machine->imu->state->mav_mode.ARMED == ARMED_ON;

	switch (state_machine->state) 
	{
		case STATE_IDLE:
			/**************************** Dependecies *****************************/
			/***************************** Commands *****************************/
			/***************************** Controls *****************************/
			/*************************** Transitions ****************************/
			if (is_armed && switch_enabled && (!state_machine->is_initialised))
			{
				state_machine->is_initialised = true;
				state_machine->state = STATE_LAUNCH_DETECTION;
				controls->velocity_control_mode = VELOCITY_MODE;

				state_machine->transition_times[STATE_IDLE] = time_keeper_get_millis();
			} 
		break;

		case STATE_LAUNCH_DETECTION:
			/**************************** Dependecies *****************************/
			launch_detection_update(&state_machine->ld, (state_machine->imu->scaled_accelero.data));
			/***************************** Commands *****************************/
			/***************************** Controls *****************************/
			/*************************** Transitions ****************************/
			if (state_machine->ld.status == LAUNCHING)
			{
				state_machine->state = STATE_ATTITUDE_CONTROL;

				state_machine->transition_times[STATE_LAUNCH_DETECTION] = time_keeper_get_millis() - state_machine->transition_times[STATE_IDLE];
			}
		break;

		case STATE_ATTITUDE_CONTROL:
			/**************************** Dependecies *****************************/
			state_machine->stabilisation_copter_conf = &stabilisation_copter_custom_config;
			/***************************** Commands *****************************/
			controls->rpy[ROLL] = 0.0f;
			controls->rpy[PITCH] = 0.0f;
			controls->thrust = state_machine->stabilisation_copter_conf->thrust_hover_point;
			/***************************** Controls *****************************/
			controls->control_mode = ATTITUDE_COMMAND_MODE;
			controls->yaw_mode = YAW_ABSOLUTE;
			/*************************** Transitions ****************************/
			bool roll_predicate = abs(state_machine->ahrs->qe.v[0]) < RP_THRESHOLD;
			bool pitch_predicate = abs(state_machine->ahrs->qe.v[1]) < RP_THRESHOLD;
			bool angular_speed_x_predicate = abs(state_machine->ahrs->angular_speed[0]) < ANGULAR_SPEED_THRESHOLD;
			bool angular_speed_y_predicate = abs(state_machine->ahrs->angular_speed[1]) < ANGULAR_SPEED_THRESHOLD;

			if (roll_predicate && pitch_predicate && angular_speed_x_predicate && angular_speed_y_predicate)
			{
				state_machine->state = STATE_VERTICAL_VELOCITY;

				state_machine->transition_times[STATE_ATTITUDE_CONTROL] = time_keeper_get_millis() - state_machine->transition_times[STATE_LAUNCH_DETECTION];
			}
		break;

		case STATE_VERTICAL_VELOCITY:
			/**************************** Dependecies *****************************/
			state_machine->stabilisation_copter_conf = &stabilisation_copter_default_config;
			/***************************** Commands *****************************/
			// controls->tvel[X] = 0.0f;
			// controls->tvel[Y] = 0.0f;
			controls->tvel[Z] = 0.0f;
			/***************************** Controls *****************************/
			controls->control_mode = VELOCITY_COMMAND_MODE;
			controls->yaw_mode = YAW_ABSOLUTE;
			controls->velocity_control_mode = VERTICAL_VELOCITY_MODE;
			/*************************** Transitions ****************************/
			bool est_speed_z_predicate = abs(state_machine->pos_est->vel[Z]) < 0.3f ;
			bool err_z_predicate = abs(state_machine->stabilisation_copter->stabiliser_stack.velocity_stabiliser.thrust_controller.error) < 0.3f;
			bool acc_z_predicate = abs(state_machine->imu->scaled_accelero.data[Z]) < 0.3f;
			
			if (est_speed_z_predicate && err_z_predicate && acc_z_predicate)
			{
				state_machine->state = STATE_HEIGHT_CONTROL;

				state_machine->navigation->state->in_the_air = true;
				state_machine->navigation->throw_recovery_position_set = false;
				state_machine->navigation->throw_recovery_enabled = true;
				state_machine->navigation->auto_landing = false;

				state_machine->transition_times[STATE_VERTICAL_VELOCITY] = time_keeper_get_millis() - state_machine->transition_times[STATE_ATTITUDE_CONTROL];
			}
		break;

		case STATE_HEIGHT_CONTROL:
			/**************************** Dependecies *****************************/
			/***************************** Commands *****************************/
			// commands set by controls_nav
			/***************************** Controls *****************************/
			controls->control_mode = VELOCITY_COMMAND_MODE;
			controls->yaw_mode = YAW_ABSOLUTE;
			controls->velocity_control_mode = VERTICAL_VELOCITY_MODE;
			/*************************** Transitions ****************************/
			bool altitude_predicate = abs(state_machine->pos_est->local_position.pos[2]) < 2.1f;

			if (altitude_predicate)
			{
				state_machine->state = STATE_HORIZONTAL_VELOCITY;

				controls->velocity_control_mode = VELOCITY_MODE;

				state_machine->transition_times[STATE_HEIGHT_CONTROL] = time_keeper_get_millis() - state_machine->transition_times[STATE_VERTICAL_VELOCITY];
			}
		break;

		case STATE_HORIZONTAL_VELOCITY:
			/**************************** Dependecies *****************************/
			/***************************** Commands *****************************/
			controls->tvel[X] = 0.0f;
			controls->tvel[Y] = 0.0f;
			/***************************** Controls *****************************/
			controls->control_mode = VELOCITY_COMMAND_MODE;
			controls->yaw_mode = YAW_ABSOLUTE;
			controls->velocity_control_mode = VELOCITY_MODE;
			/*************************** Transitions ****************************/
			bool est_speed_xy_predicate = abs(state_machine->pos_est->vel[X]) < 0.3f && abs(state_machine->pos_est->vel[Y]) < 0.3f;
			bool acc_xy_predicate = abs(state_machine->imu->scaled_accelero.data[X]) < 0.3f && abs(state_machine->imu->scaled_accelero.data[Y]) < 0.3f;

			if (est_speed_xy_predicate && acc_xy_predicate)
			{
				state_machine->state = STATE_POSITION_LOCKING;

				state_machine->transition_times[STATE_HORIZONTAL_VELOCITY] = time_keeper_get_millis() - state_machine->transition_times[STATE_HEIGHT_CONTROL];
			}
		break;

		case STATE_POSITION_LOCKING:
			/**************************** Dependecies *****************************/
			/***************************** Commands *****************************/
			/***************************** Controls *****************************/
			controls->control_mode = VELOCITY_COMMAND_MODE;
			controls->yaw_mode = YAW_ABSOLUTE;
			controls->velocity_control_mode = VELOCITY_MODE;
			/*************************** Transitions ****************************/

		break;
	}

	return TASK_RUN_SUCCESS;
}

void throw_recovery_state_machine_reset(throw_recovery_state_machine_t * state_machine)
{
	state_machine->state = STATE_IDLE;
	state_machine->is_initialised = false;
	state_machine->ld.status = false;
	state_machine->navigation->throw_recovery_enabled = false;

	for (int i=0; i<6; i++)
	{
		state_machine->transition_times[i] = 0;
	}

	pid_controller_reset_integrator(&state_machine->stabilisation_copter->stabiliser_stack.velocity_stabiliser.thrust_controller);
}

bool throw_recovery_state_machine_should_reset(throw_recovery_state_machine_t * state_machine)
{
	return ((state_machine->enabled == true) && (state_machine->debug != true) && (state_machine->is_initialised == true));
}

bool throw_recovery_state_machine_switch_enabled(throw_recovery_state_machine_t * state_machine)
{
	return (state_machine->debug ? true : ((int32_t)(state_machine->remote->channels[CHANNEL_AUX1] + 1.0f) > 0)) ;
}