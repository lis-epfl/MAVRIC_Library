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

// #include "stabilisation_copter_default_config.h"
// #include "stabilisation_copter_custom_config.h"

#include "state_machine_custom.h"
#include "launch_detection_default_config.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Resets the state machine
 *
 * \param	state_machine			The pointer to the state_machine structure
 */
void state_machine_reset(state_machine_custom_t * state_machine);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
void state_machine_reset(state_machine_custom_t * state_machine)
{
	state_machine->state = STATE_IDLE;
	state_machine->enabled = 0;
	state_machine->ld.status = 0;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_machine_custom_init(state_machine_custom_t * state_machine, remote_t * remote, imu_t * imu)
{
	bool init_success = true;

	state_machine->state = STATE_IDLE;
	state_machine->enabled = 0;

	state_machine->remote = remote;
	
	launch_detection_init(&state_machine->ld, &launch_detection_default_config);

	return init_success;
}

task_return_t state_machine_custom_update(state_machine_custom_t * state_machine, control_command_t * controls)
{
	bool switch_enabled = ((int32_t)(state_machine->remote->channels[CHANNEL_AUX1] + 1.0f) > 0);

	switch (state_machine->state) 
	{
		case STATE_IDLE:
			if (switch_enabled && (state_machine->enabled==0))
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
			controls->rpy[ROLL] = 0.0f;
			controls->rpy[PITCH] = 0.0f;
			// controls.thrust = stabilisation_copter->thrust_hover_point; // TODO !!!!
			controls->control_mode = ATTITUDE_COMMAND_MODE;
			controls->yaw_mode=YAW_RELATIVE;
		break;
	}

	// Final check if algorithm was aborted
	if (!switch_enabled)
	{
		state_machine_reset(state_machine);
	}

	return TASK_RUN_SUCCESS;
}