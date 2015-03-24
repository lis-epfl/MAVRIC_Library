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
 * \file state_machine.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Julien Lecoeur
 *   
 * \brief Handles transitions between states and modes
 *
 ******************************************************************************/


#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "scheduler.h"
#include "state.h"
#include "mavlink_waypoint_handler.h"
#include "simulation.h"
#include "joystick_parsing.h"

/**
 * \brief Defines the state machine structure
 */
typedef struct 
{
	uint8_t channel_switches;							///< State of the switches of the remote
	signal_quality_t rc_check;							///< State of the remote (receiving signal or not)
	int8_t motor_state;									///< State of the motors to switch on and off
	
	float		low_battery_update;						///< ///< The time of the last navigation_safety update in ms
	uint32_t	low_battery_counter;					///< ///< The counter of time the battery level was under the critical_battery level

	mavlink_waypoint_handler_t* waypoint_handler;		///< Pointer to the mavlink waypoint handler structure
	state_t* state;										///< Pointer to the state structure
	simulation_model_t *sim_model;						///< Pointer to the simulation structure
	remote_t* remote;									///< Pointer to the remote structure
	joystick_parsing_t* joystick;						///< Pointer to the joystick structure
} state_machine_t;


/**
 * \brief Initialize the state machine
 *
 * \param state_machine				Pointer to the state machine structure
 * \param state						Pointer to the state structure
 * \param sim_model					Pointer to the simulation structure
 * \param remote					Pointer to the remote structure
 * \param joystick					Pointer to the joystick structure
 *
 * \return	True if the init succeed, false otherwise
 */
bool state_machine_init(	state_machine_t *state_machine,
							state_t* state,
							simulation_model_t *sim_model, 
							remote_t* remote,
							joystick_parsing_t* joystick);

/**
 * \brief   Updates the state machine
 *
 * \param	state_machine			Pointer to the state machine structure
 * 
 * \return Returns the result of the task
 */
task_return_t state_machine_update(state_machine_t* state_machine);


#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H_