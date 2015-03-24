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
 * \file state.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Holds the current status of the MAV
 *
 ******************************************************************************/


#include "state.h"
#include "print_util.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_init(state_t *state, state_t* state_config, const analog_monitor_t* analog_monitor)
{
	bool init_success = true;
	
	// Init dependencies
	state->analog_monitor = analog_monitor;
	
	// Init parameters
	state->autopilot_type = state_config->autopilot_type;
	state->autopilot_name = state_config->autopilot_name;
	
	state->mav_state = state_config->mav_state;
	state->mav_mode = state_config->mav_mode;
	
	state->source_mode = state_config->source_mode;
	
	state->mav_mode_custom = CUSTOM_BASE_MODE;
	
	state->simulation_mode = state_config->simulation_mode;
	
	init_success &= battery_init(&state->battery,state_config->battery.type,state_config->battery.low_level_limit);
	
	if (state->simulation_mode == HIL_ON)
	{
		// state->mav_mode |= MAV_MODE_FLAG_HIL_ENABLED;
		state->mav_mode.HIL = HIL_ON;
	}
	else
	{
		// state->mav_mode |= !MAV_MODE_FLAG_HIL_ENABLED;
		state->mav_mode.HIL = HIL_OFF;
	}
	
	state->nav_plan_active = false;
	
	state->in_the_air = false;
	
	state->reset_position = false;
	
	state->remote_active = state_config->remote_active;
	
	print_util_dbg_print("[STATE] Initialized.\r\n");
	
	return init_success;
}

void state_switch_to_active_mode(state_t* state, mav_state_t* mav_state)
{
	*mav_state = MAV_STATE_ACTIVE;
	
	// Tell other modules to reset position and re-compute waypoints
	state->reset_position = true;
	state->nav_plan_active = false;
	
	print_util_dbg_print("Switching to active mode.\r\n");
}