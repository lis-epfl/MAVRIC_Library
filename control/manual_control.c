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
 * \file manual_control.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of taking the correct input for the control
 * (i.e. the remote or the joystick)
 *
 ******************************************************************************/
 
 
#include "manual_control.h"
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


bool manual_control_init(manual_control_t* manual_control, const remote_t* remote, const joystick_parsing_t* joystick, const state_t* state)
{
	bool init_success = true;

	manual_control->remote = remote;
	manual_control->joystick = joystick;
	manual_control->state = state;

	print_util_dbg_print("[MANUAL_CONTROL] Initialized\r\n");

	return init_success;
}

void manual_control_get_attitude_command(manual_control_t* manual_control, control_command_t* controls)
{
	if (manual_control->state->remote_active == 1)
	{
		remote_get_command_from_remote(manual_control->remote, controls);
	}
	else
	{
		joystick_parsing_get_attitude_command_from_joystick(manual_control->joystick,controls);
	}
}

void manual_control_get_velocity_command(manual_control_t* manual_control, control_command_t* controls)
{
	if (manual_control->state->remote_active == 1)
	{
		remote_get_velocity_vector_from_remote(manual_control->remote, controls);
	}
	else
	{
		joystick_parsing_get_velocity_vector_from_joystick(manual_control->joystick, controls);
	}
}

float manual_control_get_thrust(const manual_control_t* manual_control)
{
	float thrust;

	if (manual_control->state->remote_active == 1)
	{
		thrust = remote_get_throttle(manual_control->remote);
	}
	else
	{
		thrust = joystick_parsing_get_throttle(manual_control->joystick);
	}
	return thrust;
}
