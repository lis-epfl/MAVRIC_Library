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
 * \file servos_mix_wing_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Sends messages for the servo_mix_wing module.
 *
 ******************************************************************************/


#include "servos_mix_wing_telemetry.h"
#include "remote.h"



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Update the trims with the current values of the remote.
 * \details UAV has to be un-armed and throttle stick has to be down (to be sure that sticks are released).
 * 
 * \param	mix						The pointer to the servo_mix_wing_t structure
 * \param	packet					The pointer to the MAVLink command long structure
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t servo_mix_telemetry_trim(servo_mix_wing_t* mix, mavlink_command_long_t* packet);



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t servo_mix_telemetry_trim(servo_mix_wing_t* mix, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_DENIED;
	
	if( packet->param1 == 1) // Set roll and pitch trims
	{
		mix->config.trim_roll = mix->config.trim_roll + remote_get_roll(mix->remote);
		mix->config.trim_pitch = mix->config.trim_pitch + remote_get_pitch(mix->remote);
		result = MAV_RESULT_ACCEPTED;
	}
	if(packet->param2 == 1)	// Reset roll and pitch trims
	{
		mix->config.trim_roll = 0.0f;
		mix->config.trim_pitch = 0.0f;
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool servo_mix_wing_telemetry_init(servo_mix_wing_t* mix, mavlink_message_handler_t *mavlink_handler)
{
	bool init_success = true;
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	// MAV_CMD_DO_SET_SERVO (183):  It will not set PWM value, it is used instead to define the servo_mix trim values (this define is used to avoid having to create one more).
	callbackcmd.command_id    = MAV_CMD_DO_SET_SERVO;
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&servo_mix_telemetry_trim;
	callbackcmd.module_struct =										mix;
	init_success &= mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
	
	return init_success;
}


