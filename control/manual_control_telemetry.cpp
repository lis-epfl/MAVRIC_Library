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
 * \file manual_control_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the manual_control
 *
 ******************************************************************************/


#include "manual_control_telemetry.hpp"
#include "manual_control.hpp"
#include "remote_telemetry.hpp"
#include "joystick_telemetry.hpp"

extern "C" 
{
	#include "print_util.h"
	#include "time_keeper.h"
}
//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Activate/Disactivate the use of the remote controller
 *
 * \param	state				The pointer to the state structure
 * \param	packet				The pointer to the decoded MAVLink message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t manual_control_telemetry_toggle_remote_use(manual_control_t* manual_control, mavlink_command_long_t* packet);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t manual_control_telemetry_toggle_remote_use(manual_control_t* manual_control, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_UNSUPPORTED;
	
	if ( packet->param1 == 1)
	{
		manual_control->control_source = CONTROL_SOURCE_REMOTE;
		manual_control->mode_source = MODE_SOURCE_REMOTE;
		
		print_util_dbg_print("Remote control activated\r\n");
		
		result = MAV_RESULT_ACCEPTED;
	}
	else if (packet->param1 == 0)
	{
		manual_control->control_source = CONTROL_SOURCE_NONE;
		manual_control->mode_source = MODE_SOURCE_GND_STATION;
		
		print_util_dbg_print("Remote control disactivated\r\n");
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool manual_control_telemetry_init(manual_control_t* manual_control, mavlink_message_handler_t *message_handler)
{
	bool init_success = true;
	
	// Init remote and joystick telemetry
	init_success &= remote_telemetry_init(  &manual_control->remote,
											message_handler);
	init_success &= joystick_telemetry_init(	&manual_control->joystick,
												message_handler);
	
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;

	callbackcmd.command_id = MAV_CMD_DO_PARACHUTE; // 208
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_SYSTEM_CONTROL; // 250
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&manual_control_telemetry_toggle_remote_use;
	callbackcmd.module_struct =									manual_control;
	init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);

	return init_success;
}


void manual_control_telemetry_send(const manual_control_t* manual_control, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	switch( manual_control->control_source)
	{
		case CONTROL_SOURCE_NONE:
		break;

		case CONTROL_SOURCE_REMOTE:
			mavlink_msg_rc_channels_scaled_pack(	mavlink_stream->sysid,
													mavlink_stream->compid,
													msg,
													time_keeper_get_ms(),
													0,
													manual_control->remote.channels[0] * 10000.0f,
													manual_control->remote.channels[1] * 10000.0f,
													manual_control->remote.channels[2] * 10000.0f,
													manual_control->remote.channels[3] * 10000.0f,
													manual_control->remote.channels[4] * 10000.0f,
													manual_control->remote.channels[5] * 10000.0f,
													manual_control->remote.channels[6] * 10000.0f,
													manual_control->remote.channels[7] * 10000.0f,
													manual_control->remote.mode.current_desired_mode );
			mavlink_stream_send(mavlink_stream, msg);
			mavlink_msg_rc_channels_scaled_pack(	mavlink_stream->sysid,
													mavlink_stream->compid,
													msg,
													time_keeper_get_ms(),
													1,
													manual_control->remote.channels[8] * 10000.0f,
													manual_control->remote.channels[9] * 10000.0f,
													manual_control->remote.channels[10] * 10000.0f,
													manual_control->remote.channels[11] * 10000.0f,
													manual_control->remote.channels[12] * 10000.0f,
													manual_control->remote.channels[13] * 10000.0f,
													INT16_MAX, // 14 channels max 
													INT16_MAX,
													manual_control->remote.signal_quality );
		break;

		case CONTROL_SOURCE_JOYSTICK:
			mavlink_msg_manual_control_pack(mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											mavlink_stream->sysid,
											manual_control->joystick.channels.x * 1000,
											manual_control->joystick.channels.y * 1000,
											manual_control->joystick.channels.z * 1000,
											manual_control->joystick.channels.r * 1000,
											manual_control->joystick.buttons.button_mask);
		break;
	}
}