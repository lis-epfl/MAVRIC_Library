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
 * \file state_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the state
 *
 ******************************************************************************/


#include "state_telemetry.h"
#include "state.h"
#include "print_util.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief						Set the state and the mode of the vehicle
 *
 * \param	state				The pointer to the state structure
 * \param	sysid				The system ID
 * \param	msg					The received MAVLink message structure
 */
static void state_telemetry_set_mav_mode(state_t* state, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Set the MAV mode from a command message
 *
 * \param	state				The pointer to the state structure
 * \param	packet				The pointer to the decoded MAVLink message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t state_telemetry_set_mode_from_cmd(state_t* state, mavlink_command_long_t* packet );

/**
 * \brief	Activate/Disactivate the use of the remote controller
 *
 * \param	state				The pointer to the state structure
 * \param	packet				The pointer to the decoded MAVLink message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t state_telemetry_toggle_remote_use(state_t* state, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void state_telemetry_set_mav_mode(state_t* state, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_set_mode_t packet;
	
	mavlink_msg_set_mode_decode(msg,&packet);
	
	// Check if this message is for this system and subsystem
	// No component ID in mavlink_set_mode_t so no control
	if ((uint8_t)packet.target_system == (uint8_t)sysid)
	{
		print_util_dbg_print("base_mode:");
		print_util_dbg_print_num(packet.base_mode,10);
		print_util_dbg_print(", custom mode:");
		print_util_dbg_print_num(packet.custom_mode,10);
		print_util_dbg_print("\r\n");

		mav_mode_t new_mode;
		new_mode.byte = packet.base_mode;
		
		if (new_mode.ARMED == ARMED_ON)
		{
			if (state->mav_mode.ARMED == ARMED_OFF)
			{
				state_switch_to_active_mode(state, &state->mav_state);
			}
		}
		else
		{
			state->mav_state = MAV_STATE_STANDBY;
		}
		
		state->mav_mode.ARMED = new_mode.ARMED;
		state->mav_mode.MANUAL = new_mode.MANUAL;
		//state->mav_mode.HIL = new_mode.HIL;
		state->mav_mode.STABILISE = new_mode.STABILISE;
		state->mav_mode.GUIDED = new_mode.GUIDED;
		state->mav_mode.AUTO = new_mode.AUTO;
		state->mav_mode.TEST = new_mode.TEST;
		state->mav_mode.CUSTOM = new_mode.CUSTOM;
		
		//state->mav_mode_custom = packet.custom_mode;
		
		print_util_dbg_print("New mav mode:");
		print_util_dbg_print_num(state->mav_mode.byte,10);
		print_util_dbg_print("\r");
		
	}
}

static mav_result_t state_telemetry_set_mode_from_cmd(state_t* state, mavlink_command_long_t* packet )
{
	mav_result_t result = MAV_RESULT_ACCEPTED;
	
	mav_mode_t new_mode;
	new_mode.byte = packet->param1;
	
	if (new_mode.ARMED == ARMED_ON)
	{
		if (state->mav_mode.ARMED == ARMED_OFF)
		{
			state_switch_to_active_mode(state, &state->mav_state);
		}
	}
	else
	{
		state->mav_state = MAV_STATE_STANDBY;
	}
	
	state->mav_mode.ARMED = new_mode.ARMED;
	state->mav_mode.MANUAL = new_mode.MANUAL;
	state->mav_mode.STABILISE = new_mode.STABILISE;
	state->mav_mode.GUIDED = new_mode.GUIDED;
	state->mav_mode.AUTO = new_mode.AUTO;
	state->mav_mode.TEST = new_mode.TEST;
	state->mav_mode.CUSTOM = new_mode.CUSTOM;
	
	//state->mav_mode_custom = packet->param2;
	
	return result;
}

static mav_result_t state_telemetry_toggle_remote_use(state_t* state, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_UNSUPPORTED;
	
	if ( packet->param1 == 1)
	{
		state->remote_active = 1;
		state->source_mode = REMOTE;
		
		print_util_dbg_print("Remote control activated\r\n");
		
		result = MAV_RESULT_ACCEPTED;
	}
	else if (packet->param1 == 0)
	{
		state->remote_active = 0;
		state->source_mode = GND_STATION;
		
		print_util_dbg_print("Remote control disactivated\r\n");
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool state_telemetry_init(state_t* state, mavlink_message_handler_t *message_handler)
{
	bool init_success = true;
	
	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;
	
	callback.message_id 	= MAVLINK_MSG_ID_SET_MODE; // 11
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&state_telemetry_set_mav_mode;
	callback.module_struct 	= (handling_module_struct_t)		state;
	init_success &= mavlink_message_handler_add_msg_callback( message_handler, &callback );
		
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_DO_SET_MODE; // 176
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&state_telemetry_set_mode_from_cmd;
	callbackcmd.module_struct =									state;
	init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	callbackcmd.command_id = MAV_CMD_DO_PARACHUTE; // 208
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_SYSTEM_CONTROL; // 250
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&state_telemetry_toggle_remote_use;
	callbackcmd.module_struct =									state;
	init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	return init_success;
}

void state_telemetry_send_heartbeat(const state_t* state, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	mavlink_msg_heartbeat_pack(	mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								state->autopilot_type,
								state->autopilot_name,
								state->mav_mode.byte,
								state->mav_mode_custom,
								state->mav_state);
}

void state_telemetry_send_status(const state_t* state, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	float battery_voltage = state->battery.current_voltage;
	float battery_remaining = state->battery.current_level;
	
	mavlink_msg_sys_status_pack(mavlink_stream->sysid,
								mavlink_stream->compid,
								msg,
								state->sensor_present, 						// sensors present
								state->sensor_enabled, 						// sensors enabled
								state->sensor_health, 						// sensors health
								0,                  						// load
								(int32_t)(1000.0f * battery_voltage), 		// bat voltage (mV)
								0,               							// current (mA)
								battery_remaining,							// battery remaining
								0, 0,  										// comms drop, comms errors
								0, 0, 0, 0);        						// autopilot specific errors
}