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
 * \file joystick_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the joystick controller
 *
 ******************************************************************************/

#include "joystick_telemetry.hpp"

//#include "time_keeper.h"
//#include "print_util.h"
//#include "constants.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/** 
 * \brief				Parse received MAVLink message in structure
 * 
 * \param	joystick	The pointer to the joystick structure
 * \param	sysid		The sysid of the system
 * \param	msg			The pointer to the MAVLink message received
 */
static void joystick_telemetry_parse_msg(joystick_t *joystick, uint32_t sysid, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void joystick_telemetry_parse_msg(joystick_t *joystick, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_manual_control_t packet;
	mavlink_msg_manual_control_decode(msg,&packet);
	
	if ((uint8_t)packet.target == (uint8_t)sysid)
	{	
		joystick->channels.x = packet.x / 1000.0f;
		joystick->channels.y = packet.y / 1000.0f;
		joystick->channels.r = packet.r / 1000.0f;
		joystick->channels.z = packet.z / 1000.0f;
		
		joystick_button_mask(joystick,packet.buttons);
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool joystick_telemetry_init(joystick_t* joystick, mavlink_message_handler_t* message_handler)
{
	bool init_success = true;
	
	// Add callbacks for waypoint handler messages requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_MANUAL_CONTROL; // 69
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&joystick_telemetry_parse_msg;
	callback.module_struct 	= (handling_module_struct_t)		joystick;
	init_success &= mavlink_message_handler_add_msg_callback( message_handler, &callback );
	
	return init_success;
}


void joystick_telemetry_send_manual_ctrl_msg(const joystick_t* joystick, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_manual_control_pack(mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									mavlink_stream->sysid,
									joystick->channels.x * 1000,
									joystick->channels.y * 1000,
									joystick->channels.z * 1000,
									joystick->channels.r * 1000,
									joystick->buttons.button_mask);
}