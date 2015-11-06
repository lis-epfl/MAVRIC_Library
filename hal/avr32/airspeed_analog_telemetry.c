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
 * \file airspeed_analog_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Analog airspeed sensor telemetry functions
 *
 ******************************************************************************/


#include "airspeed_analog_telemetry.h"
#include "time_keeper.h"



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Update the offsets at zero speed
 * 
 * \param	airspeed_analog			The pointer to the airspeed_anaolg structure
 * \param	packet					The pointer to the MAVLink command long structure
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t airspeed_analog_telemetry_offset(airspeed_analog_t* airspeed_analog, mavlink_command_long_t* packet);



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t airspeed_analog_telemetry_offset(airspeed_analog_t* airspeed_analog, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_DENIED;
	
	if( packet->param1 == 1) // Start calib
	{
		airspeed_analog_start_calibration(airspeed_analog);
		result = MAV_RESULT_ACCEPTED;
	}
	if(packet->param2 == 1)	// Stop calib
	{
		airspeed_analog_stop_calibration(airspeed_analog);
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool airspeed_analog_telemetry_init(airspeed_analog_t* airspeed_analog, mavlink_message_handler_t *mavlink_handler)
{
	bool init_success = true;
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	// MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS (242):  Here it is used to set the pitot offset !
	callbackcmd.command_id    = MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS;
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&airspeed_analog_telemetry_offset;
	callbackcmd.module_struct =										airspeed_analog;
	init_success &= mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
	
	return init_success;
}

void airspeed_analog_telemetry_send(airspeed_analog_t* airspeed_analog, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	//TODO: create a mavlink message
	mavlink_msg_debug_vect_pack(	mavlink_stream->sysid,
									mavlink_stream->compid,
									msg,
									"Airspd",
									time_keeper_get_micros(),
									(float)airspeed_analog->raw_airspeed,
									(float)airspeed_analog->scaled_airspeed,
									(float)airspeed_analog->airspeed);
}