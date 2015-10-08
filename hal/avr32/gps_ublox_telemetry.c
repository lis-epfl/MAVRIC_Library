/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file gps_ublox_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the GPS UBlox
 *
 ******************************************************************************/


#include "gps_ublox_telemetry.h"
#include "time_keeper.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Launch the configuration of the GPS
 *
 * \param	gps						The pointer to the gps structure
 * \param	packet					The pointer to the decoded MAVLink message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t gps_ublox_start_configuration(gps_t* gps, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t gps_ublox_start_configuration(gps_t* gps, mavlink_command_long_t* packet)
{
	mav_result_t result = MAV_RESULT_TEMPORARILY_REJECTED;
	
	if (packet->param1 == 1)
	{
		gps->configure_gps = true;
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool gps_ublox_telemetry_init(gps_t* gps, mavlink_message_handler_t* message_handler)
{
	bool init_success = true;
	
	// Add callbacks for fat_fs_mounting commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_PREFLIGHT_UAVCAN; // 243
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&gps_ublox_start_configuration;
	callbackcmd.module_struct =									gps;
	init_success &= mavlink_message_handler_add_cmd_callback(message_handler, &callbackcmd);
	
	return init_success;
}

void gps_ublox_telemetry_send_raw(const gps_t* gps, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	if (gps->status == GPS_OK)
	{
		mavlink_msg_gps_raw_int_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										1000 * gps->time_last_msg,
										gps->status,
										gps->latitude * 10000000.0f,
										gps->longitude * 10000000.0f,
										gps->altitude * 1000.0f,
										gps->hdop * 100.0f,
										gps->speed_accuracy * 100.0f,
										gps->ground_speed * 100.0f,
										gps->course,
										gps->num_sats	);
	}
	else
	{
		mavlink_msg_gps_raw_int_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_micros(),
										gps->status,
										46.5193f * 10000000,
										6.56507f * 10000000,
										400 * 1000,
										0,
										0,
										0,
										0,
										gps->num_sats);
	}
}