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
 * \file position_estimation_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the position estimation
 *
 ******************************************************************************/


#include "position_estimation_telemetry.h"
#include "time_keeper.h"
#include "print_util.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Position estimation update step, performing position estimation then position correction (function to be used)
 *
 * \param	pos_est					The pointer to the position estimation structure
 * \param	packet					The pointer to the decoded MAVLink command long message
 */
static mav_result_t position_estimation_set_new_home_position(position_estimator_t *pos_est, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t position_estimation_set_new_home_position(position_estimator_t *pos_est, mavlink_command_long_t* packet)
{
	mav_result_t result;
	
	if (packet->param1 == 1)
	{
		// Set new home position to actual position
		print_util_dbg_print("Set new home location to actual position.\r\n");
		pos_est->local_position.origin = coord_conventions_local_to_global_position(pos_est->local_position);

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(pos_est->local_position.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(pos_est->local_position.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(pos_est->local_position.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\r\n");
	}
	else
	{
		// Set new home position from msg
		print_util_dbg_print("Set new home location. \r\n");

		pos_est->local_position.origin.latitude = packet->param5;
		pos_est->local_position.origin.longitude = packet->param6;
		pos_est->local_position.origin.altitude = packet->param7;

		print_util_dbg_print("New Home location: (");
		print_util_dbg_print_num(pos_est->local_position.origin.latitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(pos_est->local_position.origin.longitude * 10000000.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(pos_est->local_position.origin.altitude * 1000.0f,10);
		print_util_dbg_print(")\r\n");
	}

	*pos_est->nav_plan_active = false;
	
	result = MAV_RESULT_ACCEPTED;
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void position_estimation_telemetry_init(position_estimator_t* pos_est, mavlink_message_handler_t* mavlink_handler)
{
	mavlink_message_handler_cmd_callback_t callbackcmd;
		
	callbackcmd.command_id    = MAV_CMD_DO_SET_HOME; // 179
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&position_estimation_set_new_home_position;
	callbackcmd.module_struct =										pos_est;
	mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
}


void position_estimation_telemetry_send_position(const position_estimator_t* pos_est, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_local_position_ned_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											pos_est->local_position.pos[0],
											pos_est->local_position.pos[1],
											pos_est->local_position.pos[2],
											pos_est->vel_bf[0],
											pos_est->vel_bf[1],
											pos_est->vel_bf[2]);
}

void position_estimation_telemetry_send_global_position(const position_estimator_t* pos_est, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	// send integrated position (for now there is no GPS error correction...!!!)
	global_position_t gpos = coord_conventions_local_to_global_position(pos_est->local_position);
	
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	mavlink_msg_global_position_int_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											gpos.latitude * 10000000,
											gpos.longitude * 10000000,
											gpos.altitude * 1000.0f,
											-pos_est->local_position.pos[2] * 1000,
											pos_est->vel[0] * 100.0f,
											pos_est->vel[1] * 100.0f,
											pos_est->vel[2] * 100.0f,
											pos_est->local_position.heading);
}