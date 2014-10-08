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
 * \file remote_telemetry.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This module takes care of sending periodic telemetric messages for
 * the remote controller
 *
 ******************************************************************************/

#include "remote_telemetry.h"
#include "time_keeper.h"
#include "spektrum.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Set slave receiver into bind mode. 
 * \details has to be called 100ms after power-up
 * 
 * \param	satellite				The pointer to the satellite structure
 * \param	packet					The pointer to the MAVLink command long structure
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t remote_telemetry_satellite_bind(spektrum_satellite_t *satellite, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t remote_telemetry_satellite_bind(spektrum_satellite_t *satellite, mavlink_command_long_t* packet) 
{
	mav_result_t result = MAV_RESULT_DENIED;
	
	if (packet->param2 == 1)
	{
		spektrum_satellite_bind();
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	else if (packet->param3 == 1)
	{
		spektrum_satellite_init();
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void remote_telemetry_init(remote_t* remote, mavlink_message_handler_t *mavlink_handler)
{	
	mavlink_message_handler_cmd_callback_t callbackcmd;
		
	callbackcmd.command_id    = MAV_CMD_START_RX_PAIR; // 500
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&remote_telemetry_satellite_bind;
	callbackcmd.module_struct =										remote->sat;
	mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
}

void remote_telemetry_send_raw(const remote_t* remote, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	mavlink_msg_rc_channels_raw_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										0,
										remote->sat->channels[0] + 1024,
										remote->sat->channels[1] + 1024,
										remote->sat->channels[2] + 1024,
										remote->sat->channels[3] + 1024,
										remote->sat->channels[4] + 1024,
										remote->sat->channels[5] + 1024,
										remote->sat->channels[6] + 1024,
										remote->sat->channels[7] + 1024,
										// remote->mode.current_desired_mode.byte);
										remote->signal_quality	);
}

void remote_telemetry_send_scaled(const remote_t* remote, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_rc_channels_scaled_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_millis(),
											0,
											remote->channels[0] * 10000.0f,
											remote->channels[1] * 10000.0f,
											remote->channels[2] * 10000.0f,
											remote->channels[3] * 10000.0f,
											remote->channels[4] * 10000.0f,
											remote->channels[5] * 10000.0f,
											remote->channels[6] * 10000.0f,
											remote->channels[7] * 10000.0f,
											remote->mode.current_desired_mode.byte );
											// remote->signal_quality	);
}