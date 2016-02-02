/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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

#include "communication/remote_telemetry.hpp"
#include "drivers/satellite.hpp"

extern "C"
{
	#include "hal/common/time_keeper.hpp"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Set slave receiver into bind mode. 
 * \details has to be called 100ms after power-up
 * 
 * \param	remote					The pointer to the remote_t structure
 * \param	packet					The pointer to the MAVLink command long structure
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t remote_telemetry_satellite_bind(remote_t* remote, mavlink_command_long_t* packet);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t remote_telemetry_satellite_bind(remote_t* remote, mavlink_command_long_t* packet) 
{
	mav_result_t result = MAV_RESULT_DENIED;
	
	if( packet->param2 == 1 )	// Binding
	{
		if( packet->param4 == 10 )
		{
			remote->sat->bind( RADIO_PROTOCOL_DSM2_10BITS );
			result = MAV_RESULT_ACCEPTED;
		}
		else if( packet->param4 == 11 )
		{
			remote->sat->bind( RADIO_PROTOCOL_DSM2_11BITS );
			result = MAV_RESULT_ACCEPTED;
		}
		else
		{
			result = MAV_RESULT_DENIED;
		}
	}
	else if (packet->param3 == 1)	// Init
	{
		remote->sat->init();
		
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool remote_telemetry_init(remote_t* remote, mavlink_message_handler_t *mavlink_handler)
{	
	bool init_success = true;
	
	mavlink_message_handler_cmd_callback_t callbackcmd;
		
	callbackcmd.command_id    = MAV_CMD_START_RX_PAIR; // 500
	callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function      = (mavlink_cmd_callback_function_t)	&remote_telemetry_satellite_bind;
	callbackcmd.module_struct =										remote;
	init_success &= mavlink_message_handler_add_cmd_callback(mavlink_handler, &callbackcmd);
	
	return init_success;
}


void remote_telemetry_send_raw(const remote_t* remote, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	mavlink_msg_rc_channels_raw_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_ms(),
										0,
										remote->sat->channel(0) + 1024,
										remote->sat->channel(1) + 1024,
										remote->sat->channel(2) + 1024,
										remote->sat->channel(3) + 1024,
										remote->sat->channel(4) + 1024,
										remote->sat->channel(5) + 1024,
										remote->sat->channel(6) + 1024,
										remote->sat->channel(7) + 1024,
										// remote->mode.current_desired_mode);
										remote->signal_quality	);
	
	mavlink_stream_send(mavlink_stream, msg);
	
	mavlink_msg_rc_channels_raw_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_ms(),
										1,
										remote->sat->channel(8)+ 1024,
										remote->sat->channel(9)+ 1024,
										remote->sat->channel(10) + 1024,
										remote->sat->channel(11) + 1024,
										remote->sat->channel(12) + 1024,
										remote->sat->channel(13) + 1024,
										UINT16_MAX,
										UINT16_MAX,
										// remote->mode.current_desired_mode);
										remote->signal_quality	);
}


void remote_telemetry_send_scaled(const remote_t* remote, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
	mavlink_msg_rc_channels_scaled_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_ms(),
											0,
											remote->channels[0] * 10000.0f,
											remote->channels[1] * 10000.0f,
											remote->channels[2] * 10000.0f,
											remote->channels[3] * 10000.0f,
											remote->channels[4] * 10000.0f,
											remote->channels[5] * 10000.0f,
											remote->channels[6] * 10000.0f,
											remote->channels[7] * 10000.0f,
											remote->mode.current_desired_mode );
											// remote->signal_quality	);
	
	mavlink_stream_send(mavlink_stream, msg);
	
	mavlink_msg_rc_channels_scaled_pack(	mavlink_stream->sysid,
											mavlink_stream->compid,
											msg,
											time_keeper_get_ms(),
											1,
											remote->channels[8] * 10000.0f,
											remote->channels[9] * 10000.0f,
											remote->channels[10] * 10000.0f,
											remote->channels[11] * 10000.0f,
											remote->channels[12] * 10000.0f,
											remote->channels[13] * 10000.0f,
											INT16_MAX, // 14 channels max 
											INT16_MAX,
											remote->mode.current_desired_mode );
}