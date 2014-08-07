/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
* \file remote.c
*
* This file is the driver for the remote control
*/

#include "remote.h"
#include "time_keeper.h"


void remote_init(remote_t* remote, remote_type_t type, const mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	remote->mavlink_stream = mavlink_stream;
	remote->sat = spektrum_satellite_get_pointer();

	// Init parameters according to remote type
	remote->type = type;
	switch ( remote->type )
	{
		case REMOTE_TURNIGY:
			remote->scale = 1.0f / 880.0f;
			remote->deadzone = 30;

			remote->channel_inv[CHANNEL_THROTTLE] = NORMAL;
			remote->channel_inv[CHANNEL_ROLL]     = NORMAL;
			remote->channel_inv[CHANNEL_PITCH]    = INVERTED;
			remote->channel_inv[CHANNEL_YAW]      = NORMAL;
			remote->channel_inv[CHANNEL_GEAR]     = NORMAL;
			remote->channel_inv[CHANNEL_FLAPS]    = NORMAL;
			remote->channel_inv[CHANNEL_AUX1]     = NORMAL;
			remote->channel_inv[CHANNEL_AUX2]     = NORMAL;
			break;

		case REMOTE_SPEKTRUM:
			remote->scale = 1.0f / 700.0f;
			remote->deadzone = 30;

			remote->channel_inv[CHANNEL_THROTTLE] = NORMAL;
			remote->channel_inv[CHANNEL_ROLL]     = INVERTED;
			remote->channel_inv[CHANNEL_PITCH]    = NORMAL;
			remote->channel_inv[CHANNEL_YAW]      = NORMAL;
			remote->channel_inv[CHANNEL_GEAR]     = NORMAL;
			remote->channel_inv[CHANNEL_FLAPS]    = NORMAL;
			remote->channel_inv[CHANNEL_AUX1]     = NORMAL;
			remote->channel_inv[CHANNEL_AUX2]     = NORMAL;
			break;
	}

	// Init 
	remote->signal_quality = SIGNAL_GOOD;
	for ( uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; i++)
	{
		remote->channels[i] = 0.0f;
		remote->trims[i] = 0.0f;
	}
}


task_return_t remote_update(remote_t* remote)
{
	uint32_t now = time_keeper_get_time_ticks() ;
	float raw;
	
	if ( remote->sat->new_data_available == true )
	{
		// Check signal quality
		if ( remote->sat->dt < 100000) 
		{
			// ok
			remote->signal_quality = SIGNAL_GOOD;
		} 
		else if ( remote->sat->dt < 1500000 )
		{
			// warning
			remote->signal_quality = SIGNAL_BAD;
		}

		// Retrieve and scale channels
		for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i)
		{
			raw = remote->sat->channels[i];
			if ( raw < remote->deadzone && raw > -remote->deadzone )
			{
				remote->channels[i] = 0.0f;	
			}
			else
			{
				remote->channels[i] = raw * remote->scale - remote->trims[i];		
			}
		}

		// Indicate that data was handled
		remote->sat->new_data_available = false;
	}
	else
	{
		// Check for signal loss
		if ( ( now - remote->sat->last_update ) > 1500000 )
		{
			// CRITICAL: Set all channels to failsafe
			remote->channels[CHANNEL_THROTTLE] = -1.0f;
			for (uint8_t i = 1; i < REMOTE_CHANNEL_COUNT; i++) 
			{
				remote->channels[i] = 0.0f;
			}

			remote->signal_quality = SIGNAL_LOST;
		}
	}
	
	return TASK_RUN_SUCCESS;
}


signal_quality_t remote_check(remote_t* remote)
{
	return remote->signal_quality;
}


void remote_calibrate(remote_t* remote, remote_channel_t channel)
{
	remote->trims[channel] = remote->channels[channel] - remote->trims[channel];
}


float remote_get_throttle(remote_t* remote)
{
	return remote->channels[CHANNEL_THROTTLE];
}


float remote_get_roll(remote_t* remote)
{
	return remote->channels[CHANNEL_ROLL];
}


float remote_get_pitch(remote_t* remote)
{
	return remote->channels[CHANNEL_PITCH];
}


float remote_get_yaw(remote_t* remote)
{
	return remote->channels[CHANNEL_YAW];
}


task_return_t remote_send_raw(remote_t* remote)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_raw_pack(	remote->mavlink_stream->sysid,
										remote->mavlink_stream->compid,
										&msg,
										time_keeper_get_millis(),
										1,
										remote->sat->channels[0] + 1024,
										remote->sat->channels[1] + 1024,
										remote->sat->channels[2] + 1024,
										remote->sat->channels[3] + 1024,
										remote->sat->channels[4] + 1024,
										remote->sat->channels[5] + 1024,
										remote->sat->channels[6] + 1024,
										remote->sat->channels[7] + 1024,
										remote->signal_quality	);
	
	mavlink_stream_send(remote->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;
}


task_return_t remote_send_scaled(remote_t* remote)
{
	mavlink_message_t msg;
	mavlink_msg_rc_channels_scaled_pack(	remote->mavlink_stream->sysid,
											remote->mavlink_stream->compid,
											&msg,
											time_keeper_get_millis(),
											1,
											remote->channels[0] * 10000.0f,
											remote->channels[1] * 10000.0f,
											remote->channels[2] * 10000.0f,
											remote->channels[3] * 10000.0f,
											remote->channels[4] * 10000.0f,
											remote->channels[5] * 10000.0f,
											remote->channels[6] * 10000.0f,
											remote->channels[7] * 10000.0f,
											remote->signal_quality	);
	
	mavlink_stream_send(remote->mavlink_stream, &msg);
	
	return TASK_RUN_SUCCESS;	
}