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
 * \file remote.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the remote control
 *
 ******************************************************************************/


#include "remote.h"
#include "time_keeper.h"
#include "print_util.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Returns the value of the ARMED flag
 *
 * \param	remote			The pointer to the remote structure
 *
 * \return	The value of the ARMED flag
 */
static mode_flag_armed_t get_armed_flag(remote_t* remote);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mode_flag_armed_t get_armed_flag(remote_t* remote)
{
	const remote_mode_t* remote_mode = &remote->mode;
	mode_flag_armed_t armed = remote_mode->current_desired_mode.ARMED;

	// Get armed flag
	if( remote_get_throttle(remote) < -0.95f && 
		remote_get_yaw(remote) > 0.9f && 
		remote_get_pitch(remote) > 0.9f && 
		remote_get_roll(remote) > 0.9f )
	{
		// Both sticks in bottom right corners => arm
		print_util_dbg_print("Arming!\r\n");
		armed = ARMED_ON;
	}
	else if ( remote_get_throttle(remote) < -0.95f && 
			remote_get_yaw(remote) < -0.9f && 
			remote_get_pitch(remote) > 0.9f && 
			remote_get_roll(remote) < -0.9f )
	{
		// Both sticks in bottom left corners => disarm 
		print_util_dbg_print("Disarming!\r\n");
		armed = ARMED_OFF;
	}
	else
	{
		// Keep current flag
	}

	return armed;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


void remote_init(remote_t* remote, const remote_conf_t* config)
{
	// Init dependencies
	remote->sat = spektrum_satellite_get_pointer();

	// Init mode from remote
	remote_mode_init( &remote->mode, &config->mode_config );

	// Init parameters according to remote type
	remote->type = config->type;
	switch ( remote->type )
	{
		case REMOTE_TURNIGY:
			remote->scale = 1.0f / 880.0f;
			remote->deadzone = 30;

			remote->channel_inv[CHANNEL_THROTTLE] = NORMAL;
			remote->channel_inv[CHANNEL_ROLL]     = NORMAL;
			remote->channel_inv[CHANNEL_PITCH]    = INVERTED;
			remote->channel_inv[CHANNEL_YAW]      = NORMAL;
			remote->channel_inv[CHANNEL_GEAR]     = INVERTED;
			remote->channel_inv[CHANNEL_FLAPS]    = INVERTED;
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


void remote_update(remote_t* remote)
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
				remote->channels[i] = raw * remote->scale * remote->channel_inv[i] - remote->trims[i];		
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
}


signal_quality_t remote_check(remote_t* remote)
{
	remote_update(remote);
	
	return remote->signal_quality;
}


void remote_calibrate(remote_t* remote, remote_channel_t channel)
{
	remote->trims[channel] = remote->channels[channel] - remote->trims[channel];
}


float remote_get_throttle(const remote_t* remote)
{	
	return remote->channels[CHANNEL_THROTTLE];
}


float remote_get_roll(const remote_t* remote)
{
	return remote->channels[CHANNEL_ROLL];
}


float remote_get_pitch(const remote_t* remote)
{
	return remote->channels[CHANNEL_PITCH];
}


float remote_get_yaw(const remote_t* remote)
{
	return remote->channels[CHANNEL_YAW];
}


void remote_mode_init(remote_mode_t* remote_mode, const remote_mode_conf_t* config)
{
	// Init parameters
	remote_mode->safety_channel			= config->safety_channel;				 
	remote_mode->safety_mode			= config->safety_mode;				 
	remote_mode->mode_switch_channel	= config->mode_switch_channel;		 
	remote_mode->mode_switch_up			= config->mode_switch_up;				 
	remote_mode->mode_switch_middle		= config->mode_switch_middle;			 
	remote_mode->mode_switch_down		= config->mode_switch_down;			 
	remote_mode->use_custom_switch		= config->use_custom_switch;			 
	remote_mode->custom_switch_channel 	= config->custom_switch_channel;		 
	remote_mode->use_test_switch		= config->use_test_switch;			 
	remote_mode->test_switch_channel	= config->test_switch_channel;
	remote_mode->use_disable_remote_mode_switch	= config->use_disable_remote_mode_switch;			 
	remote_mode->disable_remote_mode_channel		= config->disable_remote_mode_channel;

	// Init state to safety state, disarmed
	remote_mode->current_desired_mode 		= remote_mode->safety_mode;	
	remote_mode->current_desired_mode.ARMED = ARMED_OFF;
}


void remote_mode_update(remote_t* remote)
{
	remote_mode_t* remote_mode = &remote->mode;
	bool do_update = false;

	remote_update(remote);

	// Check whether modes should be updated
	if ( remote_mode->use_disable_remote_mode_switch == true )
	{
		if ( remote->channels[remote_mode->disable_remote_mode_channel] >= 0.5f )
		{
			do_update = true;
		}
		else
		{
			do_update = false;
		}
	}
	else
	{
		do_update = true;
	}

	// Do update if required
	if( do_update == true )
	{
		// Fallback to safety
		mav_mode_t new_desired_mode = remote_mode->safety_mode;

		// Get armed flag from stick combinaison
		mode_flag_armed_t flag_armed = get_armed_flag(remote);

		if ( remote->channels[remote_mode->safety_channel] > 0 )
		{
			// Safety switch UP => Safety mode ON
			new_desired_mode = remote_mode->safety_mode;

			// Allow arm and disarm in safety mode
			new_desired_mode.ARMED = flag_armed;
		}
		else
		{
			// Normal mode

			// Get base mode
			if ( remote->channels[remote_mode->mode_switch_channel] >= 0.5f )
			{
				// Mode switch UP
				new_desired_mode = remote_mode->mode_switch_up;
			}
			else if ( 	remote->channels[remote_mode->mode_switch_channel] < 0.5f &&
						remote->channels[remote_mode->mode_switch_channel] > -0.5f )
			{
				// Mode switch MIDDLE
				new_desired_mode = remote_mode->mode_switch_middle;	
			}
			else if ( remote->channels[remote_mode->mode_switch_channel] <= -0.5f )
			{
				// Mode switch DOWN
				new_desired_mode = remote_mode->mode_switch_down;
			}


			// Apply custom flag
			if ( remote_mode->use_custom_switch == true )
			{
				if ( remote->channels[remote_mode->custom_switch_channel] > 0.0f )
				{
					// Custom channel at 100% => CUSTOM_ON;
					new_desired_mode.CUSTOM = CUSTOM_ON;
				}
				else
				{
					// Custom channel at -100% => CUSTOM_OFF;
					new_desired_mode.CUSTOM = CUSTOM_OFF;
				}
			}


			// Apply test flag
			if ( remote_mode->use_test_switch == true )
			{
				if ( remote->channels[remote_mode->test_switch_channel] > 0.0f )
				{
					// Test channel at 100% => TEST_ON
					new_desired_mode.TEST = TEST_ON;
				}
				else
				{
					// Test channel at -100% => TEST_OFF;
					new_desired_mode.TEST = TEST_OFF;
				}
			}


			// Allow only disarm in normal mode
			if ( flag_armed == ARMED_OFF )
			{
				new_desired_mode.ARMED = ARMED_OFF;
			}
			else
			{
				// Keep current armed flag
				new_desired_mode.ARMED = remote_mode->current_desired_mode.ARMED;
			}
		}

		// Store desired mode
		remote_mode->current_desired_mode = new_desired_mode;
	}
}


mav_mode_t remote_mode_get(const remote_t* remote)
{
	return remote->mode.current_desired_mode;
}

void remote_get_command_from_remote(remote_t* remote, control_command_t* controls)
{
	remote_update(remote);
	
	controls->rpy[ROLL]= remote_get_roll(remote) * RC_INPUT_SCALE;
	controls->rpy[PITCH]= remote_get_pitch(remote) * RC_INPUT_SCALE;
	controls->rpy[YAW]= remote_get_yaw(remote) * RC_INPUT_SCALE;
	controls->thrust = remote_get_throttle(remote);
}

void remote_get_velocity_vector_from_remote(remote_t* remote, control_command_t* controls)
{
	remote_update(remote);
	
	controls->tvel[X]= - 10.0f * remote_get_pitch(remote) * RC_INPUT_SCALE;
	controls->tvel[Y]= 10.0f * remote_get_roll(remote) * RC_INPUT_SCALE;
	controls->tvel[Z]= - 1.5f * remote_get_throttle(remote);
	controls->rpy[YAW] = remote_get_yaw(remote) * RC_INPUT_SCALE;
}