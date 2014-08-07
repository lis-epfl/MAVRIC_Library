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
* \file state_from_remote.c
*
* This module take remote channels as input and returns desired state and mode as output
*/

#include "state_from_remote.h"

void state_from_remote_init(state_from_remote_t* state_from_remote, const state_from_remote_conf_t* config, const remote_t* remote)
{
	// Init dependencies
	state_from_remote->remote = remote;

	// Init parameters
	state_from_remote->safety_channel			= config->safety_channel;				 
	state_from_remote->safety_mode				= config->safety_mode;				 
	state_from_remote->mode_switch_channel		= config->mode_switch_channel;		 
	state_from_remote->mode_switch_up			= config->mode_switch_up;				 
	state_from_remote->mode_switch_middle		= config->mode_switch_middle;			 
	state_from_remote->mode_switch_down			= config->mode_switch_down;			 
	state_from_remote->use_custom_switch		= config->use_custom_switch;			 
	state_from_remote->custom_switch_channel 	= config->custom_switch_channel;		 
	state_from_remote->use_test_switch			= config->use_test_switch;			 
	state_from_remote->test_switch_channel		= config->test_switch_channel;

	// Init state to safety state
	state_from_remote->current_desired_mode = state_from_remote->safety_mode;		 
}


void state_from_remote_update(state_from_remote_t* state_from_remote)
{
	const remote_t* remote = state_from_remote->remote;
	mode_flag_armed_t flag_armed;
	mav_mode_t new_desired_mode = state_from_remote->safety_mode;

	if ( remote->channels[state_from_remote->safety_channel] > 0 )
	{
		// Safety switch UP => Safety mode ON
		new_desired_mode = state_from_remote->safety_mode;
	}
	else
	{
		// Normal mode

		// Get base mode
		if ( remote->channels[state_from_remote->mode_switch_channel] >= 0.5f )
		{
			// Mode switch UP
			new_desired_mode = state_from_remote->mode_switch_up;
		}
		else if ( 	remote->channels[state_from_remote->mode_switch_channel] < 0.5f &&
					remote->channels[state_from_remote->mode_switch_channel] > -0.5f )
		{
			// Mode switch MIDDLE
			new_desired_mode = state_from_remote->mode_switch_middle;	
		}
		else if ( remote->channels[state_from_remote->mode_switch_channel] <= -0.5f )
		{
			// Mode switch DOWN
			new_desired_mode = state_from_remote->mode_switch_down;
		}
	

		// Get armed flag
		if( remote_get_throttle(remote) < -0.95f && 
			remote_get_yaw(remote) > 0.9f && 
			remote_get_pitch(remote) > 0.9f && 
			remote_get_roll(remote) > 0.9f )
		{
			// Both sticks in bottom right corners => arm
			flag_armed = ARMED_ON;
		}
		else if ( remote_get_throttle(remote) < -0.95f && 
				remote_get_yaw(remote) < -0.9f && 
				remote_get_pitch(remote) > 0.9f && 
				remote_get_roll(remote) < -0.9f )
		{
			// Both sticks in bottom left corners => disarm 
			flag_armed = ARMED_OFF;
		}
		else
		{
			// Keep current flag
			flag_armed = state_from_remote->current_desired_mode & ( 1 << MODE_FLAG_ARMED );
		}


		// Apply armed flag
		if ( flag_armed == ARMED_ON )
		{
			new_desired_mode |= ( 1 << MODE_FLAG_ARMED );
		}
		else
		{
			new_desired_mode &= ~( 1 << MODE_FLAG_ARMED );
		}


		// Apply custom flag
		if ( state_from_remote->use_custom_switch == true )
		{
			if ( remote->channels[state_from_remote->custom_switch_channel] < 0.0f )
			{
				// Custom switch DOWN => CUSTOM_ON;
				new_desired_mode |= ( 1 << MODE_FLAG_CUSTOM );

			}
			else
			{
				// Custom switch UP => CUSTOM_OFF;
				new_desired_mode &= ~( 1 << MODE_FLAG_CUSTOM );
			}
		}
		else
		{
			// Do nothing: use value indicated in new_desired_mode
		}


		// Apply test flag
		if ( state_from_remote->use_test_switch == true )
		{
			if ( remote->channels[state_from_remote->test_switch_channel] < 0.0f )
			{
				// Test switch DOWN => TEST_ON
				new_desired_mode |= ( 1 << MODE_FLAG_TEST );
			}
			else
			{
				// Test switch DOWN => TEST_OFF;
				new_desired_mode &= ~( 1 << MODE_FLAG_TEST );
			}
		}
		else
		{
			// Do nothing: use value indicated in new_desired_mode
		}		 
	}

	// Store desired mode
	state_from_remote->current_desired_mode = new_desired_mode;
}


mav_mode_t state_from_remote_get(state_from_remote_t* state_from_remote)
{
	state_from_remote_update(state_from_remote);
	return state_from_remote->current_desired_mode;
}