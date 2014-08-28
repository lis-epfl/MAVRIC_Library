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
* \file remote_mode.c
*
* Utility to get mav_mode from remote input
*/


#include "remote_mode.h"


// //------------------------------------------------------------------------------
// // PRIVATE FUNCTIONS DECLARATION
// //------------------------------------------------------------------------------

// static mode_flag_armed_t get_armed_flag(remote_mode_t* remote_mode);


// //------------------------------------------------------------------------------
// // PRIVATE FUNCTIONS IMPLEMENTATION
// //------------------------------------------------------------------------------

// static mode_flag_armed_t get_armed_flag(remote_mode_t* remote_mode)
// {
// 	const remote_t* remote = remote_mode->remote;
// 	mode_flag_armed_t armed = remote_mode->current_desired_mode.ARMED;

// 	// Get armed flag
// 	if( remote_get_throttle(remote) < -0.95f && 
// 		remote_get_yaw(remote) > 0.9f && 
// 		remote_get_pitch(remote) > 0.9f && 
// 		remote_get_roll(remote) > 0.9f )
// 	{
// 		// Both sticks in bottom right corners => arm
// 		armed = ARMED_ON;
// 	}
// 	else if ( remote_get_throttle(remote) < -0.95f && 
// 			remote_get_yaw(remote) < -0.9f && 
// 			remote_get_pitch(remote) > 0.9f && 
// 			remote_get_roll(remote) < -0.9f )
// 	{
// 		// Both sticks in bottom left corners => disarm 
// 		armed = ARMED_OFF;
// 	}
// 	else
// 	{
// 		// Keep current flag
// 	}

// 	return armed;
// }


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

// void remote_mode_init(remote_mode_t* remote_mode, const remote_mode_conf_t* config, const remote_t* remote)
// {
// 	// Init dependencies
// 	remote_mode->remote = remote;

// 	// Init parameters
// 	remote_mode->safety_channel			= config->safety_channel;				 
// 	remote_mode->safety_mode			= config->safety_mode;				 
// 	remote_mode->mode_switch_channel	= config->mode_switch_channel;		 
// 	remote_mode->mode_switch_up			= config->mode_switch_up;				 
// 	remote_mode->mode_switch_middle		= config->mode_switch_middle;			 
// 	remote_mode->mode_switch_down		= config->mode_switch_down;			 
// 	remote_mode->use_custom_switch		= config->use_custom_switch;			 
// 	remote_mode->custom_switch_channel 	= config->custom_switch_channel;		 
// 	remote_mode->use_test_switch		= config->use_test_switch;			 
// 	remote_mode->test_switch_channel	= config->test_switch_channel;
// 	remote_mode->use_override_switch	= config->use_override_switch;			 
// 	remote_mode->override_channel		= config->override_channel;

// 	// Init state to safety state, disarmed
// 	remote_mode->current_desired_mode 	= remote_mode->safety_mode;	
// 	remote_mode->current_desired_mode.ARMED = ARMED_OFF;
// }


// void remote_mode_update(remote_mode_t* remote_mode)
// {
// 	const remote_t* remote = remote_mode->remote;

// 	mav_mode_t new_desired_mode = remote_mode->safety_mode;
// 	mode_flag_armed_t flag_armed;


// 	// Get armed flag from stick combinaison
// 	flag_armed = get_armed_flag(remote_mode);


// 	if ( remote->channels[remote_mode->safety_channel] > 0 )
// 	{
// 		// Safety switch UP => Safety mode ON
// 		new_desired_mode = remote_mode->safety_mode;

// 		// Allow arm and disarm in safety mode
// 		new_desired_mode.ARMED = flag_armed;
// 	}
// 	else
// 	{
// 		// Normal mode

// 		// Get base mode
// 		if ( remote->channels[remote_mode->mode_switch_channel] >= 0.5f )
// 		{
// 			// Mode switch UP
// 			new_desired_mode = remote_mode->mode_switch_up;
// 		}
// 		else if ( 	remote->channels[remote_mode->mode_switch_channel] < 0.5f &&
// 					remote->channels[remote_mode->mode_switch_channel] > -0.5f )
// 		{
// 			// Mode switch MIDDLE
// 			new_desired_mode = remote_mode->mode_switch_middle;	
// 		}
// 		else if ( remote->channels[remote_mode->mode_switch_channel] <= -0.5f )
// 		{
// 			// Mode switch DOWN
// 			new_desired_mode = remote_mode->mode_switch_down;
// 		}


// 		// Apply custom flag
// 		if ( remote_mode->use_custom_switch == true )
// 		{
// 			if ( remote->channels[remote_mode->custom_switch_channel] > 0.0f )
// 			{
// 				// Custom channel at 100% => CUSTOM_ON;
// 				new_desired_mode.CUSTOM = CUSTOM_ON;
// 			}
// 			else
// 			{
// 				// Custom channel at -100% => CUSTOM_OFF;
// 				new_desired_mode.CUSTOM = CUSTOM_OFF;
// 			}
// 		}


// 		// Apply test flag
// 		if ( remote_mode->use_test_switch == true )
// 		{
// 			if ( remote->channels[remote_mode->test_switch_channel] > 0.0f )
// 			{
// 				// Test channel at 100% => TEST_ON
// 				new_desired_mode.TEST = TEST_ON;
// 			}
// 			else
// 			{
// 				// Test channel at -100% => TEST_OFF;
// 				new_desired_mode.TEST = TEST_OFF;
// 			}
// 		}


// 		// Allow only disarm in normal mode
// 		if ( flag_armed == ARMED_OFF )
// 		{
// 			new_desired_mode.ARMED = ARMED_OFF;
// 		}
// 		else
// 		{
// 			// Keep current armed flag
// 			new_desired_mode.ARMED = remote_mode->current_desired_mode.ARMED;
// 		}
// 	}

// 	// Store desired mode
// 	remote_mode->current_desired_mode = new_desired_mode;
// }


// mav_mode_t remote_mode_get(remote_mode_t* remote_mode)
// {
// 	remote_mode_update(remote_mode);
// 	return remote_mode->current_desired_mode;
// }