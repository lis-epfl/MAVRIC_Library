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
* \file state_from_remote.h
*
* This module take remote channels as input and returns desired state and mode as output
*/

#ifndef STATE_FROM_REMOTE_H_
#define STATE_FROM_REMOTE_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "state.h"
#include "remote.h"


typedef struct
{
	remote_channel_t 	safety_channel;				///< See state_from_remote_t for documentation
	mav_mode_t			safety_mode;				///< 
	remote_channel_t 	mode_switch_channel;		///< 
	mav_mode_t 			mode_switch_up;				///< 
	mav_mode_t 			mode_switch_middle;			///< 
	mav_mode_t 			mode_switch_down;			///< 
	bool				use_custom_switch;			///< 
	remote_channel_t 	custom_switch_channel;		///< 
	bool				use_test_switch;			///< 
	remote_channel_t 	test_switch_channel;		///< 	
} state_from_remote_conf_t;


typedef struct
{
	remote_channel_t 	safety_channel;				///< Channel to use as 2-way "safety" switch. UP: safety mode, DOWN: normal mode (defined by mode_switch_channel)
	mav_mode_t			safety_mode;				///< Mode when the safety switch is UP (HIL bit flag is ignored)
	remote_channel_t 	mode_switch_channel;		///< Channel to use as 3-way mode switch. The 3 corresponding modes are used when the safety switch is DOWN
	mav_mode_t 			mode_switch_up;				///< Mode when the mode switch is UP (HIL bit flag is ignored)
	mav_mode_t 			mode_switch_middle;			///< Mode when the mode switch is MIDDLE (HIL bit flag is ignored)
	mav_mode_t 			mode_switch_down;			///< Mode when the mode switch is DOWN (HIL bit flag is ignored)
	bool				use_custom_switch;			///< Indicates whether a switch to activate the custom flag should be used
	remote_channel_t 	custom_switch_channel;		///< Channel to use as 2-way custom switch. If not in safety, the custom bit flag is added to the current mode 
	bool				use_test_switch;			///< Indicates whether a switch to activate the test flag should be used
	remote_channel_t 	test_switch_channel;		///< Channel to use as 2-way test switch. If not in safety, the switch overrides the test bit flag: 0 when switch is UP, 1 when switch is DOWN
	mav_mode_t			current_desired_mode;		///< Mav mode indicated by the remote
	const remote_t* 	remote;
} state_from_remote_t;


void state_from_remote_init(state_from_remote_t* state_from_remote, const state_from_remote_conf_t* config, const remote_t* remote);


void state_from_remote_update(state_from_remote_t* state_from_remote);


mav_mode_t state_from_remote_get(state_from_remote_t* state_from_remote);


#ifdef __cplusplus
	}
#endif

#endif