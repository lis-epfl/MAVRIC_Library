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
 * \file navigation.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_waypoint_handler.h"
#include "stabilisation.h"
#include "quaternions.h"
#include "mavlink_waypoint_handler.h"
#include "position_estimation.h"
#include "tasks.h"
#include "mavlink_communication.h"
#include "state.h"
#include "remote.h"
#include "pid_control.h"
#include <stdbool.h>


/**
 * \brief The navigation structure
 */
typedef struct
{
	float dist2vel_gain;								///< The gain linking the distance to the goal to the actual speed
	pid_controller_t hovering_controller;				///< hovering controller
	pid_controller_t wpt_nav_controller;				///< waypoint navigation controller
	float cruise_speed;									///< The cruise speed in m/s
	float max_climb_rate;								///< Max climb rate in m/s
	float soft_zone_size;								///< Soft zone of the velocity controller
	
	float dt;											///< The time interval between two navigation updates
	uint32_t last_update;								///< The time of the last navigation update in ms
	
	float alt_lpf;										///< The low-pass filtered altitude for auto-landing
	float LPF_gain;										///< The value of the low-pass filter gain
	
	uint8_t loop_count;									///< A counter for sending MAVLink messages at a lower rate than the function
	
	mav_mode_t mode;									///< The mode of the MAV to have a memory of its evolution
	
	bool auto_takeoff;									///< The flag to start and end the auto takeoff procedure
	bool auto_landing;									///< The flag to start and end the auto landing procedure
	
	critical_behavior_enum critical_behavior;			///< The critical behavior enum
	auto_landing_behavior_t auto_landing_behavior;		///< The autolanding behavior enum
	
	bool critical_next_state;							///< Flag to change critical state in its dedicated state machine
	
	bool auto_landing_next_state;						///< Flag to change critical state in its dedicated state machine
	
	control_command_t *controls_nav;					///< The pointer to the navigation control structure
	const control_command_t* control_joystick;			///< The pointer to the joystick control structure
	const quat_t *qe;									///< The pointer to the attitude quaternion structure
	mavlink_waypoint_handler_t *waypoint_handler;		///< The pointer to the waypoint handler structure
	const position_estimator_t *position_estimator;		///< The pointer to the position estimation structure in central_data
	state_t* state;										///< The pointer to the state structure in central_data
	const mavlink_stream_t* mavlink_stream;				///< The pointer to the MAVLink stream structure
	remote_t* remote;									///< The pointer to the remote structure
}navigation_t;

typedef struct 
{
	float dist2vel_gain;								///< The gain linking the distance to the goal to the actual speed
	float cruise_speed;									///< The cruise speed in m/s
	float max_climb_rate;								///< Max climb rate in m/s
	
	float soft_zone_size;								///< Soft zone of the velocity controller
	
	float alt_lpf;										///< The low-pass filtered altitude for auto-landing
	float LPF_gain;										///< The value of the low-pass filter gain
	
	pid_controller_t hovering_controller;				///< hovering controller
	pid_controller_t wpt_nav_controller;				///< waypoint navigation controller
}navigation_config_t;

/**
 * \brief	Initialization
 *
 * \param	navigation				The pointer to the navigation structure
 * \param	nav_config				The pointer to the config structure
 * \param	controls_nav			The pointer to the control structure
 * \param	qe						The pointer to the attitude quaternion structure
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	position_estimator		The pointer to the position estimation structure
 * \param	state					The pointer to the state structure 
 * \param	control_joystick		The pointer to the joystick structure
 * \param	remote					The pointer to the remote structure
 * \param	mavlink_communication	The pointer to the MAVLink communication structure
 */
void navigation_init(navigation_t* navigation, navigation_config_t* nav_config, control_command_t* controls_nav, const quat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator, state_t* state, const control_command_t* control_joystick, remote_t* remote, mavlink_communication_t* mavlink_communication);

/**
 * \brief	Initialise the position hold mode
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	local_pos				The position where the position will be held
 */
void navigation_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t local_pos);

/**
 * \brief	Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	navigation		The pointer to the navigation structure in central_data
 */
task_return_t navigation_update(navigation_t* navigation);

#ifdef __cplusplus
}
#endif

#endif // NAVIGATION_H_