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
 * \file navigation.h
 * 
 * Waypoint navigation controller 
 */


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
#include "orca.h"
#include "tasks.h"
#include <stdbool.h>

/**
 * \brief The navigation structure
 */
typedef struct
{
	float dist2vel_gain;								///< The gain linking the distance to the goal to the actual speed
	float cruise_speed;									///< The cruise speed in m/s
	float max_climb_rate;								///< Max climb rate in m/s
	float softZoneSize;									///< Soft zone of the velocity controller
	
	uint8_t loopCount;									///< A counter for sending mavlink messages at a lower rate than the function
	
	
	control_command_t *controls_nav;					///< The pointer to the navigation control structure
	const UQuat_t *qe;									///< The pointer to the attitude quaternion structure
	mavlink_waypoint_handler_t *waypoint_handler;		///< The pointer to the waypoint handler structure
	const position_estimator_t *position_estimator;		///< The pointer to the position estimation structure in central_data
	orca_t *orca;									///< The pointer to the ORCA structure in central_data
	const state_t* state;			///< The pointer to the state structure in central_data
}navigation_t;

/**
 * \brief						Initialization
 *
 * \param	navigation_data		The pointer to the navigation structure
 * \param	controls_nav		The pointer to the control structure
 * \param	qe					The pointer to the attitude quaternion structure
 * \param	waypoint_handler	The pointer to the waypoint handler structure
 * \param	position_estimator	The pointer to the position estimation structure
 * \param	orca			The pointer to the ORCA structure
 * \param	state		The pointer to the state structure 
 */
void navigation_init(navigation_t* navigation_data, control_command_t* controls_nav, const UQuat_t* qe, mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator, orca_t* orca, const state_t* state);

/**
 * \brief						Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param	navigation_data		The pointer to the navigation structure in central_data
 */
task_return_t navigation_update(navigation_t* navigation_data);

#ifdef __cplusplus
}
#endif

#endif // NAVIGATION_H_