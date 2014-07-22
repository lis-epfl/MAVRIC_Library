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
 * \file mavlink_waypoint_handler.h
 * 
 * The mavlink waypoint handler
 */


#ifndef MAVLINK_WAYPOINT_HANDLER__
#define MAVLINK_WAYPOINT_HANDLER__

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"
#include "stdbool.h"
#include "position_estimation.h"
#include "imu.h"
#include "mavlink_message_handler.h"
#include "mavlink_communication.h"
#include "state.h"
#include "qfilter.h"

#define MAX_WAYPOINTS 10		///< The maximal size of the waypoint list

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

/**
 * \brief	The mavlink waypoint structure
 */
typedef struct
{
	uint8_t frame;												///< The reference frame of the waypoint
	uint16_t waypoint_id;										///< The MAV_CMD_NAV id of the waypoint
	uint8_t current;											///< Flag to tell whether the waypoint is the current one or not
	uint8_t autocontinue;										///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
	float param1;												///< Parameter depending on the MAV_CMD_NAV id
	float param2;												///< Parameter depending on the MAV_CMD_NAV id
	float param3;												///< Parameter depending on the MAV_CMD_NAV id
	float param4;												///< Parameter depending on the MAV_CMD_NAV id
	double x;													///< The value on the x axis (depends on the reference frame)
	double y;													///< The value on the y axis (depends on the reference frame)
	double z;													///< The value on the z axis (depends on the reference frame)
} waypoint_struct;

/**
 * \brief	The critical behavior enum
 */
typedef enum
{
	CLIMB_TO_SAFE_ALT,											///< First critical behavior
	FLY_TO_HOME_WP,												///< Second critical behavior, comes after CLIMB_TO_SAFE_ALT
	CRITICAL_LAND												///< Third critical behavior, comes after FLY_TO_HOME_WP
} critical_behavior_enum;

/**
 * \brief	The auto-landing enum
 */
typedef enum
{
	DESCENT_TO_SMALL_ALTITUDE,									///< First auto landing behavior
	DESCENT_TO_GND												///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
} auto_landing_enum_t;

typedef struct
{
	waypoint_struct waypoint_list[MAX_WAYPOINTS];				///< The array of all waypoints (max MAX_WAYPOINTS)
	waypoint_struct current_waypoint;							///< The structure of the current waypoint
	uint16_t number_of_waypoints;								///< The total number of waypoints
	int8_t current_waypoint_count;								///< The number of the current waypoint
	
	local_coordinates_t waypoint_coordinates;					///< The coordinates of the waypoint in GPS navigation mode (MAV_MODE_AUTO_ARMED)
	local_coordinates_t waypoint_hold_coordinates;				///< The coordinates of the waypoint in position hold mode (MAV_MODE_GUIDED_ARMED)
	local_coordinates_t waypoint_critical_coordinates;			///< The coordinates of the waypoint in critical state
	float dist2wp_sqr;											///< The square of the distance to the waypoint
	
	bool waypoint_set;											///< Flag to tell that a flight plan (min 1 waypoint) is active
	bool waypoint_sending;										///< Flag to tell whether waypoint are being sent
	bool waypoint_receiving;									///< Flag to tell whether waypoint are being received or not
	bool critical_landing;										///< Flag to execute critical landing (switching motors off)
	bool critical_next_state;									///< Flag to change critical state in its dedicated state machine
	
	bool collision_avoidance;									///< Flag to tell whether the collision avoidance is active or not
	bool automatic_landing;										///< Flag to initiate the auto landing procedure
	bool in_the_air;											///< Flag to tell whether the vehicle is airborne or not
	
	int32_t sending_waypoint_num;								///< The ID number of the sending waypoint
	int32_t waypoint_request_number;							///< The ID number of the requested waypoint

	uint16_t num_waypoint_onboard;								///< The number of waypoint onboard

	uint32_t start_timeout;										///< The start time for the waypoint timeout
	uint32_t timeout_max_waypoint;								///< The max waiting time for communication
	
	critical_behavior_enum critical_behavior;					///< The critical behavior enum
	auto_landing_enum_t auto_landing_enum;						///< The autolanding enum

	position_estimator_t* position_estimator;					///< The pointer to the position estimation structure
	const HIL_mode* simulation_mode;							///< The pointer to the simulation mode structure
	const ahrs_t* attitude_estimation;							///< The pointer to the attitude estimation structure
	const state_structure_t* state_structure;					///< The pointer to the state structure
	mavlink_communication_t* mavlink_communication;				///< The pointer to the mavlink communication structure

}mavlink_waypoint_handler_t;

/**
 * \brief	Initialize a home waypoint at (0,0,0) at start up
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_init_homing_waypoint(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Initialize a list of hardcoded waypoints
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_init_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Initialize the waypoint handler
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	position_estimator		The pointer to the position estimator structure
 * \param	attitude_estimation		The pointer to the attitude estimation structure
 * \param	state_structure			The pointer to the state structure
 * \param	mavlink_communication	The pointer to the mavlink communication structure
 */
void waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimator_t* position_estimator, const ahrs_t* attitude_estimation, const state_structure_t* state_structure, mavlink_communication_t* mavlink_communication);

/**
 * \brief	Initialize a first waypoint if a flight plan is set
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_waypoint_init(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Control if time is over timeout and change sending/receiving flags to false
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 *
 * \return	The task status
 */
task_return_t waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler);



/**
 * \brief	Initialise the position hold mode
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	localPos				The position where the position will be held
 */
void waypoint_handler_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t localPos);

/**
 * \brief	Sets the automatic takeoff waypoint
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_waypoint_take_off(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Drives the hold position procedure
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_waypoint_hold_position_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Drives the GPS navigation procedure
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_waypoint_navigation_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Drives the critical navigation behavior
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_waypoint_critical_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Sets a circle scenario, where two waypoints are set at opposite side of the circle
 *
 * \param	waypoint_list			The waypoint list of all onboard waypoints
 * \param	packet					The structure of the mavlink command message long
 */
task_return_t waypoint_handler_send_collision_avoidance_status(mavlink_waypoint_handler_t *waypoint_handler);

#ifdef __cplusplus
}
#endif

#endif // MAVLINK_WAYPOINT_HANDLER__