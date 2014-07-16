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
#include "coord_conventions.h"
#include "position_estimation.h"
#include "imu.h"
#include "mavlink_message_handler.h"
#include "mavlink_communication.h"
#include "state.h"
#include "qfilter.h"

#define MAX_WAYPOINTS 10

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

/**
 * \brief	The mavlink waypoint structure
 */
typedef struct
{
	uint8_t frame;				///< The reference frame of the waypoint
	uint16_t waypoint_id;		///< The MAV_CMD_NAV id of the waypoint
	uint8_t current;			///< Flag to tell whether the waypoint is the current one or not
	uint8_t autocontinue;		///< Flag to tell whether the vehicle should auto continue to the next waypoint once it reaches the current waypoint
	float param1;				///< Parameter depending on the MAV_CMD_NAV id
	float param2;				///< Parameter depending on the MAV_CMD_NAV id
	float param3;				///< Parameter depending on the MAV_CMD_NAV id
	float param4;				///< Parameter depending on the MAV_CMD_NAV id
	double x;					///< The value on the x axis (depends on the reference frame)
	double y;					///< The value on the y axis (depends on the reference frame)
	double z;					///< The value on the z axis (depends on the reference frame)
} waypoint_struct;

/**
 * \brief	
 */
typedef enum
{
	CLIMB_TO_SAFE_ALT,			///< First critical behavior
	FLY_TO_HOME_WP,				///< Second critical behavior, comes after CLIMB_TO_SAFE_ALT
	CRITICAL_LAND				///< Third critical behavior, comes after FLY_TO_HOME_WP
} critical_behavior_enum;

/**
 * \brief	
 */
typedef enum
{
	DESCENT_TO_SMALL_ALTITUDE,	///< First auto landing behavior
	DESCENT_TO_GND				///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
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
	bool automatic_take_off;									///< Flag to initiate the auto takeoff procedure
	bool automatic_landing;										///< Flag to initiate the auto landing procedure
	bool in_the_air;											///< Flag to tell whether the vehicle is airborne or not
	
	int32_t sending_waypoint_num;
	int32_t waypoint_request_number;

	uint16_t num_waypoint_onboard;

	uint32_t start_timeout;
	uint32_t timeout_max_waypoint;
	
	critical_behavior_enum critical_behavior;					///< The critical behavior enum
	auto_landing_enum_t auto_landing_enum;						///< The autolanding enum

	position_estimator_t* position_estimator;
	HIL_mode* simulation_mode;
	AHRS_t* attitude_estimation;
	state_structure_t* state_structure;
	mavlink_communication_t* mavlink_communication;

}mavlink_waypoint_handler_t;


/**
 * \brief						Initialize the waypoint handler
 */
void waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimator_t* position_estimator, AHRS_t* attitude_estimation, state_structure_t* state_structure, mavlink_communication_t* mavlink_communication);

/**
 * \brief						Initialize a first waypoint if a flight plan is set
 */
void waypoint_handler_waypoint_init(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief						Initialize a home waypoint at (0,0,0) at start up
 *
 * \param waypoint_list			The array of all waypoints (here only the home waypoint)
 * \param number_of_waypoints	The pointer to the number of waypoints (here 1)
 */
void waypoint_handler_init_homing_waypoint(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief						Initialize a list of hardcoded waypoints
 *
 * \param waypoint_list			The array of all waypoints are stored
 * \param number_of_waypoints	The pointer to the number of waypoints
 */
void waypoint_handler_init_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sends the number of onboard waypoint to mavlink when asked by ground station
 *
 * \param rec					The received mavlink message structure
 * \param num_of_waypoint		The number of onboard waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);

/**
 * \brief						Sends a given waypoint via a mavlink message
 *
 * \param rec					The received mavlink message structure asking for a waypoint
 * \param waypoint				The waypoint list of all onboard waypoints
 * \param num_of_waypoint		The number of onboard waypoints
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);

/**
 * \brief						Receives a acknoledge message from mavlink
 *
 * \param rec					The received mavlink message structure
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void waypoint_handler_receive_ack_msg(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Receives the number of waypoints that the ground station is sending
 *
 * \param rec					The received mavlink message structure with the total number of waypoint
 * \param number_of_waypoints	The pointer to the number of waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void waypoint_handler_receive_count(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);

/**
 * \brief						Receives a given waypoint and stores it in the local structure
 *
 * \param rec					The received mavlink message structure with the waypoint
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param number_of_waypoints	The number of onboard waypoints	
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 */
void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sets the current waypoint to num_of_waypoint
 *
 * \param rec					The received mavlink message structure with the number of the current waypoint
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param num_of_waypoint		The waypoint to be set as current
 */
void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);

/**
 * \brief						Set the current waypoint to new_current
 *
 * \param waypoint_handler		The pointer to the waypoint handler
 * \param packet				The pointer to the decoded mavlink message long
 */
void waypoint_handler_set_current_waypoint_from_parameter(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief						Clears the waypoint list
 *
 * \param rec					The received mavlink message structure with the clear command
 * \param number_of_waypoints	The pointer to the number of waypoints
 * \param waypoint_set			Flag telling whether a flight plan (true) is set or not (false)
 */
void waypoint_handler_clear_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);

/**
 * \brief						Set a new home position, origin of the local frame
 *
 * \param rec					The received mavlink message structure with the new home position
 */
void waypoint_handler_set_home(mavlink_waypoint_handler_t* waypoint_handler, mavlink_received_t* rec);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Control if time is over timeout and change sending/receiving flags to false
 *
 * \param num_of_waypoint		The pointer to the number of waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Set the waypoint depending on the reference frame defined in the current_waypoint structure
 *
 * \param current_waypoint		The waypoint from which the reference frame is taken
 * \param origin				The coordinates (latitude, longitude and altitude in global frame) of the local frame's origin
 *
 * \return						The waypoint in local coordinate frame
 */
local_coordinates_t waypoint_handler_set_waypoint_from_frame(mavlink_waypoint_handler_t* waypoint_handler, global_position_t origin);

/**
 * \brief						Initialise the position hold mode
 *
 * \param localPos				The position where the position will be held
 */
void waypoint_handler_waypoint_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_coordinates_t localPos);

/**
 * \brief						Sets the automatic takeoff waypoint
 */
void waypoint_handler_waypoint_take_off(mavlink_waypoint_handler_t* waypoint_handler);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Drives the hold position procedure
 */
void waypoint_handler_waypoint_hold_position_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief						Drives the GPS navigation procedure
 */
void waypoint_handler_waypoint_navigation_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief						Drives the critical navigation behavior
 */
void waypoint_handler_waypoint_critical_handler(mavlink_waypoint_handler_t* waypoint_handler);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Drives the auto landing procedure
 */
void waypoint_handler_auto_landing(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief						Set the next waypoint as current waypoint
 */
void waypoint_handler_continueToNextWaypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sets a circle scenario, where two waypoints are set at opposite side of the circle
 *
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param number_of_waypoints	The pointer to the number of onboard waypoints
 * \param circle_radius			The radius of the circle
 * \param num_of_vhc			The number of vehicle, the position is set ID wise
 */
void waypoint_handler_set_circle_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

void mavlink_waypoint_handler_set_auto_takeoff(mavlink_waypoint_handler_t *waypoint_handler, mavlink_command_long_t* packet);

// TODO: Add code in the function :)
//void set_stream_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc);

#ifdef __cplusplus
}
#endif



#endif // MAVLINK_WAYPOINT_HANDLER__