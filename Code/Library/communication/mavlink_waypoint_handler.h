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

#define MAX_WAYPOINTS 10

/*
 * N.B.: Reference Frames and MAV_CMD_NAV are defined in "maveric.h"
 */

/**
 * \brief	
 */
typedef struct
{
	uint8_t frame;				///<
	uint16_t waypoint_id;		///<
	uint8_t current;			///<
	uint8_t autocontinue;		///<
	float param1;				///< hold time in seconds
	float param2;				///< acceptance radius
	float param3;				///< loiter radius
	float param4;				///< desired heading at waypoint
	double x;					///<
	double y;					///<
	double z;					///<
} waypoint_struct;

/**
 * \brief	
 */
typedef enum
{
	CLIMB_TO_SAFE_ALT,			///<
	FLY_TO_HOME_WP,				///<
	CRITICAL_LAND				///<
} critical_behavior_enum;

/**
 * \brief	
 */
typedef enum
{
	DESCENT_TO_SMALL_ALTITUDE,	///<
	DESCENT_TO_GND				///<
} auto_landing_enum_t;


int sending_waypoint_num;
int waypoint_request_number;

uint16_t num_waypoint_onboard;

uint32_t start_timeout;
uint32_t timeout_max_waypoint;


/**
 * \brief						Initialize the waypoint handler
 */
void init_waypoint_handler(void);

/**
 * \brief						Initialize a first waypoint if a flight plan is set
 */
void init_waypoint(void);

/**
 * \brief						Initialize a home waypoint at (0,0,0) at start up
 *
 * \param waypoint_list			The array of all waypoints (here only the home waypoint)
 * \param number_of_waypoints	The pointer to the number of waypoints (here 1)
 */
void init_homing_waypoint(waypoint_struct waypoint_list[],uint16_t* number_of_waypoints);

/**
 * \brief						Initialize a list of hardcoded waypoints
 *
 * \param waypoint_list			The array of all waypoints are stored
 * \param number_of_waypoints	The pointer to the number of waypoints
 */
void init_waypoint_list(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sends the number of onboard waypoint to mavlink when asked by ground station
 *
 * \param rec					The received mavlink message structure
 * \param num_of_waypoint		The number of onboard waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void send_count(Mavlink_Received_t* rec, uint16_t num_of_waypoint, bool* waypoint_receiving, bool * waypoint_sending);

/**
 * \brief						Sends a given waypoint via a mavlink message
 *
 * \param rec					The received mavlink message structure asking for a waypoint
 * \param waypoint				The waypoint list of all onboard waypoints
 * \param num_of_waypoint		The number of onboard waypoints
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void send_waypoint(Mavlink_Received_t* rec, waypoint_struct waypoint[], uint16_t num_of_waypoint, bool* waypoint_sending);

/**
 * \brief						Receives a acknoledge message from mavlink
 *
 * \param rec					The received mavlink message structure
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void receive_ack_msg(Mavlink_Received_t* rec, bool* waypoint_sending);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Receives the number of waypoints that the ground station is sending
 *
 * \param rec					The received mavlink message structure with the total number of waypoint
 * \param number_of_waypoints	The pointer to the number of waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void receive_count(Mavlink_Received_t* rec, uint16_t* number_of_waypoints, bool* waypoint_receiving, bool* waypoint_sending);

/**
 * \brief						Receives a given waypoint and stores it in the local structure
 *
 * \param rec					The received mavlink message structure with the waypoint
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param number_of_waypoints	The number of onboard waypoints	
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 */
void receive_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t number_of_waypoints, bool* waypoint_receiving);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sets the current waypoint to num_of_waypoint
 *
 * \param rec					The received mavlink message structure with the number of the current waypoint
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param num_of_waypoint		The waypoint to be set as current
 */
void set_current_waypoint(Mavlink_Received_t* rec,  waypoint_struct waypoint_list[], uint16_t num_of_waypoint);

/**
 * \brief						Set the current waypoint to new_current
 *
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param num_of_waypoint		The number of onboard waypoints	
 * \param new_current			The waypoint to be set as current
 */
void set_current_waypoint_from_parameter(waypoint_struct waypoint_list[], uint16_t num_of_waypoint, uint16_t new_current);

/**
 * \brief						Clears the waypoint list
 *
 * \param rec					The received mavlink message structure with the clear command
 * \param number_of_waypoints	The pointer to the number of waypoints
 * \param waypoint_set			Flag telling whether a flight plan (true) is set or not (false)
 */
void clear_waypoint_list(Mavlink_Received_t* rec,  uint16_t* number_of_waypoints, bool* waypoint_set);

/**
 * \brief						Set a new home position, origin of the local frame
 *
 * \param rec					The received mavlink message structure with the new home position
 */
void set_home(Mavlink_Received_t* rec);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Set the state and the mode of the vehicle
 *
 * \param rec					The received mavlink message structure
_* \param board_mav_mode		The pointer to the mode of the vehicle
 * \param board_mav_state		The pointer to the state of the vehicle
 * \param sim_mode				The simulation mode
 */
void set_mav_mode(Mavlink_Received_t* rec, uint8_t* board_mav_mode, uint8_t* board_mav_state, uint8_t sim_mode);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Control if time is over timeout and change sending/receiving flags to false
 *
 * \param num_of_waypoint		The pointer to the number of waypoints
 * \param waypoint_receiving	Flag that is true when we are receiving waypoints, false otherwise
 * \param waypoint_sending		Flag that is true when we are sending waypoints, false otherwise
 */
void control_time_out_waypoint_msg(uint16_t* num_of_waypoint, bool* waypoint_receiving, bool* waypoint_sending);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Set the waypoint depending on the reference frame defined in the current_waypoint structure
 *
 * \param current_waypoint		The waypoint from which the reference frame is taken
 * \param origin				The coordinates (latitude, longitude and altitude in global frame) of the local frame's origin
 *
 * \return						The waypoint in local coordinate frame
 */
local_coordinates_t set_waypoint_from_frame(waypoint_struct current_waypoint, global_position_t origin);

/**
 * \brief						Initialise the position hold mode
 *
 * \param localPos				The position where the position will be held
 */
void waypoint_hold_init(local_coordinates_t localPos);

/**
 * \brief						Sets the automatic takeoff waypoint
 */
void waypoint_take_off(void);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Drives the hold position procedure
 */
void waypoint_hold_position_handler(void);

/**
 * \brief						Drives the GPS navigation procedure
 */
void waypoint_navigation_handler(void);

/**
 * \brief						Drives the critical navigation behavior
 */
void waypoint_critical_handler(void);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Drives the auto landing procedure
 */
void auto_landing(void);

/**
 * \brief						Set the next waypoint as current waypoint
 */
void continueToNextWaypoint(void);
//---------------------------------------------------------------------------------------------------------------------------------------//

/**
 * \brief						Sets a circle scenario, where two waypoints are set at opposite side of the circle
 *
 * \param waypoint_list			The waypoint list of all onboard waypoints
 * \param number_of_waypoints	The pointer to the number of onboard waypoints
 * \param circle_radius			The radius of the circle
 * \param num_of_vhc			The number of vehicle, the position is set ID wise
 */
void set_circle_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc);

// TODO: Add code in the function :)
//void set_stream_scenario(waypoint_struct waypoint_list[], uint16_t* number_of_waypoints, float circle_radius, float num_of_vhc);

#ifdef __cplusplus
}
#endif



#endif // MAVLINK_WAYPOINT_HANDLER__