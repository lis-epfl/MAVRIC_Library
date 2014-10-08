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
 * \file mavlink_waypoint_handler.h
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *  
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


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
 * \brief	The MAVLink waypoint structure
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
	
	bool hold_waypoint_set;										///< Flag to tell if the hold position waypoint is set

	bool waypoint_sending;										///< Flag to tell whether waypoint are being sent
	bool waypoint_receiving;									///< Flag to tell whether waypoint are being received or not
	
	int32_t sending_waypoint_num;								///< The ID number of the sending waypoint
	int32_t waypoint_request_number;							///< The ID number of the requested waypoint

	uint16_t num_waypoint_onboard;								///< The number of waypoint onboard

	uint32_t start_timeout;										///< The start time for the waypoint timeout
	uint32_t timeout_max_waypoint;								///< The max waiting time for communication

	position_estimator_t* position_estimator;					///< The pointer to the position estimation structure
	const ahrs_t* ahrs;											///< The pointer to the attitude estimation structure
	state_t* state;												///< The pointer to the state structure
	mavlink_communication_t* mavlink_communication;				///< The pointer to the MAVLink communication structure
	const mavlink_stream_t* mavlink_stream;						///< Pointer to MAVLink stream

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
 * \param	ahrs					The pointer to the attitude estimation structure
 * \param	state					The pointer to the state structure
 * \param	mavlink_communication	The pointer to the MAVLink communication structure
 */
void waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimator_t* position_estimator, const ahrs_t* ahrs, state_t* state, mavlink_communication_t* mavlink_communication, const mavlink_stream_t* mavlink_stream);

/**
 * \brief	Initialize a first waypoint if a flight plan is set
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 */
void waypoint_handler_nav_plan_init(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Control if time is over timeout and change sending/receiving flags to false
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 *
 * \return	The task status
 */
task_return_t waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief	Set the waypoint depending on the reference frame defined in the current_waypoint structure
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	origin					The coordinates (latitude, longitude and altitude in global frame) of the local frame's origin
 *
 * \return	The waypoint in local coordinate frame
 */
local_coordinates_t waypoint_handler_set_waypoint_from_frame(mavlink_waypoint_handler_t* waypoint_handler, global_position_t origin);

#ifdef __cplusplus
}
#endif

#endif // MAVLINK_WAYPOINT_HANDLER__