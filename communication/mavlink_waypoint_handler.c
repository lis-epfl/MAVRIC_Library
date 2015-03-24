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
 * \file mavlink_waypoint_handler.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


#include "mavlink_waypoint_handler.h"
#include "print_util.h"
#include "time_keeper.h"
#include "maths.h"
#include "constants.h"

#include <stdio.h>

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Sets a scenario for multiple MAV case
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 *
 * \return	mav_result_t			The result of the scenario setting up
 */
static mav_result_t waypoint_handler_set_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sets a circle scenario, where two waypoints are set at opposite side of the circle
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_circle_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sets a circle scenario, where n waypoints are set at random position on a circle
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_circle_uniform_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sets a stream scenario, where two flows of MAVs go in opposite ways
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_stream_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sets a swarm scenario, where two flocks (grouping) of MAVs go in opposite ways
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_swarm_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sends the number of onboard waypoint to MAVLink when asked by ground station
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The pointer to the received MAVLink message structure asking the send count
 */
static void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Sends a given waypoint via a MAVLink message
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The pointer to the received MAVLink message structure asking for a waypoint
 */
static void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Receives a acknowledge message from MAVLink
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure
 */
static void waypoint_handler_receive_ack_msg(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Receives the number of waypoints that the ground station is sending
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure with the total number of waypoint
 */
static void waypoint_handler_receive_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Receives a given waypoint and stores it in the local structure
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure with the waypoint
 */
static void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Sets the current waypoint to num_of_waypoint
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure with the number of the current waypoint
 */
static void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Set the current waypoint to new_current
 *
 * \param	waypoint_handler		The pointer to the waypoint handler
 * \param	packet					The pointer to the decoded MAVLink message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_set_current_waypoint_from_parameter(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Clears the waypoint list
 *
 * \param	waypoint_handler		The pointer to the waypoint handler structure
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure with the clear command
 */
static void waypoint_handler_clear_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Set a new home position, origin of the local frame
 *
 * \param	waypoint_handler		The pointer to the waypoint handler
 * \param	sysid					The system ID
 * \param	msg						The received MAVLink message structure with the new home position
 */
static void waypoint_handler_set_home(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief	Set the next waypoint as current waypoint
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_continue_to_next_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief	Sends back whether the MAV is currently stopped at a waypoint or not
 *
 * \param	waypoint_handler		The pointer to the structure of the MAVLink waypoint handler
 * \param	packet					The pointer to the structure of the MAVLink command message long
 * 
 * \return	The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_is_arrived(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static mav_result_t waypoint_handler_set_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	mav_result_t result;
	
	if (packet->param1 == 1)
	{
		waypoint_handler_set_circle_scenario(waypoint_handler, packet);
		
		result = MAV_RESULT_ACCEPTED;
	}
	else if (packet->param1 == 2)
	{
		waypoint_handler_set_circle_uniform_scenario(waypoint_handler, packet);
		
		result = MAV_RESULT_ACCEPTED;
	}
	else if (packet->param1 == 3)
	{
		waypoint_handler_set_stream_scenario(waypoint_handler, packet);
		
		result = MAV_RESULT_ACCEPTED;
	}
	else if (packet->param1 == 4)
	{
		waypoint_handler_set_swarm_scenario(waypoint_handler, packet);
		
		result = MAV_RESULT_ACCEPTED;
	} 
	else
	{
		result = MAV_RESULT_UNSUPPORTED;
	}
	
	return result;
}

static void waypoint_handler_set_circle_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	float circle_radius = packet->param2;
	float num_of_vhc = packet->param3;
	float altitude = -packet->param4;
	
	float angle_step = 2.0f * PI / num_of_vhc;
	
	waypoint_struct_t waypoint;
	
	local_coordinates_t waypoint_transfo;
	global_position_t waypoint_global;
	
	waypoint_handler->number_of_waypoints = 2;
	waypoint_handler->current_waypoint_count = -1;
	
	waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;
	
	// Start waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (waypoint_handler->mavlink_stream->sysid-1));
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (waypoint_handler->mavlink_stream->sysid-1));
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Circle departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = (packet->param5 == 1);
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + angle_step * (waypoint_handler->mavlink_stream->sysid-1))); // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// End waypoint
	waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (waypoint_handler->mavlink_stream->sysid-1) + PI);
	waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (waypoint_handler->mavlink_stream->sysid-1) + PI);
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Circle destination(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = maths_rad_to_deg(angle_step * (waypoint_handler->mavlink_stream->sysid-1)); // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	if (packet->param5 == 1)
	{
		waypoint_handler->state->nav_plan_active = true;
		print_util_dbg_print("Auto-continue, nav plan active");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
	}
	else
	{
		waypoint_handler->state->nav_plan_active = false;
		print_util_dbg_print("nav plan inactive");
		if (waypoint_handler->state->in_the_air)
		{
			print_util_dbg_print("Resetting hold waypoint");
			waypoint_handler->hold_waypoint_set = false;
		}
	}
}

static void waypoint_handler_set_circle_uniform_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	int16_t i;
	
	float circle_radius = packet->param2;
	float altitude = -packet->param4;
	
	float x;
	float y;
	
	waypoint_struct_t waypoint;
	
	local_coordinates_t waypoint_transfo;
	global_position_t waypoint_global;
	
	waypoint_handler->number_of_waypoints = 0;
	waypoint_handler->current_waypoint_count = -1;
	
	waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;
	
	for (i = 0; i < 10; ++i)
	{
		waypoint_handler->number_of_waypoints++;
		
		x = 2.0f * PI * rand();
		
		// Start waypoint
		waypoint_transfo.pos[X] = circle_radius * cos(x);
		waypoint_transfo.pos[Y] = circle_radius * sin(x);
		waypoint_transfo.pos[Z] = altitude;
		waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
		print_util_dbg_print("Circle uniform departure(x100): (");
		print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
		print_util_dbg_print(", ");
		print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
		print_util_dbg_print("). For system:");
		print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
		print_util_dbg_print(".\r\n");
		waypoint.x = waypoint_global.latitude;
		waypoint.y = waypoint_global.longitude;
		waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame
	
		waypoint.autocontinue = packet->param5;
		if(i==0)
		{	
			waypoint.current = (packet->param5 == 1);
		}
		else
		{
			waypoint.current = 0;
		}
		waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
		waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
		waypoint.param1 = 2.0f; // Hold time in decimal seconds
		waypoint.param2 = 4.0f; // Acceptance radius in meters
		waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
		waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + atan2(y,x))); // Desired yaw angle at MISSION (rotary wing)
	
		waypoint_handler->waypoint_list[i] = waypoint;
	}
	
	if (packet->param5 == 1)
	{
		waypoint_handler->state->nav_plan_active = true;
		print_util_dbg_print("Auto-continue, nav plan active");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
	}
	else
	{
		waypoint_handler->state->nav_plan_active = false;
		print_util_dbg_print("nav plan inactive");
		if (waypoint_handler->state->in_the_air)
		{
			print_util_dbg_print("Resetting hold waypoint");
			waypoint_handler->hold_waypoint_set = false;
		}
	}
}

static void waypoint_handler_set_stream_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	float dist = packet->param2;
	float num_of_vhc = packet->param3;
	float lateral_dist = 30.0f; //packet->param4;
	float altitude = -packet->param4;
	
	waypoint_struct_t waypoint;
	
	local_coordinates_t waypoint_transfo;
	
	global_position_t waypoint_global;
	
	waypoint_handler->number_of_waypoints = 2;
	waypoint_handler->current_waypoint_count = -1;
	
	waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;
	
	// Start waypoint
	if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc/2.0f))
	{
		waypoint_transfo.pos[X] = lateral_dist;
		waypoint_transfo.pos[Y] = dist/2.0f * (waypoint_handler->mavlink_stream->sysid - 1);
	}
	else
	{
		waypoint_transfo.pos[X] = - lateral_dist;
		waypoint_transfo.pos[Y] = dist/2.0f * (waypoint_handler->mavlink_stream->sysid - 1 - (num_of_vhc/2.0f));
	}
	
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Stream departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = (packet->param5 == 1);
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc/2.0f))
	{
		waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	else
	{
		waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// End waypoint
	if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc/2.0f))
	{
		waypoint_transfo.pos[X] = -lateral_dist;
		waypoint_transfo.pos[Y] = dist/2.0f * (waypoint_handler->mavlink_stream->sysid - 1);
	}
	else
	{
		waypoint_transfo.pos[X] = lateral_dist;
		waypoint_transfo.pos[Y] = dist/2.0f * (waypoint_handler->mavlink_stream->sysid - 1 - (num_of_vhc/2.0f));
	}
	
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Stream departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc/2.0f))
	{
		waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	else
	{
		waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	if (packet->param5 == 1)
	{
		waypoint_handler->state->nav_plan_active = true;
		print_util_dbg_print("Auto-continue, nav plan active");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
	}
	else
	{
		waypoint_handler->state->nav_plan_active = false;
		print_util_dbg_print("nav plan inactive");
		if (waypoint_handler->state->in_the_air)
		{
			print_util_dbg_print("Resetting hold waypoint");
			waypoint_handler->hold_waypoint_set = false;
		}
	}
}

static void waypoint_handler_set_swarm_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	float dist = packet->param2;
	uint8_t num_of_vhc = packet->param3;
	float lateral_dist = 40.0f; //packet->param4;
	float altitude = -packet->param4;
	
	float angle_step = 2.0f * PI / (float)floor((num_of_vhc-1)/2);
	
	waypoint_struct_t waypoint;
	
	local_coordinates_t waypoint_transfo;
	
	global_position_t waypoint_global;
	
	waypoint_handler->number_of_waypoints = 2;
	waypoint_handler->current_waypoint_count = -1;
	
	waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;
	
	// Start waypoint
	if ((float)((waypoint_handler->mavlink_stream->sysid-1)%num_of_vhc) <= (num_of_vhc/2.0f-1.0f))
	{
		if ((float)(waypoint_handler->mavlink_stream->sysid%num_of_vhc) == (num_of_vhc/2.0f)) //higher ID
		{
			waypoint_transfo.pos[X] = lateral_dist;
			waypoint_transfo.pos[Y] = 0.0f;
		} 
		else
		{
			waypoint_transfo.pos[X] = lateral_dist + dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10));
			waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10));
		}
	}
	else
	{
		if ((float)abs(waypoint_handler->mavlink_stream->sysid%num_of_vhc - (num_of_vhc/2.0f)) == (num_of_vhc/2.0f))
		{
			waypoint_transfo.pos[X] = - lateral_dist;
			waypoint_transfo.pos[Y] = 0.0f;
		} 
		else
		{
			waypoint_transfo.pos[X] = - lateral_dist - dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10 - floor(num_of_vhc/2)));
			waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10 - floor(num_of_vhc/2)));
		}
	}
	
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Swarm departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude;
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = (packet->param5 == 1);
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	if ((float)((waypoint_handler->mavlink_stream->sysid-1)%num_of_vhc) <= (num_of_vhc/2.0f-1.0f))
	{
		waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	else
	{
		waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// End waypoint
	if ((float)((waypoint_handler->mavlink_stream->sysid-1)%num_of_vhc) <= (num_of_vhc/2.0f-1.0f))
	{
		if ((float)(waypoint_handler->mavlink_stream->sysid%num_of_vhc) == (num_of_vhc/2.0f)) //higher ID
		{
			waypoint_transfo.pos[X] = - lateral_dist;
			waypoint_transfo.pos[Y] = 0.0f;
		}
		else
		{
			waypoint_transfo.pos[X] = - lateral_dist + dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10));
			waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10));
		}
	}
	else
	{
		if ((float)abs(waypoint_handler->mavlink_stream->sysid%num_of_vhc - (num_of_vhc/2.0f)) == (num_of_vhc/2.0f))
		{
			waypoint_transfo.pos[X] = lateral_dist;
			waypoint_transfo.pos[Y] = 0.0f;
		}
		else
		{
			waypoint_transfo.pos[X] = lateral_dist - dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10 - floor(num_of_vhc/2)));
			waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid-1)%10 - floor(num_of_vhc/2)));
		}
	}
	
	waypoint_transfo.pos[Z] = altitude;
	waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);
	
	print_util_dbg_print("Swarm departure(x100): (");
	print_util_dbg_print_num(waypoint_transfo.pos[X]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Y]*100.0f,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(waypoint_transfo.pos[Z]*100.0f,10);
	print_util_dbg_print("). For system:");
	print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid,10);
	print_util_dbg_print(".\r\n");
	waypoint.x = waypoint_global.latitude;
	waypoint.y = waypoint_global.longitude;
	waypoint.z = -altitude;
	
	waypoint.autocontinue = packet->param5;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.param1 = 10.0f; // Hold time in decimal seconds
	waypoint.param2 = 4.0f; // Acceptance radius in meters
	waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	if ((float)((waypoint_handler->mavlink_stream->sysid-1)%num_of_vhc) <= (num_of_vhc/2.0f-1.0f))
	{
		waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	else
	{
		waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
	}
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	if (packet->param5 == 1)
	{
		waypoint_handler->state->nav_plan_active = true;
		print_util_dbg_print("Auto-continue, nav plan active");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
	}
	else
	{
		waypoint_handler->state->nav_plan_active = false;
		print_util_dbg_print("nav plan inactive");
		if (waypoint_handler->state->in_the_air)
		{
			print_util_dbg_print("Resetting hold waypoint");
			waypoint_handler->hold_waypoint_set = false;
		}
	}
}


static void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_request_list_t packet;
	
	mavlink_msg_mission_request_list_decode(msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		mavlink_message_t _msg;
		mavlink_msg_mission_count_pack(	sysid,
										waypoint_handler->mavlink_stream->compid,
										&_msg,
										msg->sysid,
										msg->compid, 
										waypoint_handler->number_of_waypoints);
		mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
		
		if (waypoint_handler->number_of_waypoints != 0)
		{
			waypoint_handler->waypoint_sending = true;
			waypoint_handler->waypoint_receiving = false;
			waypoint_handler->start_timeout = time_keeper_get_millis();
		}
		
		waypoint_handler->sending_waypoint_num = 0;
		print_util_dbg_print("Will send ");
		print_util_dbg_print_num(waypoint_handler->number_of_waypoints,10);
		print_util_dbg_print(" waypoints\r\n");
	}
}

static void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	if (waypoint_handler->waypoint_sending)
	{
		mavlink_mission_request_t packet;
		
		mavlink_msg_mission_request_decode(msg,&packet);
		
		print_util_dbg_print("Asking for waypoint number ");
		print_util_dbg_print_num(packet.seq,10);
		print_util_dbg_print("\r\n");
		
		// Check if this message is for this system and subsystem
		if (((uint8_t)packet.target_system == (uint8_t)sysid)
		&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
		{
			waypoint_handler->sending_waypoint_num = packet.seq;
			if (waypoint_handler->sending_waypoint_num < waypoint_handler->number_of_waypoints)
			{
				//	Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
				// mavlink_msg_mission_item_send (	mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq,
				//									uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1,
				//									float param2, float param3, float param4, float x, float y, float z)
				mavlink_message_t _msg;
				mavlink_msg_mission_item_pack(	sysid,
												waypoint_handler->mavlink_stream->compid,
												&_msg,
												msg->sysid, 
												msg->compid, 
												packet.seq,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].frame,	
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].command,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].current,	
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].autocontinue,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param1,	
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param2,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param3,	
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].param4,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].x,		
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].y,
												waypoint_handler->waypoint_list[waypoint_handler->sending_waypoint_num].z);
				mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
										
				print_util_dbg_print("Sending waypoint ");
				print_util_dbg_print_num(waypoint_handler->sending_waypoint_num, 10);
				print_util_dbg_print("\r\n");
				
				waypoint_handler->start_timeout = time_keeper_get_millis();
			}
		}
	}
}

static void waypoint_handler_receive_ack_msg(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_mission_ack_t packet;
	
	mavlink_msg_mission_ack_decode(msg, &packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		waypoint_handler->waypoint_sending = false;
		waypoint_handler->sending_waypoint_num = 0;
		print_util_dbg_print("Acknowledgment received, end of waypoint sending.\r\n");
	}
}

static void waypoint_handler_receive_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_count_t packet;
	
	mavlink_msg_mission_count_decode(msg, &packet);
	
	print_util_dbg_print("Count:");
	print_util_dbg_print_num(packet.count,10);
	print_util_dbg_print("\r\n");
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		if (waypoint_handler->waypoint_receiving == false)
		{
			// comment these lines if you want to add new waypoints to the list instead of overwriting them
			waypoint_handler->num_waypoint_onboard = 0;
			waypoint_handler->number_of_waypoints =0;
			//---//
			
			if ((packet.count + waypoint_handler->number_of_waypoints) > MAX_WAYPOINTS)
			{
				packet.count = MAX_WAYPOINTS - waypoint_handler->number_of_waypoints;
			}
			waypoint_handler->number_of_waypoints =  packet.count + waypoint_handler->number_of_waypoints;
			print_util_dbg_print("Receiving ");
			print_util_dbg_print_num(packet.count,10);
			print_util_dbg_print(" new waypoints. ");
			print_util_dbg_print("New total number of waypoints:");
			print_util_dbg_print_num(waypoint_handler->number_of_waypoints,10);
			print_util_dbg_print("\r\n");
			
			waypoint_handler->waypoint_receiving   = true;
			waypoint_handler->waypoint_sending     = false;
			waypoint_handler->waypoint_request_number = 0;
			
			
			waypoint_handler->start_timeout = time_keeper_get_millis();
		}
		
		mavlink_message_t _msg;
		mavlink_msg_mission_request_pack(	sysid,
											waypoint_handler->mavlink_stream->compid, 
											&_msg,
											msg->sysid,
											msg->compid,
											waypoint_handler->waypoint_request_number);
		mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
		
		print_util_dbg_print("Asking for waypoint ");
		print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
		print_util_dbg_print("\r\n");
	}
	
}

static void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication,500000);
	
	mavlink_mission_item_t packet;
	
	mavlink_msg_mission_item_decode(msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		waypoint_handler->start_timeout = time_keeper_get_millis();
		
		waypoint_struct_t new_waypoint;
		
		new_waypoint.command = packet.command;
		
		new_waypoint.x = packet.x; // longitude
		new_waypoint.y = packet.y; // latitude
		new_waypoint.z = packet.z; // altitude
		
		new_waypoint.autocontinue = packet.autocontinue;
		new_waypoint.frame = packet.frame;
		
		new_waypoint.current = packet.current;
		
		new_waypoint.param1 = packet.param1;
		new_waypoint.param2 = packet.param2;
		new_waypoint.param3 = packet.param3;
		new_waypoint.param4 = packet.param4;
		
		print_util_dbg_print("New waypoint received ");
		//print_util_dbg_print("(");
		//print_util_dbg_print_num(new_waypoint.x,10);
		//print_util_dbg_print(", ");
		//print_util_dbg_print_num(new_waypoint.y,10);
		//print_util_dbg_print(", ");
		//print_util_dbg_print_num(new_waypoint.z,10);
		//print_util_dbg_print(") Autocontinue:");
		//print_util_dbg_print_num(new_waypoint.autocontinue,10);
		//print_util_dbg_print(" Frame:");
		//print_util_dbg_print_num(new_waypoint.frame,10);
		//print_util_dbg_print(" Current :");
		//print_util_dbg_print_num(packet.current,10);
		//print_util_dbg_print(" Seq :");
		//print_util_dbg_print_num(packet.seq,10);
		//print_util_dbg_print(" command id :");
		//print_util_dbg_print_num(packet.command,10);
		print_util_dbg_print(" requested num :");
		print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
		print_util_dbg_print(" receiving num :");
		print_util_dbg_print_num(packet.seq,10);
		//print_util_dbg_print(" is it receiving :");
		//print_util_dbg_print_num(waypoint_handler->waypoint_receiving,10); // boolean value
		print_util_dbg_print("\r\n");
		
		//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
		if(packet.current == 2)
		{
			// verify we received the command;
			mavlink_message_t _msg;
			mavlink_msg_mission_ack_pack(	sysid,
											waypoint_handler->mavlink_stream->compid,
											&_msg,
											msg->sysid,
											msg->compid,
											MAV_MISSION_UNSUPPORTED);
			mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
		}
		else if(packet.current == 3)
		{ //current = 3 is a flag to tell us this is a alt change only
			
				// verify we received the command
				mavlink_message_t _msg;
				mavlink_msg_mission_ack_pack(	waypoint_handler->mavlink_stream->sysid,
												waypoint_handler->mavlink_stream->compid,
												&_msg,
												msg->sysid,
												msg->compid, 
												MAV_MISSION_UNSUPPORTED);
				mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
			}
			else
			{
				// Check if receiving waypoints
				if (waypoint_handler->waypoint_receiving)
				{
					// check if this is the requested waypoint
					if (packet.seq == waypoint_handler->waypoint_request_number)
					{
						print_util_dbg_print("Receiving good waypoint, number ");
						print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
						print_util_dbg_print(" of ");
						print_util_dbg_print_num(waypoint_handler->number_of_waypoints - waypoint_handler->num_waypoint_onboard,10);
						print_util_dbg_print("\r\n");
						
						waypoint_handler->waypoint_list[waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number] = new_waypoint;
						waypoint_handler->waypoint_request_number++;
						
						if ((waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number) == waypoint_handler->number_of_waypoints)
						{
							MAV_MISSION_RESULT type = MAV_MISSION_ACCEPTED;
							
							mavlink_message_t _msg;
							mavlink_msg_mission_ack_pack( waypoint_handler->mavlink_stream->sysid,
															waypoint_handler->mavlink_stream->compid,
															&_msg,
															msg->sysid,
															msg->compid,type);
							mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
							
							print_util_dbg_print("flight plan received!\n");
							waypoint_handler->waypoint_receiving = false;
							waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
							
							waypoint_handler->start_wpt_time = time_keeper_get_millis();
							
							waypoint_handler->state->nav_plan_active = false;
							waypoint_handler_nav_plan_init(waypoint_handler);
						}
						else
						{
							mavlink_message_t _msg;
							mavlink_msg_mission_request_pack( 	waypoint_handler->mavlink_stream->sysid,
																waypoint_handler->mavlink_stream->compid,
																&_msg,
																msg->sysid,
																msg->compid,
																waypoint_handler->waypoint_request_number);
							mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
							
							print_util_dbg_print("Asking for waypoint ");
							print_util_dbg_print_num(waypoint_handler->waypoint_request_number,10);
							print_util_dbg_print("\n");
						}
				} //end of if (packet.seq == waypoint_handler->waypoint_request_number)
					else
					{
						MAV_MISSION_RESULT type = MAV_MISSION_INVALID_SEQUENCE;
						
						mavlink_message_t _msg;
						mavlink_msg_mission_ack_pack(	waypoint_handler->mavlink_stream->sysid,
														waypoint_handler->mavlink_stream->compid,
														&_msg,
														msg->sysid,
														msg->compid,
														type	);
						mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
					}
			} //end of if (waypoint_handler->waypoint_receiving)
				else
				{
					MAV_MISSION_RESULT type = MAV_MISSION_ERROR;
					print_util_dbg_print("Not ready to receive waypoints right now!\r\n");
					
					mavlink_message_t _msg;
					mavlink_msg_mission_ack_pack(	waypoint_handler->mavlink_stream->sysid,
													waypoint_handler->mavlink_stream->compid,
													&_msg,
													msg->sysid,
													msg->compid,
													type	);
					mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
			} //end of else of if (waypoint_handler->waypoint_receiving)
		} //end of else (packet.current != 2 && !=3 )
	} //end of if this message is for this system and subsystem
				}

static void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_mission_set_current_t packet;
	
	mavlink_msg_mission_set_current_decode(msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		if (packet.seq < waypoint_handler->number_of_waypoints)
		{
			for (int32_t i = 0; i < waypoint_handler->number_of_waypoints; i++)
			{
				waypoint_handler->waypoint_list[i].current = 0;
			}
			
			waypoint_handler->waypoint_list[packet.seq].current = 1;
			
			mavlink_message_t _msg;
			mavlink_msg_mission_current_pack( 	sysid,
												waypoint_handler->mavlink_stream->compid,
												&_msg,
												packet.seq);
			mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
			
			print_util_dbg_print("Set current waypoint to number");
			print_util_dbg_print_num(packet.seq,10);
			print_util_dbg_print("\r\n");
			
			waypoint_handler->start_wpt_time = time_keeper_get_millis();
			
			waypoint_handler->state->nav_plan_active = false;
			waypoint_handler_nav_plan_init(waypoint_handler);
		}
		else
		{
			mavlink_message_t _msg;
			mavlink_msg_mission_ack_pack(	waypoint_handler->mavlink_stream->sysid,
											waypoint_handler->mavlink_stream->compid,
											&_msg,
											msg->sysid,
											msg->compid,
											MAV_CMD_ACK_ERR_ACCESS_DENIED);
			mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
		}
	} //end of if this message is for this system and subsystem
	}

static mav_result_t waypoint_handler_set_current_waypoint_from_parameter(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	mav_result_t result;
	uint16_t new_current = 0;
	
	print_util_dbg_print("All MAVs: Return to first waypoint.\r\n");
	
	if (new_current < waypoint_handler->number_of_waypoints)
	{
		for (uint8_t i=0;i<waypoint_handler->number_of_waypoints;i++)
		{
			waypoint_handler->waypoint_list[i].current = 0;
		}
		waypoint_handler->waypoint_list[new_current].current = 1;
		
		mavlink_message_t msg;
		mavlink_msg_mission_current_pack( 	waypoint_handler->mavlink_stream->sysid,
											waypoint_handler->mavlink_stream->compid,
											&msg,
											new_current);
		mavlink_stream_send(waypoint_handler->mavlink_stream, &msg);
		
		print_util_dbg_print("Set current waypoint to number");
		print_util_dbg_print_num(new_current,10);
		print_util_dbg_print("\r\n");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
		
		waypoint_handler->state->nav_plan_active = false;
		waypoint_handler_nav_plan_init(waypoint_handler);

		result = MAV_RESULT_ACCEPTED;
	}
	else
	{
		result = MAV_RESULT_DENIED;
	}
	
	return result;
}

static void waypoint_handler_clear_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_mission_clear_all_t packet;
	
	mavlink_msg_mission_clear_all_decode(msg,&packet);
	
	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)sysid)
	&& ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
	{
		if (waypoint_handler->number_of_waypoints > 0)
		{
			waypoint_handler->number_of_waypoints = 0;
			waypoint_handler->num_waypoint_onboard = 0;
			waypoint_handler->state->nav_plan_active = 0;
			waypoint_handler->state->nav_plan_active = false;
			waypoint_handler->hold_waypoint_set = false;
		
			mavlink_message_t _msg;
			mavlink_msg_mission_ack_pack( 	waypoint_handler->mavlink_stream->sysid,
											waypoint_handler->mavlink_stream->compid,
											&_msg,
											msg->sysid,
											msg->compid,
											MAV_CMD_ACK_OK);
			mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
								
			print_util_dbg_print("Cleared Waypoint list.\r\n");
		}
	}
}

static void waypoint_handler_set_home(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
	mavlink_set_gps_global_origin_t packet;
	
	if(waypoint_handler->state->mav_mode.ARMED == ARMED_OFF)
	{
		mavlink_msg_set_gps_global_origin_decode(msg,&packet);
	
		// Check if this message is for this system and subsystem
		// Due to possible bug from QGroundControl, no check of target_component and compid
		if ((uint8_t)packet.target_system == (uint8_t)sysid)
		{
			print_util_dbg_print("Set new home location.\r\n");
			waypoint_handler->position_estimation->local_position.origin.latitude = (double) packet.latitude / 10000000.0f;
			waypoint_handler->position_estimation->local_position.origin.longitude = (double) packet.longitude / 10000000.0f;
			waypoint_handler->position_estimation->local_position.origin.altitude = (float) packet.altitude / 1000.0f;
		
			print_util_dbg_print("New Home location: (");
			print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.latitude*10000000.0f,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.longitude*10000000.0f,10);
			print_util_dbg_print(", ");
			print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.altitude*1000.0f,10);
			print_util_dbg_print(")\r\n");
		
			mavlink_message_t _msg;
			mavlink_msg_gps_global_origin_pack( waypoint_handler->mavlink_stream->sysid,
												waypoint_handler->mavlink_stream->compid,
												&_msg,
												waypoint_handler->position_estimation->local_position.origin.latitude*10000000.0f,
												waypoint_handler->position_estimation->local_position.origin.longitude*10000000.0f,
												waypoint_handler->position_estimation->local_position.origin.altitude*1000.0f);
			mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
		}
	}
}

static mav_result_t waypoint_handler_continue_to_next_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	mav_result_t result;
	bool force_next = false;
	uint32_t time_from_start_wpt = time_keeper_get_millis() - waypoint_handler->start_wpt_time;
	uint32_t time_wpt_limit = 5000;
	
	if( packet->param3 == 1 )
	{
		// QGroundControl sends every message twice, 
		//  therefore we do this test to avoid continuing two times in a row towards next waypoint 
		if (time_from_start_wpt > time_wpt_limit) // 5 seconds
		{
			force_next = true;
		}
	}
	
	if ((waypoint_handler->number_of_waypoints>0)&&((!waypoint_handler->state->nav_plan_active)||force_next))
	{
		print_util_dbg_print("All vehicles: Navigating to next waypoint.\r\n");
		
		waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 0;
		
		print_util_dbg_print("Continuing towards waypoint Nr");
		
		waypoint_handler->start_wpt_time = time_keeper_get_millis();
		
		if (waypoint_handler->current_waypoint_count == (waypoint_handler->number_of_waypoints-1))
		{
			waypoint_handler->current_waypoint_count = 0;
		}
		else
		{
			waypoint_handler->current_waypoint_count++;
		}
		print_util_dbg_print_num(waypoint_handler->current_waypoint_count,10);
		print_util_dbg_print("\r\n");
		waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 1;
		waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
		waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&waypoint_handler->current_waypoint, waypoint_handler->position_estimation->local_position.origin);
		
		mavlink_message_t msg;
		mavlink_msg_mission_current_pack( 	waypoint_handler->mavlink_stream->sysid,
											waypoint_handler->mavlink_stream->compid,
											&msg, 
											waypoint_handler->current_waypoint_count);
		mavlink_stream_send(waypoint_handler->mavlink_stream, &msg);
		
		waypoint_handler->state->nav_plan_active = true;

		result = MAV_RESULT_ACCEPTED;
	}
	else
	{
		result = MAV_RESULT_TEMPORARILY_REJECTED;
		
		print_util_dbg_print("Not ready to switch to next waypoint. Either no waypoint loaded or flying towards one\r\n");
	}
	
	// To avoid a MAV_RESULT_TEMPORARILY_REJECTED for the second message and thus
	//  a bad information to the user on the ground, if two messages are received
	//  in a short time interval, we still show the result as MAV_RESULT_ACCEPTED
	if( time_from_start_wpt < time_wpt_limit )
	{
		result = MAV_RESULT_ACCEPTED;
	}
	
	return result;
}

static mav_result_t waypoint_handler_is_arrived(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
	mav_result_t result;
	
	if( packet->param2 == 32)
	{
		if( waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current == 0 )
		{
			result = MAV_RESULT_ACCEPTED;
		}
		else
		{
			result = MAV_RESULT_TEMPORARILY_REJECTED;
		}
	}
	else
	{
		result = MAV_RESULT_DENIED;
	}
	
	return result;
}
//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* position_estimation, const ahrs_t* ahrs, state_t* state, mavlink_communication_t* mavlink_communication, const mavlink_stream_t* mavlink_stream)
{
	bool init_success = true;
	
	waypoint_handler->start_timeout = time_keeper_get_millis();
	waypoint_handler->timeout_max_waypoint = 10000;
	
	waypoint_handler->position_estimation = position_estimation;
	waypoint_handler->ahrs = ahrs;
	waypoint_handler->state = state;
	
	waypoint_handler->mavlink_communication = mavlink_communication;
	waypoint_handler->mavlink_stream = mavlink_stream;

	// init waypoint navigation
	waypoint_handler->number_of_waypoints = 0;
	waypoint_handler->num_waypoint_onboard = 0;
	
	waypoint_handler->sending_waypoint_num = 0;
	waypoint_handler->waypoint_request_number = 0;
	
	waypoint_handler->hold_waypoint_set = false;
	
	waypoint_handler->waypoint_sending = false;
	waypoint_handler->waypoint_receiving = false;
	
	waypoint_handler->start_wpt_time = time_keeper_get_millis();
	waypoint_handler->travel_time = 0;
	
	// Add callbacks for waypoint handler messages requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_MISSION_ITEM; // 39
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_REQUEST; // 40
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_send_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_SET_CURRENT; // 41
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_set_current_waypoint;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_REQUEST_LIST; // 43
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_send_count;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_COUNT; // 44
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_count;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_CLEAR_ALL; // 45
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_clear_waypoint_list;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_MISSION_ACK; // 47
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_receive_ack_msg;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&waypoint_handler_set_home;
	callback.module_struct 	= (handling_module_struct_t)		waypoint_handler;
	init_success &= mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // 20
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&waypoint_handler_set_current_waypoint_from_parameter;
	callbackcmd.module_struct =									waypoint_handler;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&waypoint_handler_continue_to_next_waypoint;
	callbackcmd.module_struct =									waypoint_handler;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	callbackcmd.command_id = MAV_CMD_CONDITION_LAST; // 159
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&waypoint_handler_set_scenario;
	callbackcmd.module_struct =									waypoint_handler;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	
	callbackcmd.command_id = MAV_CMD_CONDITION_DISTANCE; // 114
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&waypoint_handler_is_arrived;
	callbackcmd.module_struct =									waypoint_handler;
	init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);
	
	
	print_util_dbg_print("[WAYPOINT HANDLER] Initialised.\r\n");
	
	return init_success;
}

void waypoint_handler_init_homing_waypoint(mavlink_waypoint_handler_t* waypoint_handler)
{
	waypoint_struct_t waypoint;
	
	waypoint_handler->number_of_waypoints = 1;
	
	waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
	
	//Set home waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_LOCAL_NED;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 0.0f;
	waypoint.y = 0.0f;
	waypoint.z = -10.0f;
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 2; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 0; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
}

void waypoint_handler_init_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler)
{
	// Visit https://code.google.com/p/ardupilot-mega/wiki/MAVLink to have a description of all messages (or common.h)
	waypoint_struct_t waypoint;
	
	waypoint_handler->number_of_waypoints = 4;
	
	waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 1;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x =  465185223.6174f / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 2; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 0; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[0] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65670560 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 4; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 270; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[1] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 1;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;
	
	waypoint.x = 465186816 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m
	
	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 15; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)
	
	waypoint_handler->waypoint_list[2] = waypoint;
	
	// Set nav waypoint
	waypoint.autocontinue = 0;
	waypoint.current = 0;
	waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
	waypoint.command = MAV_CMD_NAV_WAYPOINT;

	waypoint.x = 465182186 / 1.0e7f; // convert to deg
	waypoint.y = 65659084 / 1.0e7f; // convert to deg
	waypoint.z = 20; //m

	waypoint.param1 = 10; // Hold time in decimal seconds
	waypoint.param2 = 12; // Acceptance radius in meters
	waypoint.param3 = 0; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
	waypoint.param4 = 90; // Desired yaw angle at MISSION (rotary wing)

	waypoint_handler->waypoint_list[3] = waypoint;
	
	print_util_dbg_print("Number of Waypoint onboard:");
	print_util_dbg_print_num(waypoint_handler->num_waypoint_onboard,10);
	print_util_dbg_print("\r\n");
}

void waypoint_handler_nav_plan_init(mavlink_waypoint_handler_t* waypoint_handler)
{
	float rel_pos[3];
	
	if ((waypoint_handler->number_of_waypoints > 0)
	//&& (waypoint_handler->position_estimation->init_gps_position || (*waypoint_handler->simulation_mode==HIL_ON))
	&& (waypoint_handler->position_estimation->init_gps_position || (waypoint_handler->state->mav_mode.HIL == HIL_ON))
	&& (waypoint_handler->waypoint_receiving == false))
	{
		for (uint8_t i = 0; i<waypoint_handler->number_of_waypoints; i++)
		{
			if ((waypoint_handler->waypoint_list[i].current == 1)&&(!waypoint_handler->state->nav_plan_active))
			{
				waypoint_handler->current_waypoint_count = i;
				waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
				waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&waypoint_handler->current_waypoint, waypoint_handler->position_estimation->local_position.origin);
				
				print_util_dbg_print("Waypoint Nr");
				print_util_dbg_print_num(i,10);
				print_util_dbg_print(" set,\r\n");
			
				waypoint_handler->state->nav_plan_active = true;
				
				for (uint8_t j = 0; j < 3; j++)
				{
					rel_pos[j] = waypoint_handler->waypoint_coordinates.pos[j]-waypoint_handler->position_estimation->local_position.pos[j];
				}
				waypoint_handler->dist2wp_sqr = vectors_norm_sqr(rel_pos);
			}
		}
	}
}

task_return_t waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler)
{
	if (waypoint_handler->waypoint_sending || waypoint_handler->waypoint_receiving)
	{
		uint32_t tnow = time_keeper_get_millis();
		
		if ((tnow - waypoint_handler->start_timeout) > waypoint_handler->timeout_max_waypoint)
		{
			waypoint_handler->start_timeout = tnow;
			if (waypoint_handler->waypoint_sending)
			{
				waypoint_handler->waypoint_sending = false;
				print_util_dbg_print("Sending waypoint timeout\r\n");
			}
			if (waypoint_handler->waypoint_receiving)
			{
				waypoint_handler->waypoint_receiving = false;
				
				print_util_dbg_print("Receiving waypoint timeout\r\n");
				waypoint_handler->number_of_waypoints = 0;
				waypoint_handler->num_waypoint_onboard = 0;
			}
		}
	}
	return TASK_RUN_SUCCESS;
}

local_coordinates_t waypoint_handler_set_waypoint_from_frame(waypoint_struct_t* current_waypoint, global_position_t origin)
{
	global_position_t waypoint_global;
	local_coordinates_t waypoint_coor;
	
	for (uint8_t i = 0; i < 3; i++)
	{
		waypoint_coor.pos[i] = 0.0f;
	}
	waypoint_coor.origin = origin;
	waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);
	waypoint_coor.timestamp_ms = time_keeper_get_millis();

	switch(current_waypoint->frame)
	{
		case MAV_FRAME_GLOBAL:
		waypoint_global.latitude = current_waypoint->x;
		waypoint_global.longitude = current_waypoint->y;
		waypoint_global.altitude = current_waypoint->z;
		waypoint_coor = coord_conventions_global_to_local_position(waypoint_global,origin);
		
		waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);
		
		print_util_dbg_print("waypoint_global: lat (x1e7):");
		print_util_dbg_print_num(waypoint_global.latitude*10000000,10);
		print_util_dbg_print(" long (x1e7):");
		print_util_dbg_print_num(waypoint_global.longitude*10000000,10);
		print_util_dbg_print(" alt (x1000):");
		print_util_dbg_print_num(waypoint_global.altitude*1000,10);
		print_util_dbg_print(" waypoint_coor: x (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[X]*100,10);
		print_util_dbg_print(", y (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[Y]*100,10);
		print_util_dbg_print(", z (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[Z]*100,10);
		print_util_dbg_print(" localOrigin lat (x1e7):");
		print_util_dbg_print_num(origin.latitude*10000000,10);
		print_util_dbg_print(" long (x1e7):");
		print_util_dbg_print_num(origin.longitude*10000000,10);
		print_util_dbg_print(" alt (x1000):");
		print_util_dbg_print_num(origin.altitude*1000,10);
		print_util_dbg_print("\r\n");
		break;
		
		case MAV_FRAME_LOCAL_NED:
		waypoint_coor.pos[X] = current_waypoint->x;
		waypoint_coor.pos[Y] = current_waypoint->y;
		waypoint_coor.pos[Z] = current_waypoint->z;
		waypoint_coor.heading= maths_deg_to_rad(current_waypoint->param4);
		waypoint_coor.origin = coord_conventions_local_to_global_position(waypoint_coor);
		break;
		
		case MAV_FRAME_MISSION:
		// Problem here: rec is not defined here
		//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
		case MAV_FRAME_GLOBAL_RELATIVE_ALT:
		waypoint_global.latitude = current_waypoint->x;
		waypoint_global.longitude = current_waypoint->y;
		waypoint_global.altitude = current_waypoint->z;
		
		global_position_t origin_relative_alt = origin;
		origin_relative_alt.altitude = 0.0f;
		waypoint_coor = coord_conventions_global_to_local_position(waypoint_global,origin_relative_alt);
		
		waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);
		
		print_util_dbg_print("LocalOrigin: lat (x1e7):");
		print_util_dbg_print_num(origin_relative_alt.latitude * 10000000,10);
		print_util_dbg_print(" long (x1e7):");
		print_util_dbg_print_num(origin_relative_alt.longitude * 10000000,10);
		print_util_dbg_print(" global alt (x1000):");
		print_util_dbg_print_num(origin.altitude*1000,10);
		print_util_dbg_print(" waypoint_coor: x (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[X]*100,10);
		print_util_dbg_print(", y (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[Y]*100,10);
		print_util_dbg_print(", z (x100):");
		print_util_dbg_print_num(waypoint_coor.pos[Z]*100,10);
		print_util_dbg_print("\r\n");
		
		break;
		case MAV_FRAME_LOCAL_ENU:
		// Problem here: rec is not defined here
		//mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
		break;
		
	}
	
	return waypoint_coor;
}

void mavlink_waypoint_handler_send_nav_time(mavlink_waypoint_handler_t* waypoint_handler,const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{	
	mavlink_msg_named_value_int_pack(	mavlink_stream->sysid,
										mavlink_stream->compid,
										msg,
										time_keeper_get_millis(),
										"travel_time",
										waypoint_handler->travel_time);
}
