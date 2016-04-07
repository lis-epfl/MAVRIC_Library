/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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


#include "communication/mavlink_waypoint_handler.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Sets a scenario for multiple MAV case
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 *
 * \return  mav_result_t            The result of the scenario setting up
 */
static mav_result_t waypoint_handler_set_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sets a circle scenario, where two waypoints are set at opposite side of the circle
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_circle_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sets a circle scenario, where n waypoints are set at random position on a circle
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_circle_uniform_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sets a stream scenario, where two flows of MAVs go in opposite ways
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_stream_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sets a swarm scenario, where two flocks (grouping) of MAVs go in opposite ways
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 */
static void waypoint_handler_set_swarm_scenario(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sends the number of onboard waypoint to MAVLink when asked by ground station
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The pointer to the received MAVLink message structure asking the send count
 */
static void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Sends a given waypoint via a MAVLink message
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The pointer to the received MAVLink message structure asking for a waypoint
 */
static void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Receives a acknowledge message from MAVLink
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure
 */
static void waypoint_handler_receive_ack_msg(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Receives the number of waypoints that the ground station is sending
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure with the total number of waypoint
 */
static void waypoint_handler_receive_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Receives a given waypoint and stores it in the local structure
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure with the waypoint
 */
static void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Sets the current waypoint to num_of_waypoint
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure with the number of the current waypoint
 */
static void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Set the current waypoint to new_current
 *
 * \param   waypoint_handler        The pointer to the waypoint handler
 * \param   packet                  The pointer to the decoded MAVLink message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_set_current_waypoint_from_parameter(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Clears the waypoint list
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure with the clear command
 */
static void waypoint_handler_clear_waypoint_list(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Set a new home position, origin of the local frame
 *
 * \param   waypoint_handler        The pointer to the waypoint handler
 * \param   sysid                   The system ID
 * \param   msg                     The received MAVLink message structure with the new home position
 */
static void waypoint_handler_set_home(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg);

/**
 * \brief   Set the next waypoint as current waypoint
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_continue_to_next_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sends back whether the MAV is currently stopped at a waypoint or not
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_is_arrived(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Drives the automatic takeoff procedure
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static bool waypoint_handler_take_off_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Start/Stop the navigation
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_start_stop_navigation(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Sets auto-takeoff procedure from a MAVLink command message MAV_CMD_NAV_TAKEOFF
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet              The pointer to the structure of the MAVLink command message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_set_auto_takeoff(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Drives the auto landing procedure from the MAV_CMD_NAV_LAND message long
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 * \param   packet                  The pointer to the structure of the MAVLink command message long
 *
 * \return  The MAV_RESULT of the command
 */
static mav_result_t waypoint_handler_set_auto_landing(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet);

/**
 * \brief   Drives the auto-landing navigation behavior
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static void waypoint_handler_auto_landing_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Drives the stopping behavior
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static void waypoint_handler_stopping_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   State machine to drive the navigation module
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static void waypoint_handler_state_machine(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Drives the critical navigation behavior
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static void waypoint_handler_critical_handler(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Drives the GPS navigation procedure
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 */
static void waypoint_handler_waypoint_navigation_handler(mavlink_waypoint_handler_t* waypoint_handler, bool reset_hold_wpt);

/**
 * \brief   Check if the nav mode is equal to the state mav mode
 *
 * \param   waypoint_handler        The pointer to the structure of the MAVLink waypoint handler
 *
 * \return  True if the flag STABILISE, GUIDED and ARMED are equal, false otherwise
 */
static bool waypoint_handler_mode_change(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Control if time is over timeout and change sending/receiving flags to false
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 *
 * \return  The task status
 */
static void waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler);

/**
 * \brief   Set the waypoint depending on the reference frame defined in the current_waypoint structure
 *
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   origin                  The coordinates (latitude, longitude and altitude in global frame) of the local frame's origin
 *
 * \return  The waypoint in local coordinate frame
 */
static local_position_t waypoint_handler_set_waypoint_from_frame(waypoint_struct_t* current_waypoint, global_position_t origin);

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

    local_position_t waypoint_transfo;
    global_position_t waypoint_global;

    waypoint_handler->number_of_waypoints = 2;
    waypoint_handler->current_waypoint_count = -1;

    waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;

    // Start waypoint
    waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (waypoint_handler->mavlink_stream->sysid - 1));
    waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (waypoint_handler->mavlink_stream->sysid - 1));
    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Circle departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + angle_step * (waypoint_handler->mavlink_stream->sysid - 1))); // Desired yaw angle at MISSION (rotary wing)

    waypoint_handler->waypoint_list[0] = waypoint;

    // End waypoint
    waypoint_transfo.pos[X] = circle_radius * cos(angle_step * (waypoint_handler->mavlink_stream->sysid - 1) + PI);
    waypoint_transfo.pos[Y] = circle_radius * sin(angle_step * (waypoint_handler->mavlink_stream->sysid - 1) + PI);
    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Circle destination(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    waypoint.param4 = maths_rad_to_deg(angle_step * (waypoint_handler->mavlink_stream->sysid - 1)); // Desired yaw angle at MISSION (rotary wing)

    waypoint_handler->waypoint_list[1] = waypoint;

    if (packet->param5 == 1)
    {
        waypoint_handler->state->nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        waypoint_handler->start_wpt_time = time_keeper_get_ms();
    }
    else
    {
        waypoint_handler->state->nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (waypoint_handler->navigation->internal_state > NAV_ON_GND)
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

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;
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
        print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
        print_util_dbg_print("). For system:");
        print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
        print_util_dbg_print(".\r\n");
        waypoint.x = waypoint_global.latitude;
        waypoint.y = waypoint_global.longitude;
        waypoint.z = -altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame

        waypoint.autocontinue = packet->param5;
        if (i == 0)
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
        waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + atan2(waypoint_transfo.pos[Y], waypoint_transfo.pos[X]))); // Desired yaw angle at MISSION (rotary wing)

        waypoint_handler->waypoint_list[i] = waypoint;
    }

    if (packet->param5 == 1)
    {
        waypoint_handler->state->nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        waypoint_handler->start_wpt_time = time_keeper_get_ms();
    }
    else
    {
        waypoint_handler->state->nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (waypoint_handler->navigation->internal_state > NAV_ON_GND)
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

    local_position_t waypoint_transfo;

    global_position_t waypoint_global;

    waypoint_handler->number_of_waypoints = 2;
    waypoint_handler->current_waypoint_count = -1;

    waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;

    // Start waypoint
    if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc / 2.0f))
    {
        waypoint_transfo.pos[X] = lateral_dist;
        waypoint_transfo.pos[Y] = dist / 2.0f * (waypoint_handler->mavlink_stream->sysid - 1);
    }
    else
    {
        waypoint_transfo.pos[X] = - lateral_dist;
        waypoint_transfo.pos[Y] = dist / 2.0f * (waypoint_handler->mavlink_stream->sysid - 1 - (num_of_vhc / 2.0f));
    }

    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Stream departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc / 2.0f))
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_handler->waypoint_list[0] = waypoint;

    // End waypoint
    if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc / 2.0f))
    {
        waypoint_transfo.pos[X] = -lateral_dist;
        waypoint_transfo.pos[Y] = dist / 2.0f * (waypoint_handler->mavlink_stream->sysid - 1);
    }
    else
    {
        waypoint_transfo.pos[X] = lateral_dist;
        waypoint_transfo.pos[Y] = dist / 2.0f * (waypoint_handler->mavlink_stream->sysid - 1 - (num_of_vhc / 2.0f));
    }

    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Stream departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    if (waypoint_handler->mavlink_stream->sysid <= (num_of_vhc / 2.0f))
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

        waypoint_handler->start_wpt_time = time_keeper_get_ms();
    }
    else
    {
        waypoint_handler->state->nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (waypoint_handler->navigation->internal_state > NAV_ON_GND)
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

    float angle_step = 2.0f * PI / (float)floor((num_of_vhc - 1) / 2);

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;

    global_position_t waypoint_global;

    waypoint_handler->number_of_waypoints = 2;
    waypoint_handler->current_waypoint_count = -1;

    waypoint_transfo.origin = waypoint_handler->position_estimation->local_position.origin;

    // Start waypoint
    if ((float)((waypoint_handler->mavlink_stream->sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        if ((float)(waypoint_handler->mavlink_stream->sysid % num_of_vhc) == (num_of_vhc / 2.0f)) //higher ID
        {
            waypoint_transfo.pos[X] = lateral_dist;
            waypoint_transfo.pos[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo.pos[X] = lateral_dist + dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10));
            waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10));
        }
    }
    else
    {
        if (maths_f_abs(waypoint_handler->mavlink_stream->sysid % num_of_vhc - (num_of_vhc / 2.0f)) == (num_of_vhc / 2.0f))
        {
            waypoint_transfo.pos[X] = - lateral_dist;
            waypoint_transfo.pos[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo.pos[X] = - lateral_dist - dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10 - floor(num_of_vhc / 2)));
            waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10 - floor(num_of_vhc / 2)));
        }
    }

    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Swarm departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    if ((float)((waypoint_handler->mavlink_stream->sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_handler->waypoint_list[0] = waypoint;

    // End waypoint
    if ((float)((waypoint_handler->mavlink_stream->sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        if ((float)(waypoint_handler->mavlink_stream->sysid % num_of_vhc) == (num_of_vhc / 2.0f)) //higher ID
        {
            waypoint_transfo.pos[X] = - lateral_dist;
            waypoint_transfo.pos[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo.pos[X] = - lateral_dist + dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10));
            waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10));
        }
    }
    else
    {
        if (maths_f_abs(waypoint_handler->mavlink_stream->sysid % num_of_vhc - (num_of_vhc / 2.0f)) == (num_of_vhc / 2.0f))
        {
            waypoint_transfo.pos[X] = lateral_dist;
            waypoint_transfo.pos[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo.pos[X] = lateral_dist - dist * cos(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10 - floor(num_of_vhc / 2)));
            waypoint_transfo.pos[Y] = dist * sin(angle_step * ((waypoint_handler->mavlink_stream->sysid - 1) % 10 - floor(num_of_vhc / 2)));
        }
    }

    waypoint_transfo.pos[Z] = altitude;
    waypoint_global = coord_conventions_local_to_global_position(waypoint_transfo);

    print_util_dbg_print("Swarm departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo.pos[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo.pos[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(waypoint_handler->mavlink_stream->sysid, 10);
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
    if ((float)((waypoint_handler->mavlink_stream->sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
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

        waypoint_handler->start_wpt_time = time_keeper_get_ms();
    }
    else
    {
        waypoint_handler->state->nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (waypoint_handler->navigation->internal_state > NAV_ON_GND)
        {
            print_util_dbg_print("Resetting hold waypoint");
            waypoint_handler->hold_waypoint_set = false;
        }
    }
}


static void waypoint_handler_send_count(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication, 500000);

    mavlink_mission_request_list_t packet;

    mavlink_msg_mission_request_list_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        mavlink_message_t _msg;
        mavlink_msg_mission_count_pack(sysid,
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
            waypoint_handler->start_timeout = time_keeper_get_ms();
        }

        waypoint_handler->sending_waypoint_num = 0;
        print_util_dbg_print("Will send ");
        print_util_dbg_print_num(waypoint_handler->number_of_waypoints, 10);
        print_util_dbg_print(" waypoints\r\n");
    }
}

static void waypoint_handler_send_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    if (waypoint_handler->waypoint_sending)
    {
        mavlink_mission_request_t packet;

        mavlink_msg_mission_request_decode(msg, &packet);

        print_util_dbg_print("Asking for waypoint number ");
        print_util_dbg_print_num(packet.seq, 10);
        print_util_dbg_print("\r\n");

        // Check if this message is for this system and subsystem
        if (((uint8_t)packet.target_system == (uint8_t)sysid)
                && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
        {
            waypoint_handler->sending_waypoint_num = packet.seq;
            if (waypoint_handler->sending_waypoint_num < waypoint_handler->number_of_waypoints)
            {
                //  Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
                // mavlink_msg_mission_item_send (  mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq,
                //                                  uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1,
                //                                  float param2, float param3, float param4, float x, float y, float z)
                mavlink_message_t _msg;
                mavlink_msg_mission_item_pack(sysid,
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

                waypoint_handler->start_timeout = time_keeper_get_ms();
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
    mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication, 500000);

    mavlink_mission_count_t packet;

    mavlink_msg_mission_count_decode(msg, &packet);

    print_util_dbg_print("Count:");
    print_util_dbg_print_num(packet.count, 10);
    print_util_dbg_print("\r\n");

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        if (waypoint_handler->waypoint_receiving == false)
        {
            // comment these lines if you want to add new waypoints to the list instead of overwriting them
            waypoint_handler->num_waypoint_onboard = 0;
            waypoint_handler->number_of_waypoints = 0;
            //---//

            if ((packet.count + waypoint_handler->number_of_waypoints) > MAX_WAYPOINTS)
            {
                packet.count = MAX_WAYPOINTS - waypoint_handler->number_of_waypoints;
            }
            waypoint_handler->number_of_waypoints =  packet.count + waypoint_handler->number_of_waypoints;
            print_util_dbg_print("Receiving ");
            print_util_dbg_print_num(packet.count, 10);
            print_util_dbg_print(" new waypoints. ");
            print_util_dbg_print("New total number of waypoints:");
            print_util_dbg_print_num(waypoint_handler->number_of_waypoints, 10);
            print_util_dbg_print("\r\n");

            waypoint_handler->waypoint_receiving   = true;
            waypoint_handler->waypoint_sending     = false;
            waypoint_handler->waypoint_request_number = 0;


            waypoint_handler->start_timeout = time_keeper_get_ms();
        }

        mavlink_message_t _msg;
        mavlink_msg_mission_request_pack(sysid,
                                         waypoint_handler->mavlink_stream->compid,
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         waypoint_handler->waypoint_request_number);
        mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);

        print_util_dbg_print("Asking for waypoint ");
        print_util_dbg_print_num(waypoint_handler->waypoint_request_number, 10);
        print_util_dbg_print("\r\n");
    }

}

static void waypoint_handler_receive_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_communication_suspend_downstream(waypoint_handler->mavlink_communication, 500000);

    mavlink_mission_item_t packet;

    mavlink_msg_mission_item_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        waypoint_handler->start_timeout = time_keeper_get_ms();

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
        print_util_dbg_print_num(waypoint_handler->waypoint_request_number, 10);
        print_util_dbg_print(" receiving num :");
        print_util_dbg_print_num(packet.seq, 10);
        //print_util_dbg_print(" is it receiving :");
        //print_util_dbg_print_num(waypoint_handler->waypoint_receiving,10); // boolean value
        print_util_dbg_print("\r\n");

        //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
        if (packet.current == 2)
        {
            // verify we received the command;
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(sysid,
                                         waypoint_handler->mavlink_stream->compid,
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         MAV_MISSION_UNSUPPORTED);
            mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
        }
        else if (packet.current == 3)
        {
            //current = 3 is a flag to tell us this is a alt change only

            // verify we received the command
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
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
                    print_util_dbg_print_num(waypoint_handler->waypoint_request_number, 10);
                    print_util_dbg_print(" of ");
                    print_util_dbg_print_num(waypoint_handler->number_of_waypoints - waypoint_handler->num_waypoint_onboard, 10);
                    print_util_dbg_print("\r\n");

                    waypoint_handler->waypoint_list[waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number] = new_waypoint;
                    waypoint_handler->waypoint_request_number++;

                    if ((waypoint_handler->num_waypoint_onboard + waypoint_handler->waypoint_request_number) == waypoint_handler->number_of_waypoints)
                    {
                        MAV_MISSION_RESULT type = MAV_MISSION_ACCEPTED;

                        mavlink_message_t _msg;
                        mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
                                                     waypoint_handler->mavlink_stream->compid,
                                                     &_msg,
                                                     msg->sysid,
                                                     msg->compid, type);
                        mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);

                        print_util_dbg_print("flight plan received!\n");
                        waypoint_handler->waypoint_receiving = false;
                        waypoint_handler->num_waypoint_onboard = waypoint_handler->number_of_waypoints;

                        waypoint_handler->start_wpt_time = time_keeper_get_ms();

                        waypoint_handler->state->nav_plan_active = false;
                        waypoint_handler_nav_plan_init(waypoint_handler);
                    }
                    else
                    {
                        mavlink_message_t _msg;
                        mavlink_msg_mission_request_pack(waypoint_handler->mavlink_stream->sysid,
                                                         waypoint_handler->mavlink_stream->compid,
                                                         &_msg,
                                                         msg->sysid,
                                                         msg->compid,
                                                         waypoint_handler->waypoint_request_number);
                        mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);

                        print_util_dbg_print("Asking for waypoint ");
                        print_util_dbg_print_num(waypoint_handler->waypoint_request_number, 10);
                        print_util_dbg_print("\n");
                    }
                } //end of if (packet.seq == waypoint_handler->waypoint_request_number)
                else
                {
                    MAV_MISSION_RESULT type = MAV_MISSION_INVALID_SEQUENCE;

                    mavlink_message_t _msg;
                    mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
                                                 waypoint_handler->mavlink_stream->compid,
                                                 &_msg,
                                                 msg->sysid,
                                                 msg->compid,
                                                 type);
                    mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
                }
            } //end of if (waypoint_handler->waypoint_receiving)
            else
            {
                MAV_MISSION_RESULT type = MAV_MISSION_ERROR;
                print_util_dbg_print("Not ready to receive waypoints right now!\r\n");

                mavlink_message_t _msg;
                mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
                                             waypoint_handler->mavlink_stream->compid,
                                             &_msg,
                                             msg->sysid,
                                             msg->compid,
                                             type);
                mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
            } //end of else of if (waypoint_handler->waypoint_receiving)
        } //end of else (packet.current != 2 && !=3 )
    } //end of if this message is for this system and subsystem
}

static void waypoint_handler_set_current_waypoint(mavlink_waypoint_handler_t* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_set_current_t packet;

    mavlink_msg_mission_set_current_decode(msg, &packet);

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
            mavlink_msg_mission_current_pack(sysid,
                                             waypoint_handler->mavlink_stream->compid,
                                             &_msg,
                                             packet.seq);
            mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);

            print_util_dbg_print("Set current waypoint to number");
            print_util_dbg_print_num(packet.seq, 10);
            print_util_dbg_print("\r\n");

            waypoint_handler->start_wpt_time = time_keeper_get_ms();

            waypoint_handler->state->nav_plan_active = false;
            waypoint_handler_nav_plan_init(waypoint_handler);
        }
        else
        {
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
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
        for (uint8_t i = 0; i < waypoint_handler->number_of_waypoints; i++)
        {
            waypoint_handler->waypoint_list[i].current = 0;
        }
        waypoint_handler->waypoint_list[new_current].current = 1;

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(waypoint_handler->mavlink_stream->sysid,
                                         waypoint_handler->mavlink_stream->compid,
                                         &msg,
                                         new_current);
        mavlink_stream_send(waypoint_handler->mavlink_stream, &msg);

        print_util_dbg_print("Set current waypoint to number");
        print_util_dbg_print_num(new_current, 10);
        print_util_dbg_print("\r\n");

        waypoint_handler->start_wpt_time = time_keeper_get_ms();

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

    mavlink_msg_mission_clear_all_decode(msg, &packet);

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
            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream->sysid,
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

    if (!mav_modes_is_armed(waypoint_handler->state->mav_mode))
    {
        mavlink_msg_set_gps_global_origin_decode(msg, &packet);

        // Check if this message is for this system and subsystem
        // Due to possible bug from QGroundControl, no check of target_component and compid
        if ((uint8_t)packet.target_system == (uint8_t)sysid)
        {
            print_util_dbg_print("Set new home location.\r\n");
            waypoint_handler->position_estimation->local_position.origin.latitude = (double) packet.latitude / 10000000.0f;
            waypoint_handler->position_estimation->local_position.origin.longitude = (double) packet.longitude / 10000000.0f;
            waypoint_handler->position_estimation->local_position.origin.altitude = (float) packet.altitude / 1000.0f;

            print_util_dbg_print("New Home location: (");
            print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.latitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.longitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.origin.altitude * 1000.0f, 10);
            print_util_dbg_print(")\r\n");

            waypoint_handler->position_estimation->fence_set = false;
            position_estimation_set_new_fence_origin(waypoint_handler->position_estimation);

            mavlink_message_t _msg;
            mavlink_msg_gps_global_origin_pack(waypoint_handler->mavlink_stream->sysid,
                                               waypoint_handler->mavlink_stream->compid,
                                               &_msg,
                                               waypoint_handler->position_estimation->local_position.origin.latitude * 10000000.0f,
                                               waypoint_handler->position_estimation->local_position.origin.longitude * 10000000.0f,
                                               waypoint_handler->position_estimation->local_position.origin.altitude * 1000.0f);
            mavlink_stream_send(waypoint_handler->mavlink_stream, &_msg);
        }
    }
}

static mav_result_t waypoint_handler_continue_to_next_waypoint(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;
    bool force_next = false;
    uint32_t time_from_start_wpt = time_keeper_get_ms() - waypoint_handler->start_wpt_time;
    uint32_t time_wpt_limit = 5000;

    if (packet->param3 == 1)
    {
        // QGroundControl sends every message twice,
        //  therefore we do this test to avoid continuing two times in a row towards next waypoint
        if (time_from_start_wpt > time_wpt_limit) // 5 seconds
        {
            force_next = true;
        }
    }

    if ((waypoint_handler->number_of_waypoints > 0) && ((!waypoint_handler->state->nav_plan_active) || force_next))
    {
        print_util_dbg_print("All vehicles: Navigating to next waypoint.\r\n");

        waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 0;

        print_util_dbg_print("Continuing towards waypoint Nr");

        waypoint_handler->start_wpt_time = time_keeper_get_ms();

        if (waypoint_handler->current_waypoint_count == (waypoint_handler->number_of_waypoints - 1))
        {
            waypoint_handler->current_waypoint_count = 0;
        }
        else
        {
            waypoint_handler->current_waypoint_count++;
        }
        print_util_dbg_print_num(waypoint_handler->current_waypoint_count, 10);
        print_util_dbg_print("\r\n");
        waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 1;
        waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
        waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&waypoint_handler->current_waypoint, waypoint_handler->position_estimation->local_position.origin);

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(waypoint_handler->mavlink_stream->sysid,
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
    if (time_from_start_wpt < time_wpt_limit)
    {
        result = MAV_RESULT_ACCEPTED;
    }

    return result;
}

static mav_result_t waypoint_handler_is_arrived(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (packet->param2 == 32)
    {
        if (waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current == 0)
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

static bool waypoint_handler_take_off_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
    bool result = false;

    if (!waypoint_handler->hold_waypoint_set)
    {
        print_util_dbg_print("Automatic take-off, will hold position at: (");
        print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(waypoint_handler->position_estimation->local_position.pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(-10.0f, 10);
        print_util_dbg_print("), with heading of: ");
        print_util_dbg_print_num((int32_t)(waypoint_handler->position_estimation->local_position.heading * 180.0f / 3.14f), 10);
        print_util_dbg_print("\r\n");

        waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimation->local_position;
        waypoint_handler->waypoint_hold_coordinates.pos[Z] = -10.0;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(waypoint_handler->ahrs->qe);
        waypoint_handler->waypoint_hold_coordinates.heading = aero_attitude.rpy[2];

        waypoint_handler->navigation->dist2wp_sqr = 100.0f; // same position, 10m above => dist_sqr = 100.0f

        waypoint_handler->hold_waypoint_set = true;
    }

    if (waypoint_handler_mode_change(waypoint_handler))
    {
        if (waypoint_handler->navigation->dist2wp_sqr <= 16.0f)
        {
            result = true;

            print_util_dbg_print("Automatic take-off finished, dist2wp_sqr (10x):");
            print_util_dbg_print_num(waypoint_handler->navigation->dist2wp_sqr * 10.0f, 10);
            print_util_dbg_print(".\r\n");
        }
    }

    return result;
}

static mav_result_t waypoint_handler_start_stop_navigation(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    if (packet->param1 == MAV_GOTO_DO_HOLD)
    {
        if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
        {
            waypoint_handler_hold_init(waypoint_handler, waypoint_handler->position_estimation->local_position);

            waypoint_handler->navigation->internal_state = NAV_STOP_ON_POSITION;

            result = MAV_RESULT_ACCEPTED;
        }
        else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
        {
            //navigation->stop_nav_there = true;
            waypoint_handler->navigation->internal_state = NAV_STOP_THERE;

            waypoint_struct_t waypoint;

            waypoint.frame = packet->param3;
            waypoint.param4 = packet->param4;
            waypoint.x = packet->param5;
            waypoint.y = packet->param6;
            waypoint.z = packet->param7;

            local_position_t waypoint_goal = waypoint_handler_set_waypoint_from_frame(&waypoint, waypoint_handler->position_estimation->local_position.origin);
            waypoint_handler_hold_init(waypoint_handler, waypoint_goal);

            result = MAV_RESULT_ACCEPTED;
        }
    }
    else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
    {

        if (mav_modes_is_auto(waypoint_handler->mode))
        {
            waypoint_handler->navigation->internal_state = NAV_NAVIGATING;
        }
        else if (mav_modes_is_guided(waypoint_handler->mode))
        {
            waypoint_handler->navigation->internal_state = NAV_HOLD_POSITION;
        }


        result = MAV_RESULT_ACCEPTED;
    }

    return result;
}

static mav_result_t waypoint_handler_set_auto_takeoff(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (waypoint_handler->navigation->internal_state == NAV_ON_GND)
    {
        print_util_dbg_print("Starting automatic take-off from button\r\n");
        waypoint_handler->navigation->internal_state = NAV_TAKEOFF;
        waypoint_handler->hold_waypoint_set = false;

        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

static mav_result_t waypoint_handler_set_auto_landing(mavlink_waypoint_handler_t* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;


    if ((waypoint_handler->navigation->internal_state == NAV_NAVIGATING) || (waypoint_handler->navigation->internal_state == NAV_HOLD_POSITION))
    {
        result = MAV_RESULT_ACCEPTED;

        waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
        waypoint_handler->auto_landing_next_state = false;

        waypoint_handler->navigation->internal_state = NAV_LANDING;

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

static void waypoint_handler_auto_landing_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
    float rel_pos[3];

    bool next_state = false;

    if (!waypoint_handler->auto_landing_next_state)
    {
        waypoint_handler->auto_landing_next_state = true;

        switch (waypoint_handler->navigation->auto_landing_behavior)
        {
            case DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Cust: descent to small alt");
                waypoint_handler->state->mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                waypoint_handler->state->mav_mode_custom = CUST_DESCENT_TO_SMALL_ALTITUDE;
                waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimation->local_position;
                waypoint_handler->waypoint_hold_coordinates.pos[Z] = -5.0f;
                break;

            case DESCENT_TO_GND:
                print_util_dbg_print("Cust: descent to gnd");
                waypoint_handler->state->mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                waypoint_handler->state->mav_mode_custom = CUST_DESCENT_TO_GND;
                waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimation->local_position;
                waypoint_handler->waypoint_hold_coordinates.pos[Z] = 0.0f;
                waypoint_handler->navigation->alt_lpf = waypoint_handler->position_estimation->local_position.pos[2];
                break;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_handler->waypoint_hold_coordinates.pos[i] - waypoint_handler->position_estimation->local_position.pos[i];
        }

        waypoint_handler->navigation->dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (waypoint_handler->navigation->auto_landing_behavior == DESCENT_TO_GND)
    {
        waypoint_handler->navigation->alt_lpf = waypoint_handler->navigation->LPF_gain * (waypoint_handler->navigation->alt_lpf) + (1.0f - waypoint_handler->navigation->LPF_gain) * waypoint_handler->position_estimation->local_position.pos[2];
        if ((waypoint_handler->position_estimation->local_position.pos[2] > -0.1f) && (maths_f_abs(waypoint_handler->position_estimation->local_position.pos[2] - waypoint_handler->navigation->alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state = true;
        }
    }

    if (waypoint_handler->navigation->auto_landing_behavior == DESCENT_TO_SMALL_ALTITUDE)
    {
        if ((waypoint_handler->navigation->dist2wp_sqr < 3.0f) && (maths_f_abs(waypoint_handler->position_estimation->local_position.pos[2] - waypoint_handler->waypoint_hold_coordinates.pos[2]) < 0.5f))
        {
            next_state = true;
        }
    }

    if (next_state)
    {
        waypoint_handler->auto_landing_next_state = false;

        switch (waypoint_handler->navigation->auto_landing_behavior)
        {
            case DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_GND;
                break;

            case DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
                waypoint_handler->state->mav_mode_custom = CUSTOM_BASE_MODE;
                waypoint_handler->hold_waypoint_set = false;
                waypoint_handler->navigation->internal_state = NAV_ON_GND;
                waypoint_handler->state->mav_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                waypoint_handler->state->mav_state = MAV_STATE_STANDBY;
                break;
        }
    }
}

static void waypoint_handler_auto_land_on_tag_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
    float tag_pos[3];
    float cur_pos[3];

    // Set position vectors to shorten code later
    for (uint8_t i = 0; i < 3; i++)
    {
        tag_pos[i] = waypoint_handler->waypoint_hold_coordinates.pos[i];
        cur_pos[i] = waypoint_handler->position_estimation->local_position.pos[i];
    }

    bool next_state = false;

    // If the camera has detected the tag...
    if (waypoint_handler->navigation->land_on_tag_behavior == TAG_FOUND)
    {
        // The hold coordinates has been already been updated during the tag location reading...
        // Changed the z goal to ground if we are positioned directly above the tag
        float horizontal_distance_to_tag_sqr = (cur_pos[0] - tag_pos[0]) * (cur_pos[0] - tag_pos[0]) + (cur_pos[1] - tag_pos[1]) * (cur_pos[1] - tag_pos[1]);

        // If we are not above tag
        if (horizontal_distance_to_tag_sqr > ALLOWABLE_HORIZONTAL_TAG_OFFSET_SQR)
        {
            // Stay at tag search altitude
            waypoint_handler->waypoint_hold_coordinates.pos[2] = waypoint_handler->navigation->tag_search_altitude;

        }
        else // Descend to ground 
        {
            waypoint_handler->waypoint_hold_coordinates.pos[2] = 0.0f;

            // Set tag search altitude to current height, so it will reposition itself at this altitude if it drifts away
            waypoint_handler->navigation->tag_search_altitude = waypoint_handler->position_estimation->local_position.pos[2];
        }

        waypoint_handler->navigation->alt_lpf = waypoint_handler->position_estimation->local_position.pos[2];
    }
    else if (waypoint_handler->navigation->land_on_tag_behavior == TAG_NOT_FOUND)// Else we need to search for the tag ...
    {
        // Set the hold position to be the current location
        
    }

    // Calculate low-pass filter altitude for when to turn off motors
    waypoint_handler->navigation->alt_lpf = waypoint_handler->navigation->LPF_gain * (waypoint_handler->navigation->alt_lpf) + (1.0f - waypoint_handler->navigation->LPF_gain) * waypoint_handler->position_estimation->local_position.pos[2];
    if ((waypoint_handler->position_estimation->local_position.pos[2] > -0.1f) && (maths_f_abs(waypoint_handler->position_estimation->local_position.pos[2] - waypoint_handler->navigation->alt_lpf) <= 0.2f))
    {
        // Disarming
        next_state = true;
    }

    // Disarm if needed
    if (next_state)
    {
        print_util_dbg_print("Auto-landing on tag: disarming motors \r\n");
        waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
        waypoint_handler->state->mav_mode_custom = CUSTOM_BASE_MODE;
        waypoint_handler->hold_waypoint_set = false;
        waypoint_handler->navigation->internal_state = NAV_ON_GND;
        waypoint_handler->state->mav_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
        waypoint_handler->state->mav_state = MAV_STATE_STANDBY;
    }
}

static void waypoint_handler_state_machine(mavlink_waypoint_handler_t* waypoint_handler)
{
    mav_mode_t mode_local = waypoint_handler->state->mav_mode;

    float thrust;

    bool takeoff_result = false;

    switch (waypoint_handler->navigation->internal_state)
    {
        case NAV_ON_GND:
            thrust = manual_control_get_thrust(waypoint_handler->manual_control);

            if (thrust > -0.7f)
            {
                if (mav_modes_is_guided(mode_local) || mav_modes_is_auto(mode_local))
                {
                    waypoint_handler->hold_waypoint_set = false;
                    waypoint_handler->navigation->internal_state = NAV_TAKEOFF;
                }
                else
                {
                    waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
                }
            }
            break;

        case NAV_TAKEOFF:
            takeoff_result = waypoint_handler_take_off_handler(waypoint_handler);

            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            if (takeoff_result)
            {
                if (mav_modes_is_auto(mode_local))
                {
                    waypoint_handler->navigation->internal_state = NAV_NAVIGATING;
                }
                else if (mav_modes_is_guided(mode_local))
                {
                    waypoint_handler->navigation->internal_state = NAV_HOLD_POSITION;
                }
            }

            if ((!mav_modes_is_guided(mode_local)) && (!mav_modes_is_auto(mode_local)))
            {
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;

        case NAV_MANUAL_CTRL:
            if (mav_modes_is_auto(mode_local))
            {
                waypoint_handler->navigation->internal_state = NAV_NAVIGATING;
            }
            else if (mav_modes_is_guided(mode_local))
            {
                waypoint_handler->navigation->internal_state = NAV_HOLD_POSITION;
            }

            waypoint_handler->state->mav_mode_custom = CUSTOM_BASE_MODE;
            waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
            waypoint_handler->critical_next_state = false;
            waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
            break;

        case NAV_NAVIGATING:
            waypoint_handler_waypoint_navigation_handler(waypoint_handler, waypoint_handler_mode_change(waypoint_handler));

            waypoint_handler->navigation->goal = waypoint_handler->waypoint_coordinates;

            if (!mav_modes_is_auto(mode_local))
            {
                if (mav_modes_is_guided(mode_local))
                {
                    print_util_dbg_print("switching to NAV_HOLD_POSITION\r\n");
                    waypoint_handler->waypoint_hold_coordinates = waypoint_handler->position_estimation->local_position;
                    waypoint_handler->navigation->internal_state = NAV_HOLD_POSITION;
                }
                else
                {
                    print_util_dbg_print("switching to NAV_MANUAL_CTRL\r\n");
                    waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
                }
            }

            break;

        case NAV_HOLD_POSITION:
            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            if (mav_modes_is_auto(mode_local))
            {
                print_util_dbg_print("switching to NAV_NAVIGATING\r\n");
                waypoint_handler->navigation->internal_state = NAV_NAVIGATING;
            }
            else if (!mav_modes_is_guided(mode_local))
            {
                print_util_dbg_print("switching to NAV_MANUAL_CTRL\r\n");
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;

        case NAV_STOP_ON_POSITION:
            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;

        case NAV_STOP_THERE:
            waypoint_handler_stopping_handler(waypoint_handler);

            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;

        case NAV_LANDING:
            waypoint_handler_auto_landing_handler(waypoint_handler);

            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;

        case NAV_LAND_ON_TAG:
            waypoint_handler_auto_land_on_tag_handler(waypoint_handler);

            waypoint_handler->navigation->goal = waypoint_handler->waypoint_hold_coordinates;

            print_util_dbg_print("GOAL: (");
            print_util_dbg_print_num(waypoint_handler->navigation->goal.pos[0], 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->navigation->goal.pos[1], 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->navigation->goal.pos[2], 10);
            print_util_dbg_print(")");

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                waypoint_handler->navigation->internal_state = NAV_MANUAL_CTRL;
            }
            break;
    }
}

static void waypoint_handler_stopping_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
    float dist2wp_sqr;
    float rel_pos[3];

    rel_pos[X] = (float)(waypoint_handler->waypoint_hold_coordinates.pos[X] - waypoint_handler->position_estimation->local_position.pos[X]);
    rel_pos[Y] = (float)(waypoint_handler->waypoint_hold_coordinates.pos[Y] - waypoint_handler->position_estimation->local_position.pos[Y]);
    rel_pos[Z] = (float)(waypoint_handler->waypoint_hold_coordinates.pos[Z] - waypoint_handler->position_estimation->local_position.pos[Z]);

    dist2wp_sqr = vectors_norm_sqr(rel_pos);
    if (dist2wp_sqr < 25.0f)
    {
        //navigation->stop_nav = true;
        waypoint_handler->navigation->internal_state = NAV_STOP_ON_POSITION;
    }
}

static void waypoint_handler_critical_handler(mavlink_waypoint_handler_t* waypoint_handler)
{
    float rel_pos[3];
    bool next_state = false;

    //Check whether we entered critical mode due to a battery low level or a lost
    // connection with the GND station or are out of fence control
    if (waypoint_handler->state->battery_.is_low() ||
            waypoint_handler->state->connection_lost ||
            waypoint_handler->state->out_of_fence_2 ||
            waypoint_handler->position_estimation->gps->fix() == false)
    {
        if (waypoint_handler->navigation->critical_behavior != CRITICAL_LAND)
        {
            waypoint_handler->navigation->critical_behavior = CRITICAL_LAND;
            waypoint_handler->critical_next_state = false;
        }
    }

    if (!(waypoint_handler->critical_next_state))
    {
        waypoint_handler->critical_next_state = true;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(*waypoint_handler->navigation->qe);
        waypoint_handler->waypoint_critical_coordinates.heading = aero_attitude.rpy[2];

        switch (waypoint_handler->navigation->critical_behavior)
        {
            case CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Climbing to safe alt...\r\n");
                waypoint_handler->state->mav_mode_custom |= CUST_CRITICAL_CLIMB_TO_SAFE_ALT;

                waypoint_handler->waypoint_critical_coordinates.pos[X] = waypoint_handler->position_estimation->local_position.pos[X];
                waypoint_handler->waypoint_critical_coordinates.pos[Y] = waypoint_handler->position_estimation->local_position.pos[Y];
                waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;

                break;

            case FLY_TO_HOME_WP:
                waypoint_handler->state->mav_mode_custom &= ~CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
                waypoint_handler->state->mav_mode_custom |= CUST_CRITICAL_FLY_TO_HOME_WP;

                waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
                waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
                waypoint_handler->waypoint_critical_coordinates.pos[Z] = -30.0f;
                break;

            case HOME_LAND:
                waypoint_handler->state->mav_mode_custom &= ~CUST_CRITICAL_FLY_TO_HOME_WP;
                waypoint_handler->state->mav_mode_custom |= CUST_CRITICAL_LAND;
                waypoint_handler->waypoint_critical_coordinates.pos[X] = 0.0f;
                waypoint_handler->waypoint_critical_coordinates.pos[Y] = 0.0f;
                waypoint_handler->waypoint_critical_coordinates.pos[Z] = 5.0f;
                waypoint_handler->navigation->alt_lpf = waypoint_handler->navigation->position_estimation->local_position.pos[2];
                break;

            case CRITICAL_LAND:
                print_util_dbg_print("Critical land...\r\n");
                waypoint_handler->state->mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                waypoint_handler->state->mav_mode_custom |= CUST_CRITICAL_LAND;
                waypoint_handler->waypoint_critical_coordinates.pos[X] = waypoint_handler->position_estimation->local_position.pos[X];
                waypoint_handler->waypoint_critical_coordinates.pos[Y] = waypoint_handler->position_estimation->local_position.pos[Y];
                waypoint_handler->waypoint_critical_coordinates.pos[Z] = 5.0f;
                waypoint_handler->navigation->alt_lpf = waypoint_handler->position_estimation->local_position.pos[2];
                break;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_handler->waypoint_critical_coordinates.pos[i] - waypoint_handler->position_estimation->local_position.pos[i];
        }
        waypoint_handler->navigation->dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (waypoint_handler->navigation->critical_behavior == CRITICAL_LAND || waypoint_handler->navigation->critical_behavior == HOME_LAND)
    {
        waypoint_handler->navigation->alt_lpf = waypoint_handler->navigation->LPF_gain * waypoint_handler->navigation->alt_lpf + (1.0f - waypoint_handler->navigation->LPF_gain) * waypoint_handler->position_estimation->local_position.pos[2];
        if ((waypoint_handler->position_estimation->local_position.pos[2] > -0.1f) && (maths_f_abs(waypoint_handler->position_estimation->local_position.pos[2] - waypoint_handler->navigation->alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state = true;
        }
    }

    if ((waypoint_handler->navigation->critical_behavior == CLIMB_TO_SAFE_ALT) || (waypoint_handler->navigation->critical_behavior == FLY_TO_HOME_WP))
    {
        if (waypoint_handler->navigation->dist2wp_sqr < 3.0f)
        {
            next_state = true;
        }
    }

    if (next_state)
    {
        waypoint_handler->critical_next_state = false;
        switch (waypoint_handler->navigation->critical_behavior)
        {
            case CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Critical State! Flying to home waypoint.\r\n");
                waypoint_handler->navigation->critical_behavior = FLY_TO_HOME_WP;
                break;

            case FLY_TO_HOME_WP:
                if (waypoint_handler->state->out_of_fence_1)
                {
                    //stop auto navigation, to prevent going out of fence 1 again
                    waypoint_handler->waypoint_hold_coordinates = waypoint_handler->waypoint_critical_coordinates;
                    waypoint_handler->navigation->internal_state = NAV_STOP_ON_POSITION;
                    waypoint_handler_stopping_handler(waypoint_handler);
                    waypoint_handler->state->out_of_fence_1 = false;
                    waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
                    waypoint_handler->state->mav_state = MAV_STATE_ACTIVE;
                    waypoint_handler->state->mav_mode_custom &= ~CUST_CRITICAL_FLY_TO_HOME_WP;
                }
                else
                {
                    print_util_dbg_print("Critical State! Performing critical landing.\r\n");
                    waypoint_handler->navigation->critical_behavior = HOME_LAND;
                }
                break;

            case HOME_LAND:
            case CRITICAL_LAND:
                print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\r\n");
                waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
                waypoint_handler->state->mav_mode_custom = CUSTOM_BASE_MODE;
                waypoint_handler->navigation->internal_state = NAV_ON_GND;
                waypoint_handler->state->mav_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                waypoint_handler->state->mav_state = MAV_STATE_EMERGENCY;
                break;
        }
    }
}

static void waypoint_handler_waypoint_navigation_handler(mavlink_waypoint_handler_t* waypoint_handler, bool reset_hold_wpt)
{
    if (!reset_hold_wpt)
    {
        waypoint_handler->hold_waypoint_set = false;
    }

    if (waypoint_handler->state->nav_plan_active)
    {
        float rel_pos[3];

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_handler->waypoint_coordinates.pos[i] - waypoint_handler->position_estimation->local_position.pos[i];
        }
        waypoint_handler->navigation->dist2wp_sqr = vectors_norm_sqr(rel_pos);

        if (waypoint_handler->navigation->dist2wp_sqr < (waypoint_handler->current_waypoint.param2 * waypoint_handler->current_waypoint.param2))
        {
            print_util_dbg_print("Waypoint Nr");
            print_util_dbg_print_num(waypoint_handler->current_waypoint_count, 10);
            print_util_dbg_print(" reached, distance:");
            print_util_dbg_print_num(sqrt(waypoint_handler->navigation->dist2wp_sqr), 10);
            print_util_dbg_print(" less than :");
            print_util_dbg_print_num(waypoint_handler->current_waypoint.param2, 10);
            print_util_dbg_print(".\r\n");

            mavlink_message_t msg;
            mavlink_msg_mission_item_reached_pack(waypoint_handler->mavlink_stream->sysid,
                                                  waypoint_handler->mavlink_stream->compid,
                                                  &msg,
                                                  waypoint_handler->current_waypoint_count);
            mavlink_stream_send(waypoint_handler->mavlink_stream, &msg);

            waypoint_handler->travel_time = time_keeper_get_ms() - waypoint_handler->start_wpt_time;

            waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 0;
            if ((waypoint_handler->current_waypoint.autocontinue == 1) && (waypoint_handler->number_of_waypoints > 1))
            {
                print_util_dbg_print("Autocontinue towards waypoint Nr");

                waypoint_handler->start_wpt_time = time_keeper_get_ms();

                if (waypoint_handler->current_waypoint_count == (waypoint_handler->number_of_waypoints - 1))
                {
                    waypoint_handler->current_waypoint_count = 0;
                }
                else
                {
                    waypoint_handler->current_waypoint_count++;
                }
                print_util_dbg_print_num(waypoint_handler->current_waypoint_count, 10);
                print_util_dbg_print("\r\n");
                waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count].current = 1;
                waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];

                waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&waypoint_handler->current_waypoint, waypoint_handler->position_estimation->local_position.origin);

                mavlink_message_t msg;
                mavlink_msg_mission_current_pack(waypoint_handler->mavlink_stream->sysid,
                                                 waypoint_handler->mavlink_stream->compid,
                                                 &msg,
                                                 waypoint_handler->current_waypoint_count);
                mavlink_stream_send(waypoint_handler->mavlink_stream, &msg);

            }
            else
            {
                waypoint_handler->state->nav_plan_active = false;
                print_util_dbg_print("Stop\r\n");
            }
        }
    }
    else
    {
        if (!waypoint_handler->hold_waypoint_set)
        {
            waypoint_handler->hold_waypoint_set = true;
            waypoint_handler->waypoint_coordinates = waypoint_handler->position_estimation->local_position;
        }
    }
}

static bool waypoint_handler_mode_change(mavlink_waypoint_handler_t* waypoint_handler)
{
    mav_mode_t mode_local = waypoint_handler->state->mav_mode;
    mav_mode_t mode_nav = waypoint_handler->mode;

    bool result = false;

    if (mav_modes_are_equal_autonomous_modes(mode_local, mode_nav))
    {
        result = true;
    }

    return result;
}

static void waypoint_handler_control_time_out_waypoint_msg(mavlink_waypoint_handler_t* waypoint_handler)
{
    if (waypoint_handler->waypoint_sending || waypoint_handler->waypoint_receiving)
    {
        uint32_t tnow = time_keeper_get_ms();

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
}

static local_position_t waypoint_handler_set_waypoint_from_frame(waypoint_struct_t* current_waypoint, global_position_t origin)
{
    global_position_t waypoint_global;
    local_position_t waypoint_coor;
    global_position_t origin_relative_alt;

    for (uint8_t i = 0; i < 3; i++)
    {
        waypoint_coor.pos[i] = 0.0f;
    }
    waypoint_coor.origin = origin;
    waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);

    switch (current_waypoint->frame)
    {
        case MAV_FRAME_GLOBAL:
            waypoint_global.latitude    = current_waypoint->x;
            waypoint_global.longitude   = current_waypoint->y;
            waypoint_global.altitude    = current_waypoint->z;
            waypoint_global.heading     = maths_deg_to_rad(current_waypoint->param4);
            waypoint_coor = coord_conventions_global_to_local_position(waypoint_global, origin);

            print_util_dbg_print("waypoint_global: lat (x1e7):");
            print_util_dbg_print_num(waypoint_global.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(waypoint_global.longitude * 10000000, 10);
            print_util_dbg_print(" alt (x1000):");
            print_util_dbg_print_num(waypoint_global.altitude * 1000, 10);
            print_util_dbg_print(" waypoint_coor: x (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[X] * 100, 10);
            print_util_dbg_print(", y (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Y] * 100, 10);
            print_util_dbg_print(", z (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Z] * 100, 10);
            print_util_dbg_print(" localOrigin lat (x1e7):");
            print_util_dbg_print_num(origin.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(origin.longitude * 10000000, 10);
            print_util_dbg_print(" alt (x1000):");
            print_util_dbg_print_num(origin.altitude * 1000, 10);
            print_util_dbg_print("\r\n");
            break;

        case MAV_FRAME_LOCAL_NED:
            waypoint_coor.pos[X] = current_waypoint->x;
            waypoint_coor.pos[Y] = current_waypoint->y;
            waypoint_coor.pos[Z] = current_waypoint->z;
            waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);
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
            waypoint_global.heading     = maths_deg_to_rad(current_waypoint->param4);

            origin_relative_alt = origin;
            origin_relative_alt.altitude = 0.0f;
            waypoint_coor = coord_conventions_global_to_local_position(waypoint_global, origin_relative_alt);

            waypoint_coor.heading = maths_deg_to_rad(current_waypoint->param4);

            print_util_dbg_print("LocalOrigin: lat (x1e7):");
            print_util_dbg_print_num(origin_relative_alt.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(origin_relative_alt.longitude * 10000000, 10);
            print_util_dbg_print(" global alt (x1000):");
            print_util_dbg_print_num(origin.altitude * 1000, 10);
            print_util_dbg_print(" waypoint_coor: x (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[X] * 100, 10);
            print_util_dbg_print(", y (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Y] * 100, 10);
            print_util_dbg_print(", z (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Z] * 100, 10);
            print_util_dbg_print("\r\n");

            break;
        case MAV_FRAME_LOCAL_ENU:
            // Problem here: rec is not defined here
            //mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
            break;

    }

    return waypoint_coor;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool waypoint_handler_init(mavlink_waypoint_handler_t* waypoint_handler, position_estimation_t* position_estimation, navigation_t* navigation, const ahrs_t* ahrs, State* state, const manual_control_t* manual_control, mavlink_communication_t* mavlink_communication, const mavlink_stream_t* mavlink_stream)
{
    bool init_success = true;

    waypoint_handler->start_timeout = time_keeper_get_ms();
    waypoint_handler->timeout_max_waypoint = 10000;

    waypoint_handler->position_estimation = position_estimation;
    waypoint_handler->ahrs = ahrs;
    waypoint_handler->state = state;

    waypoint_handler->mavlink_communication = mavlink_communication;
    waypoint_handler->mavlink_stream = mavlink_stream;

    waypoint_handler->navigation = navigation;

    waypoint_handler->manual_control = manual_control;

    // init waypoint navigation
    waypoint_handler->number_of_waypoints = 0;
    waypoint_handler->num_waypoint_onboard = 0;

    waypoint_handler->sending_waypoint_num = 0;
    waypoint_handler->waypoint_request_number = 0;

    waypoint_handler->hold_waypoint_set = false;

    waypoint_handler->waypoint_sending = false;
    waypoint_handler->waypoint_receiving = false;

    waypoint_handler->start_wpt_time = time_keeper_get_ms();
    waypoint_handler->travel_time = 0;

    waypoint_handler->navigation->internal_state = NAV_ON_GND;
    waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
    waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;

    waypoint_handler->critical_next_state = false;
    waypoint_handler->auto_landing_next_state = false;

    waypoint_handler->mode = state->mav_mode;

    // Add callbacks for waypoint handler messages requests
    mavlink_message_handler_msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_MISSION_ITEM; // 39
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_receive_waypoint;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_REQUEST; // 40
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_send_waypoint;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_SET_CURRENT; // 41
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_set_current_waypoint;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_REQUEST_LIST; // 43
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_send_count;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_COUNT; // 44
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_receive_count;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_CLEAR_ALL; // 45
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_clear_waypoint_list;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_ACK; // 47
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_receive_ack_msg;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    callback.message_id     = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (mavlink_msg_callback_function_t) &waypoint_handler_set_home;
    callback.module_struct  = (handling_module_struct_t)        waypoint_handler;
    init_success &= mavlink_message_handler_add_msg_callback(&mavlink_communication->message_handler, &callback);

    // Add callbacks for waypoint handler commands requests
    mavlink_message_handler_cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // 20
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_set_current_waypoint_from_parameter;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_set_auto_landing;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_set_auto_takeoff;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_continue_to_next_waypoint;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    callbackcmd.command_id = MAV_CMD_CONDITION_LAST; // 159
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_set_scenario;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);


    callbackcmd.command_id = MAV_CMD_CONDITION_DISTANCE; // 114
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_is_arrived;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (mavlink_cmd_callback_function_t)    &waypoint_handler_start_stop_navigation;
    callbackcmd.module_struct =                                 waypoint_handler;
    init_success &= mavlink_message_handler_add_cmd_callback(&mavlink_communication->message_handler, &callbackcmd);

    return init_success;
}

bool waypoint_handler_update(mavlink_waypoint_handler_t* waypoint_handler)
{
    mav_mode_t mode_local = waypoint_handler->state->mav_mode;


    switch (waypoint_handler->state->mav_state)
    {
        case MAV_STATE_STANDBY:
            waypoint_handler->navigation->internal_state = NAV_ON_GND;
            waypoint_handler->hold_waypoint_set = false;
            waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
            waypoint_handler->critical_next_state = false;
            waypoint_handler->navigation->auto_landing_behavior = DESCENT_TO_SMALL_ALTITUDE;
            break;

        case MAV_STATE_ACTIVE:
            waypoint_handler->navigation->critical_behavior = CLIMB_TO_SAFE_ALT;
            waypoint_handler->critical_next_state = false;

            if (!waypoint_handler->state->nav_plan_active)
            {
                waypoint_handler_nav_plan_init(waypoint_handler);
            }

            waypoint_handler_state_machine(waypoint_handler);
            break;

        case MAV_STATE_CRITICAL:
            // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
            if (mav_modes_is_stabilise(mode_local))
            {
                if ((waypoint_handler->navigation->internal_state == NAV_NAVIGATING) || (waypoint_handler->navigation->internal_state == NAV_LANDING))
                {
                    waypoint_handler_critical_handler(waypoint_handler);

                    waypoint_handler->navigation->goal = waypoint_handler->waypoint_critical_coordinates;
                }
            }
            break;

        default:
            waypoint_handler->navigation->internal_state = NAV_ON_GND;
            break;
    }

    waypoint_handler->mode = mode_local;

    waypoint_handler_control_time_out_waypoint_msg(waypoint_handler);

    return true;
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
    print_util_dbg_print_num(waypoint_handler->num_waypoint_onboard, 10);
    print_util_dbg_print("\r\n");
}

void waypoint_handler_nav_plan_init(mavlink_waypoint_handler_t* waypoint_handler)
{
    float rel_pos[3];

    if ((waypoint_handler->number_of_waypoints > 0)
            && (waypoint_handler->position_estimation->init_gps_position || mav_modes_is_hil(waypoint_handler->state->mav_mode))
            && (waypoint_handler->waypoint_receiving == false))
    {
        for (uint8_t i = 0; i < waypoint_handler->number_of_waypoints; i++)
        {
            if ((waypoint_handler->waypoint_list[i].current == 1) && (!waypoint_handler->state->nav_plan_active))
            {
                waypoint_handler->current_waypoint_count = i;
                waypoint_handler->current_waypoint = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_count];
                waypoint_handler->waypoint_coordinates = waypoint_handler_set_waypoint_from_frame(&waypoint_handler->current_waypoint, waypoint_handler->position_estimation->local_position.origin);

                print_util_dbg_print("Waypoint Nr");
                print_util_dbg_print_num(i, 10);
                print_util_dbg_print(" set,\r\n");

                waypoint_handler->state->nav_plan_active = true;

                for (uint8_t j = 0; j < 3; j++)
                {
                    rel_pos[j] = waypoint_handler->waypoint_coordinates.pos[j] - waypoint_handler->position_estimation->local_position.pos[j];
                }
                waypoint_handler->navigation->dist2wp_sqr = vectors_norm_sqr(rel_pos);
            }
        }
    }
}

void waypoint_handler_hold_init(mavlink_waypoint_handler_t* waypoint_handler, local_position_t local_pos)
{
    waypoint_handler->hold_waypoint_set = true;

    waypoint_handler->waypoint_hold_coordinates = local_pos;

    //waypoint_handler->waypoint_hold_coordinates.heading = coord_conventions_get_yaw(waypoint_handler->ahrs->qe);
    //waypoint_handler->waypoint_hold_coordinates.heading = local_pos.heading;

    print_util_dbg_print("Position hold at: (");
    print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[X], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Y], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_handler->waypoint_hold_coordinates.pos[Z], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num((int32_t)(waypoint_handler->waypoint_hold_coordinates.heading * 180.0f / 3.14f), 10);
    print_util_dbg_print(")\r\n");

}

void mavlink_waypoint_handler_send_nav_time(mavlink_waypoint_handler_t* waypoint_handler, const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_named_value_int_pack(mavlink_stream->sysid,
                                     mavlink_stream->compid,
                                     msg,
                                     time_keeper_get_ms(),
                                     "travel_time",
                                     waypoint_handler->travel_time);
}
