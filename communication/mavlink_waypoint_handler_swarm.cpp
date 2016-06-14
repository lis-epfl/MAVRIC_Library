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

#include <cstdlib>

#include "communication/mavlink_waypoint_handler_swarm.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

mav_result_t Mavlink_waypoint_handler_swarm::set_scenario(Mavlink_waypoint_handler_swarm* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (packet->param1 == 1)
    {
        waypoint_handler->set_circle_scenario(packet);

        result = MAV_RESULT_ACCEPTED;
    }
    else if (packet->param1 == 2)
    {
        waypoint_handler->set_circle_uniform_scenario(packet);

        result = MAV_RESULT_ACCEPTED;
    }
    else if (packet->param1 == 3)
    {
        waypoint_handler->set_stream_scenario(packet);

        result = MAV_RESULT_ACCEPTED;
    }
    else if (packet->param1 == 4)
    {
        waypoint_handler->set_swarm_scenario(packet);

        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_UNSUPPORTED;
    }

    return result;
}

void Mavlink_waypoint_handler_swarm::set_circle_scenario(mavlink_command_long_t* packet)
{
    float circle_radius = packet->param2;
    float num_of_vhc = packet->param3;
    float altitude = -packet->param4;

    float angle_step = 2.0f * PI / num_of_vhc;

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;
    global_position_t waypoint_global;

    waypoint_count_ = 2;
    current_waypoint_index_ = -1;

    // Start waypoint
    uint32_t sysid = mavlink_stream_.sysid();
    waypoint_transfo[X] = circle_radius * cos(angle_step * (sysid - 1));
    waypoint_transfo[Y] = circle_radius * sin(angle_step * (sysid - 1));
    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Circle departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
    print_util_dbg_print(".\r\n");
    waypoint.x = waypoint_global.latitude;
    waypoint.y = waypoint_global.longitude;
    waypoint.z = - altitude; // Positive Z axis is pointing downwards, so the altitude is negative is the local frame

    waypoint.autocontinue = packet->param5;
    waypoint.current = (packet->param5 == 1);
    waypoint.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    waypoint.command = MAV_CMD_NAV_WAYPOINT;

    waypoint.param1 = 10.0f; // Hold time in decimal seconds
    waypoint.param2 = 4.0f; // Acceptance radius in meters
    waypoint.param3 = 0.0f; //  0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
    waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + angle_step * (sysid - 1))); // Desired yaw angle at MISSION (rotary wing)

    waypoint_list[0] = waypoint;

    // End waypoint
    waypoint_transfo[X] = circle_radius * cos(angle_step * (sysid - 1) + PI);
    waypoint_transfo[Y] = circle_radius * sin(angle_step * (sysid - 1) + PI);
    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Circle destination(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
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
    waypoint.param4 = maths_rad_to_deg(angle_step * (sysid - 1)); // Desired yaw angle at MISSION (rotary wing)

    waypoint_list[1] = waypoint;

    if (packet->param5 == 1)
    {
        state_.nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        start_wpt_time_ = time_keeper_get_ms();
    }
    else
    {
        state_.nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (navigation_.internal_state_ > Navigation::NAV_ON_GND)
        {
            print_util_dbg_print("Resetting hold waypoint");
            hold_waypoint_set_ = false;
        }
    }
}

void Mavlink_waypoint_handler_swarm::set_circle_uniform_scenario(mavlink_command_long_t* packet)
{
    int16_t i;

    float circle_radius = packet->param2;
    float altitude = -packet->param4;

    float x;

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;
    global_position_t waypoint_global;

    waypoint_count_ = 0;
    current_waypoint_index_ = -1;

    for (i = 0; i < 10; ++i)
    {
        waypoint_count_++;

        x = 2.0f * PI * rand();

        // Start waypoint
        waypoint_transfo[X] = circle_radius * cos(x);
        waypoint_transfo[Y] = circle_radius * sin(x);
        waypoint_transfo[Z] = altitude;
        coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

        print_util_dbg_print("Circle uniform departure(x100): (");
        print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
        print_util_dbg_print("). For system:");
        print_util_dbg_print_num(mavlink_stream_.sysid(), 10);
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
        waypoint.param4 = maths_rad_to_deg(maths_calc_smaller_angle(PI + atan2(waypoint_transfo[Y], waypoint_transfo[X]))); // Desired yaw angle at MISSION (rotary wing)

        waypoint_list[i] = waypoint;
    }

    if (packet->param5 == 1)
    {
        state_.nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        start_wpt_time_ = time_keeper_get_ms();
    }
    else
    {
        state_.nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (navigation_.internal_state_ > Navigation::NAV_ON_GND)
        {
            print_util_dbg_print("Resetting hold waypoint");
            hold_waypoint_set_ = false;
        }
    }
}

void Mavlink_waypoint_handler_swarm::set_stream_scenario(mavlink_command_long_t* packet)
{
    float dist = packet->param2;
    float num_of_vhc = packet->param3;
    float lateral_dist = 30.0f; //packet->param4;
    float altitude = -packet->param4;

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;

    global_position_t waypoint_global;

    waypoint_count_ = 2;
    current_waypoint_index_ = -1;

    // Start waypoint
    uint32_t sysid = mavlink_stream_.sysid();
    if (sysid <= (num_of_vhc / 2.0f))
    {
        waypoint_transfo[X] = lateral_dist;
        waypoint_transfo[Y] = dist / 2.0f * (sysid - 1);
    }
    else
    {
        waypoint_transfo[X] = - lateral_dist;
        waypoint_transfo[Y] = dist / 2.0f * (sysid - 1 - (num_of_vhc / 2.0f));
    }

    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Stream departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
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
    if (sysid <= (num_of_vhc / 2.0f))
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_list[0] = waypoint;

    // End waypoint
    if (sysid <= (num_of_vhc / 2.0f))
    {
        waypoint_transfo[X] = -lateral_dist;
        waypoint_transfo[Y] = dist / 2.0f * (sysid - 1);
    }
    else
    {
        waypoint_transfo[X] = lateral_dist;
        waypoint_transfo[Y] = dist / 2.0f * (sysid - 1 - (num_of_vhc / 2.0f));
    }

    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Stream departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
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
    if (sysid <= (num_of_vhc / 2.0f))
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_list[1] = waypoint;

    if (packet->param5 == 1)
    {
        state_.nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        start_wpt_time_ = time_keeper_get_ms();
    }
    else
    {
        state_.nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (navigation_.internal_state_ > Navigation::NAV_ON_GND)
        {
            print_util_dbg_print("Resetting hold waypoint");
            hold_waypoint_set_ = false;
        }
    }
}

void Mavlink_waypoint_handler_swarm::set_swarm_scenario(mavlink_command_long_t* packet)
{
    float dist = packet->param2;
    uint8_t num_of_vhc = packet->param3;
    float lateral_dist = 40.0f; //packet->param4;
    float altitude = -packet->param4;

    float angle_step = 2.0f * PI / (float)floor((num_of_vhc - 1) / 2);

    waypoint_struct_t waypoint;

    local_position_t waypoint_transfo;

    global_position_t waypoint_global;

    waypoint_count_ = 2;
    current_waypoint_index_ = -1;

    // Start waypoint
    uint32_t sysid = mavlink_stream_.sysid();
    if ((float)((sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        if ((float)(sysid % num_of_vhc) == (num_of_vhc / 2.0f)) //higher ID
        {
            waypoint_transfo[X] = lateral_dist;
            waypoint_transfo[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo[X] = lateral_dist + dist * cos(angle_step * ((sysid - 1) % 10));
            waypoint_transfo[Y] = dist * sin(angle_step * ((sysid - 1) % 10));
        }
    }
    else
    {
        if (maths_f_abs(sysid % num_of_vhc - (num_of_vhc / 2.0f)) == (num_of_vhc / 2.0f))
        {
            waypoint_transfo[X] = - lateral_dist;
            waypoint_transfo[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo[X] = - lateral_dist - dist * cos(angle_step * ((sysid - 1) % 10 - floor(num_of_vhc / 2)));
            waypoint_transfo[Y] = dist * sin(angle_step * ((sysid - 1) % 10 - floor(num_of_vhc / 2)));
        }
    }

    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Swarm departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
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
    if ((float)((sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_list[0] = waypoint;

    // End waypoint
    if ((float)((sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        if ((float)(sysid % num_of_vhc) == (num_of_vhc / 2.0f)) //higher ID
        {
            waypoint_transfo[X] = - lateral_dist;
            waypoint_transfo[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo[X] = - lateral_dist + dist * cos(angle_step * ((sysid - 1) % 10));
            waypoint_transfo[Y] = dist * sin(angle_step * ((sysid - 1) % 10));
        }
    }
    else
    {
        if (maths_f_abs(sysid % num_of_vhc - (num_of_vhc / 2.0f)) == (num_of_vhc / 2.0f))
        {
            waypoint_transfo[X] = lateral_dist;
            waypoint_transfo[Y] = 0.0f;
        }
        else
        {
            waypoint_transfo[X] = lateral_dist - dist * cos(angle_step * ((sysid - 1) % 10 - floor(num_of_vhc / 2)));
            waypoint_transfo[Y] = dist * sin(angle_step * ((sysid - 1) % 10 - floor(num_of_vhc / 2)));
        }
    }

    waypoint_transfo[Z] = altitude;
    coord_conventions_local_to_global_position(waypoint_transfo, ins_.origin(), waypoint_global);

    print_util_dbg_print("Swarm departure(x100): (");
    print_util_dbg_print_num(waypoint_transfo[X] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Y] * 100.0f, 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(waypoint_transfo[Z] * 100.0f, 10);
    print_util_dbg_print("). For system:");
    print_util_dbg_print_num(sysid, 10);
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
    if ((float)((sysid - 1) % num_of_vhc) <= (num_of_vhc / 2.0f - 1.0f))
    {
        waypoint.param4 = 0.0f; // Desired yaw angle at MISSION (rotary wing)
    }
    else
    {
        waypoint.param4 = 180.0f; // Desired yaw angle at MISSION (rotary wing)
    }

    waypoint_list[1] = waypoint;

    if (packet->param5 == 1)
    {
        state_.nav_plan_active = true;
        print_util_dbg_print("Auto-continue, nav plan active");

        start_wpt_time_ = time_keeper_get_ms();
    }
    else
    {
        state_.nav_plan_active = false;
        print_util_dbg_print("nav plan inactive");
        if (navigation_.internal_state_ > Navigation::NAV_ON_GND)
        {
            print_util_dbg_print("Resetting hold waypoint");
            hold_waypoint_set_ = false;
        }
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_waypoint_handler_swarm::Mavlink_waypoint_handler_swarm(Position_estimation& position_estimation_,
                                                               Navigation& navigation_,
                                                               const ahrs_t& ahrs,
                                                               State& state_,
                                                               const Manual_control& manual_control,
                                                               Mavlink_message_handler& message_handler,
                                                               const Mavlink_stream& mavlink_stream_):
    Mavlink_waypoint_handler(position_estimation_, navigation_, ahrs, state_, manual_control, message_handler, mavlink_stream_)
{
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_CONDITION_LAST; // 159
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)    &set_scenario;
    callbackcmd.module_struct =                                 this;
    message_handler.add_cmd_callback(&callbackcmd);
}
