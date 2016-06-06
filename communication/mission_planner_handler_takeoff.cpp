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
 * \file mission_planner_handler_takeoff.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the takeoff state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_takeoff.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Mission_planner_handler_takeoff::take_off_handler()
{
    bool result = false;

    if (!hold_waypoint_set_)
    {
        print_util_dbg_print("Automatic take-off, will hold position at: (");
        print_util_dbg_print_num(position_estimation_.local_position.pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(position_estimation_.local_position.pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(-10.0f, 10);
        print_util_dbg_print("), with heading of: ");
        print_util_dbg_print_num((int32_t)(position_estimation_.local_position.heading * 180.0f / 3.14f), 10);
        print_util_dbg_print("\r\n");

        waypoint_hold_coordinates.waypoint = position_estimation_.local_position;
        waypoint_hold_coordinates.waypoint.pos[Z] = navigation_.takeoff_altitude;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(ahrs_.qe);
        waypoint_hold_coordinates.waypoint.heading = aero_attitude.rpy[2];

        navigation_.dist2wp_sqr = waypoint_hold_coordinates.waypoint.pos[Z] * waypoint_hold_coordinates.waypoint.pos[Z];

        hold_waypoint_set_ = true;
    }

    if (mode_change())
    {
        switch(navigation_.navigation_strategy)
        {
            case Navigation::strategy_t::DIRECT_TO:
               if (navigation_.dist2wp_sqr <= 16.0f)
                {
                    result = true;
                }
            break;

            case Navigation::strategy_t::DUBIN:
                if (state_.autopilot_type == MAV_TYPE_QUADROTOR)
                {
                    if (navigation_.dist2wp_sqr <= 16.0f)
                    {
                        result = true;
                    }
                }
                else
                {
                    if (position_estimation_.local_position.pos[Z] <= navigation_.takeoff_altitude)
                    {
                        result = true;
                    }
                }
            break;
        }

        if (result)
        {
            navigation_.dubin_state = DUBIN_INIT;

            print_util_dbg_print("Automatic take-off finished.\r\n");
        }
    }

    return result;
}

mav_result_t Mission_planner_handler_takeoff::set_auto_takeoff(Mission_planner_handler_takeoff* takeoff_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (takeoff_handler->navigation_.internal_state_ == Navigation::NAV_ON_GND)
    {
        print_util_dbg_print("Starting automatic take-off from button\r\n");
        takeoff_handler->navigation_.internal_state_ = Navigation::NAV_TAKEOFF;
        takeoff_handler->hold_waypoint_set_ = false;

        result = MAV_RESULT_ACCEPTED;
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

Mission_planner_handler_takeoff::Mission_planner_handler_takeoff(   Position_estimation& position_estimation_,
                                                                    Navigation& navigation_,
                                                                    const ahrs_t& ahrs_,
                                                                    State& state_,
                                                                    Mavlink_message_handler& message_handler):
            position_estimation_(position_estimation_),
            state_(state_),
            ahrs_(ahrs_),
            navigation_(navigation_),
            manual_control_(manual_control_)
{
    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_takeoff;
    callbackcmd.module_struct =                                 this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);
}

Mission_planner_handler_on_ground::handle()
{
    mav_mode_t mode_local = state_.mav_mode();

    takeoff_result = take_off_handler();
    navigation_.goal = waypoint_hold_coordinates;

    if (takeoff_result)
    {
        if (mav_modes_is_auto(mode_local))
        {
            navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
        }
        else if (mav_modes_is_guided(mode_local))
        {
            navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
        }
    }

    if ((!mav_modes_is_guided(mode_local)) && (!mav_modes_is_auto(mode_local)))
    {
        print_util_dbg_print("Switching to NAV_MANUAL_CTRL from NAV_TAKEOFF\r\n");
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}
