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

bool Mission_planner_handler_takeoff::take_off_handler(Mission_planner& mission_planner)
{
    bool result = false;
    float waypoint_radius = navigation_.takeoff_altitude*navigation_.takeoff_altitude*0.16f;

    if (!mission_planner.hold_waypoint_set())
    {
        print_util_dbg_print("Automatic take-off, will hold position at: (");
        print_util_dbg_print_num(position_estimation_.local_position.pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(position_estimation_.local_position.pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(navigation_.takeoff_altitude, 10);
        print_util_dbg_print("), with heading of: ");
        print_util_dbg_print_num((int32_t)(position_estimation_.local_position.heading * 180.0f / 3.14f), 10);
        print_util_dbg_print("\r\n");

        local_position_t takeoff_pos = position_estimation_.local_position;
        takeoff_pos.pos[Z] = navigation_.takeoff_altitude;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(ahrs_.qe);
        takeoff_pos.heading = aero_attitude.rpy[2];

        mission_planner.waypoint_hold_coordinates.set_local_pos(takeoff_pos);

        navigation_.dist2wp_sqr = mission_planner.waypoint_hold_coordinates.local_pos().pos[Z] * mission_planner.waypoint_hold_coordinates.local_pos().pos[Z];

        mission_planner.set_hold_waypoint_set(true);
    }

    if (mission_planner.mode_change())
    {
        switch(navigation_.navigation_strategy)
        {
            case Navigation::strategy_t::DIRECT_TO:
               if (navigation_.dist2wp_sqr <= waypoint_radius)
                {
                    result = true;
                }
            break;

            case Navigation::strategy_t::DUBIN:
                if (state_.autopilot_type == MAV_TYPE_QUADROTOR)
                {
                    if (navigation_.dist2wp_sqr <= waypoint_radius)
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
            /*
            if (!state_.nav_plan_active)
            {
                waypoint_coordinates_ = mission_planner.waypoint_hold_coordinates;
                waypoint_coordinates_.radius = 0.0f;
            }*/

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
        takeoff_handler->mission_planner_.set_hold_waypoint_set(false);

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

Mission_planner_handler_takeoff::Mission_planner_handler_takeoff(   Mission_planner& mission_planner,
                                                                    Position_estimation& position_estimation,
                                                                    Navigation& navigation,
                                                                    const ahrs_t& ahrs,
                                                                    State& state,
                                                                    Mavlink_message_handler& message_handler):
            mission_planner_(mission_planner),
            position_estimation_(position_estimation),
            navigation_(navigation),
            ahrs_(ahrs),
            state_(state),
            message_handler_(message_handler)
{

}

bool Mission_planner_handler_takeoff::init()
{
    bool init_success = true;

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_takeoff;
    callbackcmd.module_struct =                                 this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER_HANDLER_TAKEOFF] constructor: ERROR\r\n");
    }

    return init_success;
}

void Mission_planner_handler_takeoff::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    bool takeoff_result = take_off_handler(mission_planner);

    navigation_.goal = mission_planner.waypoint_hold_coordinates;

    if (takeoff_result)
    {
        if (mode_local.is_auto())
        {
            navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
        }
        else if (mode_local.ctrl_mode() == Mav_mode::POSITION_HOLD)
        {
            navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
        }
    }

    if (mode_local.is_manual())
    {
        print_util_dbg_print("Switching to NAV_MANUAL_CTRL from NAV_TAKEOFF\r\n");
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}
