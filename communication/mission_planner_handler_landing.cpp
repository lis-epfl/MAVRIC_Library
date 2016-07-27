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
 * \file mission_planner_handler_landing.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the landing state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_landing.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Mission_planner_handler_landing::set_behavior()
{
    local_position_t local_pos = hold_waypoint().local_pos();;

    // Change states and landing height appropriately
    switch (navigation_.auto_landing_behavior)
    {
        case Navigation::DESCENT_TO_SMALL_ALTITUDE:
        {
            print_util_dbg_print("Cust: descent to small alt");
            state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
            state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
            local_pos[Z] = navigation_.takeoff_altitude/2.0f;
            float heading = coord_conventions_get_yaw(ahrs_.qe);
            set_hold_waypoint(local_pos, heading);
            break;
        }

        case Navigation::DESCENT_TO_GND:
        {
            print_util_dbg_print("Cust: descent to gnd");
            state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
            state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
            local_pos[Z] = 0.0f;
            set_hold_waypoint(local_pos);
            navigation_.alt_lpf = ins_.position_lf()[Z];
            break;
        }
    }

    // Update navigation dist2wp_sqr
    float rel_pos[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        rel_pos[i] = local_pos[i] - ins_.position_lf()[i];
    }
    navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
}

void Mission_planner_handler_landing::auto_landing_handler(Mission_planner& mission_planner)
{
    bool next_state = false;

    // If there has not been a waypoint set for some reason, land on spot
    if (!hold_waypoint_set())
    {
        set_behavior();
    }

    // Determine if we should switch between the landing states
    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_GND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * (navigation_.alt_lpf) + (1.0f - navigation_.LPF_gain) * ins_.position_lf()[Z];
        if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - navigation_.alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state = true;
        }
    }

    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_SMALL_ALTITUDE)
    {
        if (maths_f_abs(ins_.position_lf()[Z] - hold_waypoint().local_pos()[Z]) < 0.5f)
        {
            next_state = true;
        }
    }

    if (next_state)
    {
        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_GND;
                set_behavior(); // Set behavior as we are still landing
                break;

            case Navigation::DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
                //Do not reset custom flag here, to be able to check after landing
                // in case something went wrong. Is reset while arming
                reset_hold_waypoint();
                navigation_.set_internal_state(Navigation::NAV_ON_GND);
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                // Dont need to set behavior, state switched to NAV_ON_GND
                break;
        }
    }
}

mav_result_t Mission_planner_handler_landing::set_auto_landing(Mission_planner_handler_landing* landing_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    // Only land if we are in an appropriate navigation state already
    if (   (landing_handler->navigation_.internal_state() == Navigation::NAV_NAVIGATING)
        || (landing_handler->navigation_.internal_state() == Navigation::NAV_HOLD_POSITION)
        || (landing_handler->navigation_.internal_state() == Navigation::NAV_STOP_ON_POSITION)
        || (landing_handler->navigation_.internal_state() == Navigation::NAV_STOP_THERE))
    {
        result = MAV_RESULT_ACCEPTED;

        // Change states
        landing_handler->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
        landing_handler->navigation_.set_internal_state(Navigation::NAV_LANDING);

        // Determine landing location
        local_position_t landing_position = landing_handler->ins_.position_lf();
        landing_position[Z] = -5.0f;
        if (packet->param1 == 1)
        {
            print_util_dbg_print("Landing at a given location\r\n");
            landing_position[X] = packet->param5;
            landing_position[Y] = packet->param6;

        }
        else
        {
            print_util_dbg_print("Landing on the spot\r\n");
        }

        // Set hold position for landing
        float heading = coord_conventions_get_yaw(landing_handler->ahrs_.qe);
        landing_handler->set_hold_waypoint(landing_position, heading);

        // If dubin, update structure
        /*
        if (landing_handler->navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
        {
            landing_handler->dubin_hold_init(landing_position);
        }*/

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");
        print_util_dbg_print("Landing at: (");
        local_position_t local_pos = landing_handler->hold_waypoint().local_pos();
        print_util_dbg_print_num(local_pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Z], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num((int32_t)(landing_handler->hold_waypoint().heading() * 180.0f / 3.14f), 10);
        print_util_dbg_print(")\r\n");

        landing_handler->set_behavior();
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}
/*
void Mission_planner_handler_landing::dubin_hold_init(local_position_t local_pos)
{
    switch (navigation_.dubin_state)
    {
        case DUBIN_INIT:
        case DUBIN_CIRCLE1:
            // Staying on the waypoint
            for (int i = 0; i < 3; ++i)
            {
                hold_waypoint().dubin().circle_center_2[i] = navigation_.goal().dubin().circle_center_1[i];
            }

            hold_waypoint().set_radius(navigation_.goal().dubin().radius_1);
            hold_waypoint().set_heading(coord_conventions_get_yaw(ahrs_.qe));

            navigation_.dubin_state = DUBIN_CIRCLE2;

            print_util_dbg_print("DUBINCIRCLE1: Position hold at: (");
            print_util_dbg_print_num(hold_waypoint().dubin().circle_center_2[X],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().dubin().circle_center_2[Y],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().dubin().circle_center_2[Z],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num((int32_t)(hold_waypoint().heading()*180.0f/3.14f),10);
            print_util_dbg_print(")\r\n");
        break;

        case DUBIN_STRAIGHT:
            navigation_.dubin_state = DUBIN_INIT;
            set_hold_waypoint(local_pos);

            hold_waypoint().set_loiter_time(0.0f);
            hold_waypoint().set_radius(navigation_.minimal_radius);
            hold_waypoint().set_heading(coord_conventions_get_yaw(ahrs_.qe));

            print_util_dbg_print("DUBINSTRAIGHT: Position hold at: (");
            print_util_dbg_print_num(hold_waypoint().local_pos()[X],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().local_pos()[Y],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().local_pos()[Z],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num((int32_t)(hold_waypoint().heading()*180.0f/3.14f),10);
            print_util_dbg_print(")\r\n");
        break;

        case DUBIN_CIRCLE2:
            // Staying on the waypoint
            //if (state_.nav_plan_active)
            //{
            //    waypoint_hold_coordinates = navigation_.goal;
            //}
            set_hold_waypoint(navigation_.goal());

            print_util_dbg_print("DUBIN_CIRCLE2: Position hold at: (");
            print_util_dbg_print_num(hold_waypoint().local_pos()[X],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().local_pos()[Y],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(hold_waypoint().local_pos()[Z],10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num((int32_t)(hold_waypoint().heading()*180.0f/3.14f),10);
            print_util_dbg_print(")\r\n");
        break;
    }
}*/

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_landing::Mission_planner_handler_landing(   const INS& ins,
                                                                    Navigation& navigation,
                                                                    const ahrs_t& ahrs,
                                                                    State& state,
                                                                    Mavlink_message_handler& message_handler):
            Mission_planner_handler(ins),
            navigation_(navigation),
            ahrs_(ahrs),
            state_(state),
            message_handler_(message_handler)
{
}

bool Mission_planner_handler_landing::init()
{
    bool init_success = true;

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_landing;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER_HANDLER_LANDING] constructor: ERROR\r\n");
    }

    return init_success;
}

void Mission_planner_handler_landing::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    auto_landing_handler(mission_planner);

    navigation_.set_goal(hold_waypoint());

    if (mode_local.is_manual())
    {
        navigation_.set_internal_state(Navigation::NAV_MANUAL_CTRL);
    }
}
