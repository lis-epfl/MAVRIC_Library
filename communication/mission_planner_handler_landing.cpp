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

void Mission_planner_handler_landing::auto_landing_handler(Mission_planner& mission_planner)
{
    float rel_pos[3];

    bool next_state = false;

    if (!auto_landing_next_state_)
    {
        auto_landing_next_state_ = true;

        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
            {
                print_util_dbg_print("Cust: descent to small alt");
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
                local_position_t local_pos = position_estimation_.local_position;
                local_pos.pos[Z] = navigation_.takeoff_altitude/2.0f;
                mission_planner.waypoint_hold_coordinates.set_local_pos(local_pos);
                break;
            }

            case Navigation::DESCENT_TO_GND:
            {
                print_util_dbg_print("Cust: descent to gnd");
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
                local_position_t local_pos = mission_planner.waypoint_hold_coordinates.local_pos();
                local_pos.pos[Z] = 0.0f;
                mission_planner.waypoint_hold_coordinates.set_local_pos(local_pos);
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;
            }
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = mission_planner.waypoint_hold_coordinates.local_pos().pos[i] - position_estimation_.local_position.pos[i];
        }

        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_GND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * (navigation_.alt_lpf) + (1.0f - navigation_.LPF_gain) * position_estimation_.local_position.pos[2];
        if ((position_estimation_.local_position.pos[2] > -0.1f) && (maths_f_abs(position_estimation_.local_position.pos[2] - navigation_.alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state = true;
        }
    }

    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_SMALL_ALTITUDE)
    {
        if (maths_f_abs(position_estimation_.local_position.pos[2] - mission_planner.waypoint_hold_coordinates.local_pos().pos[2]) < 0.5f)
        {
            next_state = true;
        }
    }

    if (next_state)
    {
        auto_landing_next_state_ = false;

        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_GND;
                break;

            case Navigation::DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
                //Do not reset custom flag here, to be able to check after landing
                // in case something went wrong. Is reset while arming
                mission_planner.set_hold_waypoint_set(false);
                navigation_.internal_state_ = Navigation::NAV_ON_GND;
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                break;
        }
    }
}

mav_result_t Mission_planner_handler_landing::set_auto_landing(Mission_planner_handler_landing* landing_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;


    if (   (landing_handler->navigation_.internal_state_ == Navigation::NAV_NAVIGATING)
        || (landing_handler->navigation_.internal_state_ == Navigation::NAV_HOLD_POSITION)
        || (landing_handler->navigation_.internal_state_ == Navigation::NAV_STOP_ON_POSITION)
        || (landing_handler->navigation_.internal_state_ == Navigation::NAV_STOP_THERE))
    {
        result = MAV_RESULT_ACCEPTED;

        landing_handler->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
        landing_handler->auto_landing_next_state_ = false;

        landing_handler->navigation_.internal_state_ = Navigation::NAV_LANDING;

        //waypoint_handler->navigation_.dubin_state = DUBIN_INIT;

        local_position_t landing_position = landing_handler->position_estimation_.local_position;
        landing_position.pos[Z] = -5.0f;
        if (packet->param1 == 1)
        {
            print_util_dbg_print("Landing at a given location\r\n");

            landing_position.pos[X] = packet->param5;
            landing_position.pos[Y] = packet->param6;

            landing_position.heading = landing_handler->position_estimation_.local_position.heading;

        }
        else
        {
            print_util_dbg_print("Landing on the spot\r\n");
        }

        if (landing_handler->navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
        {
            landing_handler->mission_planner_.dubin_hold_init(landing_position);
        }
        else
        {
            landing_handler->mission_planner_.waypoint_hold_coordinates.set_local_pos(landing_position);
        }

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");

        print_util_dbg_print("Landing at: (");
        print_util_dbg_print_num(landing_handler->mission_planner_.waypoint_hold_coordinates.local_pos().pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(landing_handler->mission_planner_.waypoint_hold_coordinates.local_pos().pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(landing_handler->mission_planner_.waypoint_hold_coordinates.local_pos().pos[Z], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num((int32_t)(landing_handler->mission_planner_.waypoint_hold_coordinates.local_pos().heading * 180.0f / 3.14f), 10);
        print_util_dbg_print(")\r\n");
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

Mission_planner_handler_landing::Mission_planner_handler_landing(   Mission_planner& mission_planner,
                                                                    Position_estimation& position_estimation,
                                                                    Navigation& navigation,
                                                                    State& state,
                                                                    Mavlink_message_handler& message_handler):
            mission_planner_(mission_planner),
            position_estimation_(position_estimation),
            navigation_(navigation),
            state_(state),
            auto_landing_next_state_(false)
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
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER_HANDLER_LANDING] constructor: ERROR\r\n");
    }
}

void Mission_planner_handler_landing::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    auto_landing_handler(mission_planner);

    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
    {
        mission_planner.dubin_state_machine(&mission_planner.waypoint_hold_coordinates);
    }

    navigation_.goal = mission_planner.waypoint_hold_coordinates;

    if (mode_local.is_manual())
    {
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}
