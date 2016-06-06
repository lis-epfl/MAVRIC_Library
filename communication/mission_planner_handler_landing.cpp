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

void Mission_planner_handler_landing::auto_landing_handler()
{
    float rel_pos[3];

    bool next_state_ = false;

    if (!auto_landing_next_state_)
    {
        auto_landing_next_state_ = true;

        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Cust: descent to small alt");
                state_.mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= CUST_DESCENT_TO_SMALL_ALTITUDE;
                waypoint_hold_coordinates.waypoint = position_estimation_.local_position;
                waypoint_hold_coordinates.waypoint.pos[Z] = -5.0f;
                break;

            case Navigation::DESCENT_TO_GND:
                print_util_dbg_print("Cust: descent to gnd");
                state_.mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= CUST_DESCENT_TO_GND;
                waypoint_hold_coordinates.waypoint = position_estimation_.local_position;
                waypoint_hold_coordinates.waypoint.pos[Z] = 0.0f;
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_hold_coordinates.waypoint.pos[i] - position_estimation_.local_position.pos[i];
        }

        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_GND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * (navigation_.alt_lpf) + (1.0f - navigation_.LPF_gain) * position_estimation_.local_position.pos[2];
        if ((position_estimation_.local_position.pos[2] > -0.1f) && (maths_f_abs(position_estimation_.local_position.pos[2] - navigation_.alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state_ = true;
        }
    }

    if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_SMALL_ALTITUDE)
    {
        if ((navigation_.dist2wp_sqr < 3.0f) && (maths_f_abs(position_estimation_.local_position.pos[2] - waypoint_hold_coordinates.waypoint.pos[2]) < 0.5f))
        {
            next_state_ = true;
        }
    }

    if (next_state_)
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
                //state_.mav_mode_custom = CUSTOM_BASE_MODE;
                hold_waypoint_set_ = false;
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


    if ((landing_handler->navigation_.internal_state_ == Navigation::NAV_NAVIGATING) || (landing_handler->navigation_.internal_state_ == Navigation::NAV_HOLD_POSITION)
        || (landing_handler->navigation_.internal_state_ == Navigation::NAV_STOP_ON_POSITION) || (landing_handler->navigation_.internal_state_ == Navigation::NAV_STOP_THERE))
    {
        result = MAV_RESULT_ACCEPTED;

        landing_handler->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
        landing_handler->auto_landing_next_state_ = false;

        landing_handler->navigation_.internal_state_ = Navigation::NAV_LANDING;

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");
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

Mission_planner_handler_landing::Mission_planner_handler_landing(   Position_estimation& position_estimation_,
                                                                    Navigation& navigation_,
                                                                    State& state_,
                                                                    Mavlink_message_handler& message_handler):
            position_estimation_(position_estimation_),
            state_(state_),
            navigation_(navigation_),
            manual_control_(manual_control_)
{
    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_landing;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);
}

Mission_planner_handler_landing::handle(Mission_planner& mission_planner)
{
    mav_mode_t mode_local = state_.mav_mode();

    auto_landing_handler();

    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
    {
        dubin_state_machine(&waypoint_hold_coordinates);
    }

    navigation_.goal = waypoint_hold_coordinates;

    if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
    {
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}
