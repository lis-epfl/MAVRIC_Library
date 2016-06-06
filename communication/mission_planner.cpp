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
 * \file mission_planner.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The mission planner
 *
 ******************************************************************************/


#include "communication/mission_planner.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.h"
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


mav_result_t Mission_planner::set_current_waypoint_from_parameter(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;
    uint16_t new_current = 0;

    print_util_dbg_print("All MAVs: Return to first waypoint.\r\n");

    if (new_current < waypoint_handler->waypoint_count_)
    {
        for (uint8_t i = 0; i < waypoint_handler->waypoint_count_; i++)
        {
            waypoint_handler->waypoint_list[i].current = 0;
        }
        waypoint_handler->waypoint_list[new_current].current = 1;

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(waypoint_handler->mavlink_stream_.sysid(),
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &msg,
                                         new_current);
        waypoint_handler->mavlink_stream_.send(&msg);

        print_util_dbg_print("Set current waypoint to number");
        print_util_dbg_print_num(new_current, 10);
        print_util_dbg_print("\r\n");

        waypoint_handler->start_wpt_time_ = time_keeper_get_ms();

        waypoint_handler->state_.nav_plan_active = false;
        waypoint_handler->nav_plan_init();

        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}


void Mission_planner::set_home(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_set_gps_global_origin_t packet;

    if (!waypoint_handler->state_.is_armed())
    {
        mavlink_msg_set_gps_global_origin_decode(msg, &packet);

        // Check if this message is for this system and subsystem
        // Due to possible bug from QGroundControl, no check of target_component and compid
        if ((uint8_t)packet.target_system == (uint8_t)sysid)
        {
            print_util_dbg_print("Set new home location.\r\n");
            waypoint_handler->position_estimation_.local_position.origin.latitude = (double) packet.latitude / 10000000.0f;
            waypoint_handler->position_estimation_.local_position.origin.longitude = (double) packet.longitude / 10000000.0f;
            waypoint_handler->position_estimation_.local_position.origin.altitude = (float) packet.altitude / 1000.0f;

            print_util_dbg_print("New Home location: (");
            print_util_dbg_print_num(waypoint_handler->position_estimation_.local_position.origin.latitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->position_estimation_.local_position.origin.longitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_handler->position_estimation_.local_position.origin.altitude * 1000.0f, 10);
            print_util_dbg_print(")\r\n");


            waypoint_handler->position_estimation_.set_new_fence_origin();

            mavlink_message_t _msg;
            mavlink_msg_gps_global_origin_pack(waypoint_handler->mavlink_stream_.sysid(),
                                               waypoint_handler->mavlink_stream_.compid(),
                                               &_msg,
                                               waypoint_handler->position_estimation_.local_position.origin.latitude * 10000000.0f,
                                               waypoint_handler->position_estimation_.local_position.origin.longitude * 10000000.0f,
                                               waypoint_handler->position_estimation_.local_position.origin.altitude * 1000.0f);
            waypoint_handler->mavlink_stream_.send(&_msg);
        }
    }
}

mav_result_t Mission_planner::continue_to_next_waypoint(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;
    bool force_next = false;
    uint32_t time_from_start_wpt = time_keeper_get_ms() - waypoint_handler->start_wpt_time_;
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

    if ((waypoint_handler->waypoint_count_ > 0) && ((!waypoint_handler->state_.nav_plan_active) || force_next))
    {
        print_util_dbg_print("All vehicles: Navigating to next waypoint.\r\n");

        waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_index_].current = 0;

        print_util_dbg_print("Continuing towards waypoint Nr");

        waypoint_handler->start_wpt_time_ = time_keeper_get_ms();

        if (waypoint_handler->current_waypoint_index_ == (waypoint_handler->waypoint_count_ - 1))
        {
            waypoint_handler->current_waypoint_index_ = 0;
        }
        else
        {
            waypoint_handler->current_waypoint_index_++;
        }
        print_util_dbg_print_num(waypoint_handler->current_waypoint_index_, 10);
        print_util_dbg_print("\r\n");
        waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_index_].current = 1;
        waypoint_handler->current_waypoint_ = waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_index_];
        waypoint_handler->waypoint_coordinates_ = waypoint_handler.convert_to_waypoint_local_struct(&waypoint_handler->current_waypoint_,
                                                                                            waypoint_handler->position_estimation_.local_position.origin,
                                                                                            &waypoint_handler->navigation_.dubin_state);

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(waypoint_handler->mavlink_stream_.sysid(),
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &msg,
                                         waypoint_handler->current_waypoint_index_);
        waypoint_handler->mavlink_stream_.send(&msg);

        waypoint_handler->state_.nav_plan_active = true;

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

mav_result_t Mission_planner::is_arrived(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (packet->param2 == 32)
    {
        if (waypoint_handler->waypoint_list[waypoint_handler->current_waypoint_index_].current == 0)
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


mav_result_t Mission_planner::start_stop_navigation(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    if (packet->param1 == MAV_GOTO_DO_HOLD)
    {
        if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
        {
            waypoint_handler->hold_init(waypoint_handler->position_estimation_.local_position);

            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_STOP_ON_POSITION;

            result = MAV_RESULT_ACCEPTED;
        }
        else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
        {
            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_STOP_THERE;

            waypoint_struct_t waypoint;

            waypoint.frame = packet->param3;
            waypoint.param4 = packet->param4;
            waypoint.x = packet->param5;
            waypoint.y = packet->param6;
            waypoint.z = packet->param7;

            waypoint_local_struct_t waypoint_goal = waypoint_handler.convert_to_waypoint_local_struct(   &waypoint,
                                                                                                waypoint_handler->position_estimation_.local_position.origin,
                                                                                                &waypoint_handler->navigation_.dubin_state);
            waypoint_handler->hold_init(waypoint_goal.waypoint);

            result = MAV_RESULT_ACCEPTED;
        }
    }
    else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
    {
        if ( (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_STOP_THERE) || (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_STOP_ON_POSITION) )
        {
            waypoint_handler->navigation_.dubin_state = DUBIN_INIT;
        }

        if (mav_modes_is_auto(waypoint_handler->last_mode_))  // WHY USE LAST_MODE RATHER THAN STATE->MODE?
        {
            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
        }
        else if (mav_modes_is_guided(waypoint_handler->last_mode_))  // WHY USE LAST_MODE RATHER THAN STATE->MODE?
        {
            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
        }

        result = MAV_RESULT_ACCEPTED;
    }

    return result;
}

mav_result_t Mission_planner::set_auto_landing(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    mav_result_t result;


    if ((waypoint_handler->navigation_.internal_state_ == Navigation::NAV_NAVIGATING) || (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_HOLD_POSITION)
        || (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_STOP_ON_POSITION) || (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_STOP_THERE))
    {
        result = MAV_RESULT_ACCEPTED;

        waypoint_handler->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
        waypoint_handler->auto_landing_next_state_ = false;

        waypoint_handler->navigation_.internal_state_ = Navigation::NAV_LANDING;

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

void Mission_planner::auto_landing_handler()
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

void Mission_planner::state_machine()
{
    mav_mode_t mode_local = state_.mav_mode();

    float thrust;

    bool takeoff_result = false;
    bool new_mode = true;

    switch (navigation_.internal_state_)
    {
        case Navigation::NAV_ON_GND:
            thrust = manual_control_.get_thrust();

            if (thrust > -0.7f)
            {
                if (mav_modes_is_guided(mode_local) || mav_modes_is_auto(mode_local))
                {
                    hold_waypoint_set_ = false;
                    navigation_.internal_state_ = Navigation::NAV_TAKEOFF;
                }
                else
                {
                    navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
                }
            }
            break;

        case Navigation::NAV_TAKEOFF:
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
            break;

        case Navigation::NAV_MANUAL_CTRL:
            if (mav_modes_is_auto(mode_local))
            {
                navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
            }
            else if (mav_modes_is_guided(mode_local))
            {
                print_util_dbg_print("Switching to NAV_HOLD_POSITION from NAV_MANUAL_CTRL\r\n");
                hold_init(position_estimation_.local_position);
                navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
            }

            navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            critical_next_state_ = false;
            navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
            break;

        case Navigation::NAV_NAVIGATING:
            if (!mav_modes_is_auto(last_mode_))
            {
                new_mode = mode_change();
            }
            waypoint_navigation_handler(new_mode);

            if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
            {
                dubin_state_machine(&waypoint_coordinates_);
            }

            navigation_.goal = waypoint_coordinates_;

            if (!mav_modes_is_auto(mode_local))
            {
                if (mav_modes_is_guided(mode_local))
                {
                    print_util_dbg_print("Switching to NAV_HOLD_POSITION from NAV_NAVIGATING\r\n");
                    hold_init(position_estimation_.local_position);
                    navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
                }
                else
                {
                    print_util_dbg_print("Switching to NAV_MANUAL_CTRL from NAV_NAVIGATING\r\n");
                    navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
                }
            }

            break;

        case Navigation::NAV_HOLD_POSITION:
            if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
            {
                dubin_state_machine(&waypoint_hold_coordinates);
            }

            navigation_.goal = waypoint_hold_coordinates;

            if (mav_modes_is_auto(mode_local))
            {
                print_util_dbg_print("Switching to NAV_NAVIGATING from NAV_HOLD_POSITION\r\n");
                navigation_.dubin_state = DUBIN_INIT;
                navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
            }
            else if (!mav_modes_is_guided(mode_local))
            {
                print_util_dbg_print("Switching to Navigation::NAV_MANUAL_CTRL from Navigation::NAV_HOLD_POSITION\r\n");
                navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
            }
            break;

        case Navigation::NAV_STOP_ON_POSITION:
            if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
            {
                dubin_state_machine(&waypoint_hold_coordinates);
            }
            navigation_.goal = waypoint_hold_coordinates;

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
            }
            break;

        case Navigation::NAV_STOP_THERE:
            stopping_handler();
            if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
            {
                dubin_state_machine(&waypoint_hold_coordinates);
            }

            navigation_.goal = waypoint_hold_coordinates;

            if ((!mav_modes_is_auto(mode_local)) && (!mav_modes_is_guided(mode_local)))
            {
                navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
            }
            break;

        case Navigation::NAV_LANDING:
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
            break;
    }
}

void Mission_planner::stopping_handler()
{
    float dist2wp_sqr;
    float rel_pos[3];

    rel_pos[X] = (float)(waypoint_hold_coordinates.waypoint.pos[X] - position_estimation_.local_position.pos[X]);
    rel_pos[Y] = (float)(waypoint_hold_coordinates.waypoint.pos[Y] - position_estimation_.local_position.pos[Y]);
    rel_pos[Z] = (float)(waypoint_hold_coordinates.waypoint.pos[Z] - position_estimation_.local_position.pos[Z]);

    dist2wp_sqr = vectors_norm_sqr(rel_pos);
    if (dist2wp_sqr < 25.0f)
    {
        navigation_.internal_state_ = Navigation::NAV_STOP_ON_POSITION;
    }
}

void Mission_planner::critical_handler()
{
    float rel_pos[3];
    bool next_state_ = false;

    //Check whether we entered critical mode due to a battery low level or a lost
    // connection with the GND station or are out of fence control
    if (state_.battery_.is_low() ||
            state_.connection_lost ||
            state_.out_of_fence_2 ||
            position_estimation_.healthy() == false)
    {
        if (navigation_.critical_behavior != Navigation::CRITICAL_LAND)
        {
            navigation_.critical_behavior = Navigation::CRITICAL_LAND;
            critical_next_state_ = false;
        }
    }

    if (!(critical_next_state_))
    {
        critical_next_state_ = true;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(navigation_.qe);
        waypoint_critical_coordinates_.waypoint.heading = aero_attitude.rpy[2];

        switch (navigation_.critical_behavior)
        {
            case Navigation::CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Climbing to safe alt...\r\n");
                state_.mav_mode_custom |= CUST_CRITICAL_CLIMB_TO_SAFE_ALT;

                waypoint_critical_coordinates_.waypoint.pos[X] = position_estimation_.local_position.pos[X];
                waypoint_critical_coordinates_.waypoint.pos[Y] = position_estimation_.local_position.pos[Y];
                waypoint_critical_coordinates_.waypoint.pos[Z] = -30.0f;

                break;

            case Navigation::FLY_TO_HOME_WP:
                state_.mav_mode_custom &= ~CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
                state_.mav_mode_custom |= CUST_CRITICAL_FLY_TO_HOME_WP;

                waypoint_critical_coordinates_.waypoint.pos[X] = 0.0f;
                waypoint_critical_coordinates_.waypoint.pos[Y] = 0.0f;
                waypoint_critical_coordinates_.waypoint.pos[Z] = -30.0f;
                break;

            case Navigation::HOME_LAND:
                state_.mav_mode_custom &= ~CUST_CRITICAL_FLY_TO_HOME_WP;
                state_.mav_mode_custom |= CUST_CRITICAL_LAND;

                waypoint_critical_coordinates_.waypoint.pos[X] = 0.0f;
                waypoint_critical_coordinates_.waypoint.pos[Y] = 0.0f;
                waypoint_critical_coordinates_.waypoint.pos[Z] = 5.0f;
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;

            case Navigation::CRITICAL_LAND:
                print_util_dbg_print("Critical land...\r\n");

                state_.mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= CUST_CRITICAL_LAND;

                waypoint_critical_coordinates_.waypoint.pos[X] = position_estimation_.local_position.pos[X];
                waypoint_critical_coordinates_.waypoint.pos[Y] = position_estimation_.local_position.pos[Y];
                waypoint_critical_coordinates_.waypoint.pos[Z] = 5.0f;
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_critical_coordinates_.waypoint.pos[i] - position_estimation_.local_position.pos[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (navigation_.critical_behavior == Navigation::CRITICAL_LAND || navigation_.critical_behavior == Navigation::HOME_LAND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * navigation_.alt_lpf + (1.0f - navigation_.LPF_gain) * position_estimation_.local_position.pos[2];
        if ((position_estimation_.local_position.pos[2] > -0.1f) && (maths_f_abs(position_estimation_.local_position.pos[2] - navigation_.alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state_ = true;
        }
    }

    if ((navigation_.critical_behavior == Navigation::CLIMB_TO_SAFE_ALT) || (navigation_.critical_behavior == Navigation::FLY_TO_HOME_WP))
    {
        if (navigation_.dist2wp_sqr < 3.0f)
        {
            next_state_ = true;
        }
    }

    if (next_state_)
    {
        critical_next_state_ = false;
        switch (navigation_.critical_behavior)
        {
            case Navigation::CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Critical State! Flying to home waypoint.\r\n");
                navigation_.critical_behavior = Navigation::FLY_TO_HOME_WP;
                break;

            case Navigation::FLY_TO_HOME_WP:
                if (state_.out_of_fence_1)
                {
                    //stop auto navigation_, to prevent going out of fence 1 again
                    waypoint_hold_coordinates = waypoint_critical_coordinates_;
                    navigation_.internal_state_ = Navigation::NAV_STOP_ON_POSITION;
                    stopping_handler();
                    state_.out_of_fence_1 = false;
                    navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
                    state_.mav_state_ = MAV_STATE_ACTIVE;
                    state_.mav_mode_custom &= ~CUST_CRITICAL_FLY_TO_HOME_WP;
                }
                else
                {
                    print_util_dbg_print("Critical State! Performing critical landing.\r\n");
                    navigation_.critical_behavior = Navigation::HOME_LAND;
                }
                break;

            case Navigation::HOME_LAND:
            case Navigation::CRITICAL_LAND:
                print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\r\n");
                navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
                //state_.mav_mode_custom = CUSTOM_BASE_MODE;
                navigation_.internal_state_ = Navigation::NAV_ON_GND;
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_EMERGENCY;
                break;
        }
    }
}

void Mission_planner::waypoint_navigation_handler(bool reset_hold_wpt)
{
    if (!reset_hold_wpt)
    {
        if (state_.nav_plan_active)
        {
            navigation_.dubin_state = DUBIN_INIT;
        }
        hold_waypoint_set_ = false;
    }

    if (state_.nav_plan_active)
    {
        float rel_pos[3];
        uint16_t i;

        for (i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_coordinates_.waypoint.pos[i] - position_estimation_.local_position.pos[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);

        float margin = 0.0f;
        if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
        {
            margin = 36.0f;
        }

        if (navigation_.dist2wp_sqr < (current_waypoint_.param2 * current_waypoint_.param2 + margin) ||
               (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN && navigation_.dubin_state == DUBIN_CIRCLE2))
        {
            if (waypoint_list[current_waypoint_index_].current > 0)
            {
                print_util_dbg_print("Waypoint Nr");
                print_util_dbg_print_num(current_waypoint_index_, 10);
                print_util_dbg_print(" reached, distance:");
                print_util_dbg_print_num(sqrt(navigation_.dist2wp_sqr), 10);
                print_util_dbg_print(" less than :");
                print_util_dbg_print_num(current_waypoint_.param2, 10);
                print_util_dbg_print(".\r\n");

                mavlink_message_t msg;
                mavlink_msg_mission_item_reached_pack(mavlink_stream_.sysid(),
                                                      mavlink_stream_.compid(),
                                                      &msg,
                                                      current_waypoint_index_);
                mavlink_stream_.send(&msg);

                travel_time_ = time_keeper_get_ms() - start_wpt_time_;

                next_waypoint_.current = 0;
            }

            waypoint_list[current_waypoint_index_].current = 0;

            if ((current_waypoint_.autocontinue == 1) && (waypoint_count_ > 1))
            {
                if (next_waypoint_.current == 0)
                {
                    if (current_waypoint_index_ == (waypoint_count_ - 1))
                    {
                        current_waypoint_index_ = 0;
                    }
                    else
                    {
                        current_waypoint_index_++;
                    }
                    next_waypoint_ = waypoint_list[current_waypoint_index_];

                    next_waypoint_.current = 1;
                    dubin_state_t dubin_state;
                    waypoint_next_ = waypoint_handler_.convert_to_waypoint_local_struct(   &next_waypoint_,
                                                                                position_estimation_.local_position.origin,
                                                                                &dubin_state);
                }

                for (i=0;i<3;i++)
                {
                    rel_pos[i] = waypoint_next_.waypoint.pos[i]-waypoint_coordinates_.waypoint.pos[i];
                }

                float rel_pos_norm[3];

                vectors_normalize(rel_pos, rel_pos_norm);

                float outter_pt[3];
                outter_pt[X] = waypoint_next_.waypoint.pos[X]+rel_pos_norm[Y]*waypoint_next_.radius;
                outter_pt[Y] = waypoint_next_.waypoint.pos[Y]-rel_pos_norm[X]*waypoint_next_.radius;
                outter_pt[Z] = 0.0f;

                for (i=0;i<3;i++)
                {
                    rel_pos[i] = outter_pt[i]-position_estimation_.local_position.pos[i];
                }

                // float rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - position_estimation_.local_position.heading);
                float rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - atan2(position_estimation_.vel[Y], position_estimation_.vel[X]));

                if ( (maths_f_abs(rel_heading) < navigation_.heading_acceptance) || (navigation_.navigation_strategy == Navigation::strategy_t::DIRECT_TO) )
                {
                    print_util_dbg_print("Autocontinue towards waypoint Nr");
                    print_util_dbg_print_num(current_waypoint_index_,10);
                    print_util_dbg_print("\r\n");

                    start_wpt_time_ = time_keeper_get_ms();

                    waypoint_list[current_waypoint_index_].current = 1;
                    next_waypoint_.current = 0;
                    current_waypoint_ = waypoint_list[current_waypoint_index_];
                    waypoint_coordinates_ = waypoint_next_;
                    navigation_.dubin_state = DUBIN_INIT;

                    mavlink_message_t msg;
                    mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                     mavlink_stream_.compid(),
                                                     &msg,
                                                     current_waypoint_index_);
                    mavlink_stream_.send(&msg);
                }
            }
            else
            {
                state_.nav_plan_active = false;
                print_util_dbg_print("Stop\r\n");
            }
        }
    }
    else
    {
        if (!hold_waypoint_set_)
        {
            hold_init(position_estimation_.local_position);
            hold_waypoint_set_ = true;
        }
    }
}

bool Mission_planner::mode_change()
{
    return mav_modes_are_equal_autonomous_modes(state_.mav_mode(), last_mode_);
}




void Mission_planner::dubin_state_machine(waypoint_local_struct_t* waypoint_next_)
{
    float rel_pos[3];

    rel_pos[Z] = 0.0f;

    float dir_init_bf[3], dir_init[3], init_radius;

    float scalar_product, heading_diff;

    quat_t q_rot;
    aero_attitude_t attitude_yaw;

    switch(navigation_.dubin_state)
    {
        case DUBIN_INIT:
            print_util_dbg_print("DUBIN_INIT\r\n");
            if (state_.nav_plan_active)
            {
                init_radius = maths_f_abs(navigation_.goal.radius);
            }
            else
            {
                init_radius = navigation_.minimal_radius;
            }

            dir_init_bf[X] = init_radius;
            dir_init_bf[Y] = 0.0f;
            dir_init_bf[Z] = 0.0f;

            attitude_yaw = coord_conventions_quat_to_aero(navigation_.qe);
            attitude_yaw.rpy[ROLL] = 0.0f;
            attitude_yaw.rpy[PITCH] = 0.0f;
            attitude_yaw.rpy[YAW] = attitude_yaw.rpy[2];
            q_rot = coord_conventions_quaternion_from_aero(attitude_yaw);

            quaternions_rotate_vector(q_rot, dir_init_bf, dir_init);

            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = waypoint_next_->waypoint.pos[i]- position_estimation_.local_position.pos[i];
            }
            rel_pos[Z] = 0.0f;

            float dir_final[3];
            float pos_goal[3];
            float rel_pos_norm[3];

            if (vectors_norm_sqr(rel_pos) > 0.1f)
            {
                vectors_normalize(rel_pos,rel_pos_norm);

                dir_final[X] = -rel_pos_norm[Y]*waypoint_next_->radius;
                dir_final[Y] = rel_pos_norm[X]*waypoint_next_->radius;
                dir_final[Z] = 0.0f;

                for (uint8_t i = 0; i < 2; ++i)
                {
                    pos_goal[i] = waypoint_next_->waypoint.pos[i] + rel_pos_norm[i]*maths_f_abs(waypoint_next_->radius);
                }
                pos_goal[Z] = 0.0f;

                waypoint_next_->dubin = dubin_2d(    position_estimation_.local_position.pos,
                                                    pos_goal,
                                                    dir_init,
                                                    dir_final,
                                                    maths_sign(waypoint_next_->radius));

                navigation_.dubin_state = DUBIN_CIRCLE1;
            }
            else
            {
                dir_final[X] = -dir_init[Y];
                dir_final[Y] = dir_init[X];
                dir_final[Z] = 0.0f;

                for (uint8_t i = 0; i < 2; ++i)
                {
                    waypoint_next_->dubin.circle_center_2[i] = position_estimation_.local_position.pos[i];
                }
                waypoint_next_->dubin.circle_center_2[Z] = 0.0f;

                navigation_.dubin_state = DUBIN_CIRCLE2;
            }

            break;
        case DUBIN_CIRCLE1:
            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = waypoint_next_->dubin.tangent_point_2[i] - position_estimation_.local_position.pos[i];
            }
            // heading_diff = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - position_estimation_.local_position.heading);
            heading_diff = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - atan2(position_estimation_.vel[Y], position_estimation_.vel[X]));

            if (maths_f_abs(heading_diff) < navigation_.heading_acceptance)
            {
                navigation_.dubin_state = DUBIN_STRAIGHT;
            }
            break;
        case DUBIN_STRAIGHT:
            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = waypoint_next_->dubin.tangent_point_2[i] - position_estimation_.local_position.pos[i];
            }

            scalar_product = rel_pos[X] * waypoint_next_->dubin.line_direction[X] + rel_pos[Y] * waypoint_next_->dubin.line_direction[Y];
            if (scalar_product < 0.0f)
            {
                navigation_.dubin_state = DUBIN_CIRCLE2;
            }

        case DUBIN_CIRCLE2:
        break;
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner::Mission_planner(Position_estimation& position_estimation_, Navigation& navigation_, const ahrs_t& ahrs_, State& state_, const Manual_control& manual_control_, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream_, conf_t config):
            waypoint_count_(0),
            current_waypoint_index_(0),
            hold_waypoint_set_(false),
            start_wpt_time_(time_keeper_get_ms()),
            mavlink_stream_(mavlink_stream_),
            state_(state_),
            navigation_(navigation_),
            position_estimation_(position_estimation_),
            waypoint_sending_(false),
            waypoint_receiving_(false),
            sending_waypoint_num_(0),
            waypoint_request_number_(0),
            waypoint_onboard_count_(0),
            start_timeout_(time_keeper_get_ms()),
            timeout_max_waypoint_(10000),
            travel_time_(0),
            critical_next_state_(false),
            auto_landing_next_state_(0),
            last_mode_(state_.mav_mode()),
            ahrs_(ahrs_),
            manual_control_(manual_control_),
            config_(config)
{
    bool init_success = true;

    // init waypoint navigation

    next_waypoint_.current = 0;

    // Add callbacks for waypoint handler messages requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &set_home;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // 20
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_current_waypoint_from_parameter;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_landing;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &continue_to_next_waypoint;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_CONDITION_DISTANCE; // 114
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &is_arrived;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &start_stop_navigation;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    init_homing_waypoint();
    nav_plan_init();

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK_WAYPOINT_HANDLER] constructor: ERROR\r\n");
    }
}


bool Mission_planner::update(Mavlink_waypoint_handler* waypoint_handler)
{
    mav_mode_t mode_local = waypoint_handler->state_.mav_mode();


    switch (waypoint_handler->state_.mav_state_)
    {
        case MAV_STATE_STANDBY:
            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_ON_GND;
            waypoint_handler->hold_waypoint_set_ = false;
            waypoint_handler->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            waypoint_handler->critical_next_state_ = false;
            waypoint_handler->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
            break;

        case MAV_STATE_ACTIVE:
            waypoint_handler->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            waypoint_handler->critical_next_state_ = false;

            if (!waypoint_handler->state_.nav_plan_active)
            {
                waypoint_handler->nav_plan_init();
            }

            waypoint_handler->state_machine();
            break;

        case MAV_STATE_CRITICAL:
            // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
            if (mav_modes_is_stabilize(mode_local))
            {
                if ((waypoint_handler->navigation_.internal_state_ == Navigation::NAV_NAVIGATING) || (waypoint_handler->navigation_.internal_state_ == Navigation::NAV_LANDING))
                {
                    waypoint_handler->critical_handler();

                    waypoint_handler->navigation_.goal = waypoint_handler->waypoint_critical_coordinates_;
                }
            }
            break;

        default:
            waypoint_handler->navigation_.internal_state_ = Navigation::NAV_ON_GND;
            break;
    }

    waypoint_handler->last_mode_ = mode_local;

    waypoint_handler->control_time_out_waypoint_msg();

    return true;
}


void Mission_planner::nav_plan_init()
{
    float rel_pos[3];

    if ((waypoint_count_ > 0)
            && (position_estimation_.init_gps_position || mav_modes_is_hil(state_.mav_mode()))
            && (waypoint_receiving_ == false))
    {
        for (uint8_t i = 0; i < waypoint_count_; i++)
        {
            if ((waypoint_list[i].current == 1) && (!state_.nav_plan_active))
            {
                current_waypoint_index_ = i;
                current_waypoint_ = waypoint_list[current_waypoint_index_];
                waypoint_coordinates_ = waypoint_handler_.convert_to_waypoint_local_struct(   &current_waypoint_,
                                                                                    position_estimation_.local_position.origin,
                                                                                    &navigation_.dubin_state);

                print_util_dbg_print("Waypoint Nr");
                print_util_dbg_print_num(i, 10);
                print_util_dbg_print(" set,\r\n");

                state_.nav_plan_active = true;

                for (uint8_t j = 0; j < 3; j++)
                {
                    rel_pos[j] = waypoint_coordinates_.waypoint.pos[j] - position_estimation_.local_position.pos[j];
                }
                navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
            }
        }
    }
}

void Mission_planner::hold_init(local_position_t local_pos)
{
    hold_waypoint_set_ = true;

    switch(navigation_.navigation_strategy)
    {
        case Navigation::strategy_t::DIRECT_TO:
            waypoint_hold_coordinates.waypoint = local_pos;
            print_util_dbg_print("Position hold at: (");
            print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[X], 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Y], 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Z], 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num((int32_t)(waypoint_hold_coordinates.waypoint.heading * 180.0f / 3.14f), 10);
            print_util_dbg_print(")\r\n");
        break;

        case Navigation::strategy_t::DUBIN:
            switch (navigation_.dubin_state)
            {
                case DUBIN_INIT:
                case DUBIN_CIRCLE1:
                    // Staying on the waypoint
                    for (int i = 0; i < 3; ++i)
                    {
                        waypoint_hold_coordinates.dubin.circle_center_2[i] = navigation_.goal.dubin.circle_center_1[i];
                    }
                    waypoint_hold_coordinates.radius = navigation_.goal.dubin.radius_1;

                    navigation_.dubin_state = DUBIN_CIRCLE2;

                    print_util_dbg_print("DUBINCIRCLE1: Position hold at: (");
                    print_util_dbg_print_num(waypoint_hold_coordinates.dubin.circle_center_2[X],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.dubin.circle_center_2[Y],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.dubin.circle_center_2[Z],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num((int32_t)(waypoint_hold_coordinates.waypoint.heading*180.0f/3.14f),10);
                    print_util_dbg_print(")\r\n");
                break;

                case DUBIN_STRAIGHT:
                    navigation_.dubin_state = DUBIN_INIT;
                    waypoint_hold_coordinates.waypoint = local_pos;

                    waypoint_hold_coordinates.loiter_time = 0.0f;
                    waypoint_hold_coordinates.radius = 30.0f;

                    print_util_dbg_print("DUBINSTRAIGHT: Position hold at: (");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[X],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Y],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Z],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num((int32_t)(waypoint_hold_coordinates.waypoint.heading*180.0f/3.14f),10);
                    print_util_dbg_print(")\r\n");
                break;

                case DUBIN_CIRCLE2:
                    // Staying on the waypoint
                    if (state_.nav_plan_active)
                    {
                        waypoint_hold_coordinates = navigation_.goal;
                    }

                    print_util_dbg_print("DUBIN_CIRCLE2: Position hold at: (");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[X],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Y],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num(waypoint_hold_coordinates.waypoint.pos[Z],10);
                    print_util_dbg_print(", ");
                    print_util_dbg_print_num((int32_t)(waypoint_hold_coordinates.waypoint.heading*180.0f/3.14f),10);
                    print_util_dbg_print(")\r\n");
                break;
            }
        break;
    }
}

void Mission_planner::send_nav_time(const Mavlink_stream* mavlink_stream_, mavlink_message_t* msg)
{
    mavlink_msg_named_value_int_pack(mavlink_stream_->sysid(),
                                     mavlink_stream_->compid(),
                                     msg,
                                     time_keeper_get_ms(),
                                     "travel_time_",
                                     travel_time_);
}
