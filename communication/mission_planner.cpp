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

void Mission_planner::set_home(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_set_gps_global_origin_t packet;

    if (!mission_planner->state_.is_armed())
    {
        mavlink_msg_set_gps_global_origin_decode(msg, &packet);

        // Check if this message is for this system and subsystem
        // Due to possible bug from QGroundControl, no check of target_component and compid
        if ((uint8_t)packet.target_system == (uint8_t)sysid)
        {
            print_util_dbg_print("Set new home location.\r\n");
            mission_planner->position_estimation_.local_position.origin.latitude = (double) packet.latitude / 10000000.0f;
            mission_planner->position_estimation_.local_position.origin.longitude = (double) packet.longitude / 10000000.0f;
            mission_planner->position_estimation_.local_position.origin.altitude = (float) packet.altitude / 1000.0f;

            print_util_dbg_print("New Home location: (");
            print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.latitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.longitude * 10000000.0f, 10);
            print_util_dbg_print(", ");
            print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.altitude * 1000.0f, 10);
            print_util_dbg_print(")\r\n");


            mission_planner->position_estimation_.set_new_fence_origin();

            mavlink_message_t _msg;
            mavlink_msg_gps_global_origin_pack(mission_planner->mavlink_stream_.sysid(),
                                               mission_planner->mavlink_stream_.compid(),
                                               &_msg,
                                               mission_planner->position_estimation_.local_position.origin.latitude * 10000000.0f,
                                               mission_planner->position_estimation_.local_position.origin.longitude * 10000000.0f,
                                               mission_planner->position_estimation_.local_position.origin.altitude * 1000.0f);
            mission_planner->mavlink_stream_.send(&_msg);
        }
    }
}

mav_result_t Mission_planner::continue_to_next_waypoint(Mission_planner* mission_planner, mavlink_command_long_t* packet)
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

    if ((mission_planner->waypoint_handler_.waypoint_count() > 0) && ((!mission_planner->state_.nav_plan_active) || force_next))
    {
        print_util_dbg_print("All vehicles: Navigating to next waypoint.\r\n");

        mission_planner->start_wpt_time_ = time_keeper_get_ms();

        mission_planner->waypoint_handler_.advance_to_next_waypoint();

        mission_planner->waypoint_handler_.convert_to_waypoint_local_struct(&waypoint_handler->current_waypoint_,
                                                                            waypoint_handler->position_estimation_.local_position.origin,
                                                                            &waypoint_handler->navigation_.dubin_state);

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(mission_planner->mavlink_stream_.sysid(),
                                         mission_planner->mavlink_stream_.compid(),
                                         &msg,
                                         mission_planner->waypoint_handler.current_waypoint_index());
        mission_planner->mavlink_stream_.send(&msg);

        mission_planner->state_.nav_plan_active = true;

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
        if (waiting_at_waypoint_)
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


void Mission_planner::state_machine()
{
    mav_mode_t mode_local = state_.mav_mode();

    float thrust;

    bool takeoff_result = false;
    bool new_mode = true;

    switch (navigation_.internal_state_)
    {
        case Navigation::NAV_ON_GND:
            on_ground_handler_.handle(this);
            break;

        case Navigation::NAV_TAKEOFF:
            takeoff_handler_.handle(this);
            break;

        case Navigation::NAV_MANUAL_CTRL:
            manual_control_handler_.handle(this);
            break;

        case Navigation::NAV_NAVIGATING:
            navigating_handler_.handle(this);
            break;

        case Navigation::NAV_HOLD_POSITION:
            hold_position_handler_.handle(this);
            break;

        case Navigation::NAV_STOP_ON_POSITION:
            stop_on_position_handler_.handle(this);
            break;

        case Navigation::NAV_STOP_THERE:
            stop_there_handler_.handle(this);
            break;

        case Navigation::NAV_LANDING:
            landing_handler_.handle(this);
            break;
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
        local_position_t local_critical_pos;
        local_critical_pos.origin = position_estimation_.origin;

        critical_next_state_ = true;

        aero_attitude_t aero_attitude;
        aero_attitude = coord_conventions_quat_to_aero(navigation_.qe);
        local_critical_pos.heading = aero_attitude.rpy[2];

        switch (navigation_.critical_behavior)
        {
            case Navigation::CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Climbing to safe alt...\r\n");
                state_.mav_mode_custom |= CUST_CRITICAL_CLIMB_TO_SAFE_ALT;

                local_critical_pos.pos[X] = position_estimation_.local_position.pos[X];
                local_critical_pos.pos[Y] = position_estimation_.local_position.pos[Y];
                local_critical_pos.pos[Z] = -30.0f;

                break;

            case Navigation::FLY_TO_HOME_WP:
                state_.mav_mode_custom &= ~CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
                state_.mav_mode_custom |= CUST_CRITICAL_FLY_TO_HOME_WP;

                local_critical_pos.pos[X] = 0.0f;
                local_critical_pos.pos[Y] = 0.0f;
                local_critical_pos.pos[Z] = -30.0f;
                break;

            case Navigation::HOME_LAND:
                state_.mav_mode_custom &= ~CUST_CRITICAL_FLY_TO_HOME_WP;
                state_.mav_mode_custom |= CUST_CRITICAL_LAND;

                local_critical_pos.pos[X] = 0.0f;
                local_critical_pos.pos[Y] = 0.0f;
                local_critical_pos.pos[Z] = 5.0f;
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;

            case Navigation::CRITICAL_LAND:
                print_util_dbg_print("Critical land...\r\n");

                state_.mav_mode_custom &= static_cast<mav_mode_custom_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= CUST_CRITICAL_LAND;

                local_critical_pos.pos[X] = position_estimation_.local_position.pos[X];
                local_critical_pos.pos[Y] = position_estimation_.local_position.pos[Y];
                local_critical_pos.pos[Z] = 5.0f;
                navigation_.alt_lpf = position_estimation_.local_position.pos[2];
                break;
        }

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = local_critical_pos.pos[i] - position_estimation_.local_position.pos[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);

        waypoint_critical_coordinates_.set_local_pos(local_critical_pos);
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

bool Mission_planner::mode_change()
{
    return mav_modes_are_equal_autonomous_modes(state_.mav_mode(), last_mode_);
}

void Mission_planner::dubin_state_machine(Waypoint* next_waypoint)
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
                rel_pos[i] = next_waypoint->local_pos().pos[i]- position_estimation_.local_position.pos[i];
            }
            rel_pos[Z] = 0.0f;

            float dir_final[3];
            float pos_goal[3];
            float rel_pos_norm[3];

            if (vectors_norm_sqr(rel_pos) > 0.1f)
            {
                vectors_normalize(rel_pos,rel_pos_norm);

                dir_final[X] = -rel_pos_norm[Y]*next_waypoint->radius();
                dir_final[Y] =  rel_pos_norm[X]*next_waypoint->radius();
                dir_final[Z] = 0.0f;

                for (uint8_t i = 0; i < 2; ++i)
                {
                    pos_goal[i] = next_waypoint->local_pos().pos[i] + rel_pos_norm[i]*maths_f_abs(next_waypoint->radius());
                }
                pos_goal[Z] = 0.0f;

                next_waypoint->dubin() = dubin_2d(  position_estimation_.local_position.pos,
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
                    next_waypoint->dubin().circle_center_2[i] = position_estimation_.local_position.pos[i];
                }
                next_waypoint->dubin().circle_center_2[Z] = 0.0f;

                navigation_.dubin_state = DUBIN_CIRCLE2;
            }

            break;
        case DUBIN_CIRCLE1:
            for (uint8_t i = 0; i < 2; ++i)
            {
                rel_pos[i] = next_waypoint->dubin().tangent_point_2[i] - position_estimation_.local_position.pos[i];
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
                rel_pos[i] = next_waypoint->dubin().tangent_point_2[i] - position_estimation_.local_position.pos[i];
            }

            scalar_product = rel_pos[X] * next_waypoint->dubin().line_direction[X] + rel_pos[Y] * next_waypoint->dubin().line_direction[Y];
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
            hold_waypoint_set_(false),
            start_wpt_time_(time_keeper_get_ms()),
            waiting_at_waypoint_(false),
            travel_time_(0),
            critical_next_state_(false),
            last_mode_(state_.mav_mode()),
            mavlink_stream_(mavlink_stream_),
            state_(state_),
            navigation_(navigation_),
            position_estimation_(position_estimation_),
            ahrs_(ahrs_),
            manual_control_(manual_control_),
            config_(config)
{
    bool init_success = true;

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

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER] constructor: ERROR\r\n");
    }
}


bool Mission_planner::update(Mission_planner* mission_planner)
{
    mav_mode_t mode_local = mission_planner->state_.mav_mode();


    switch (mission_planner->state_.mav_state_)
    {
        case MAV_STATE_STANDBY:
            mission_planner->navigation_.internal_state_ = Navigation::NAV_ON_GND;
            mission_planner->hold_waypoint_set_ = false;
            mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            mission_planner->critical_next_state_ = false;
            mission_planner->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
            break;

        case MAV_STATE_ACTIVE:
            mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            mission_planner->critical_next_state_ = false;

            if (!mission_planner->state_.nav_plan_active)
            {
                mission_planner->waypoint_handler_.nav_plan_init();
            }

            mission_planner->state_machine();
            break;

        case MAV_STATE_CRITICAL:
            // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
            if (mav_modes_is_stabilize(mode_local))
            {
                if ((mission_planner->navigation_.internal_state_ == Navigation::NAV_NAVIGATING) || (mission_planner->navigation_.internal_state_ == Navigation::NAV_LANDING))
                {
                    mission_planner->critical_handler();

                    mission_planner->navigation_.goal = mission_planner->waypoint_critical_coordinates_;
                }
            }
            break;

        default:
            mission_planner->navigation_.internal_state_ = Navigation::NAV_ON_GND;
            break;
    }

    mission_planner->last_mode_ = mode_local;

    mission_planner->waypoint_handler_.control_time_out_waypoint_msg();

    return true;
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

void Mission_planner::set_hold_waypoint_set(bool hold_waypoint_set)
{
    hold_waypoint_set_ = hold_waypoint_set;
}

bool Mission_planner::hold_waypoint_set()
{
    return hold_waypoint_set_;
}

bool Mission_planner::waiting_at_waypoint() const
{
    return waiting_at_waypoint_;
}
