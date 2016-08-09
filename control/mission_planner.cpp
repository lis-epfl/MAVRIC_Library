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


#include "control/mission_planner.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

#include "control/mission_handler_takeoff.hpp"
#include "control/mission_handler_landing.hpp"
#include "control/mission_handler_on_ground.hpp"
#include "control/mission_handler_navigating.hpp"
#include "control/mission_handler_stop_there.hpp"
#include "control/mission_handler_stop_on_position.hpp"
#include "control/mission_handler_manual_control.hpp"
#include "control/mission_handler_hold_position.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.hpp"
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

// void Mission_planner::set_home(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg)
// {
//     mavlink_set_gps_global_origin_t packet;
//
//     if (!mission_planner->state_.is_armed())
//     {
//         mavlink_msg_set_gps_global_origin_decode(msg, &packet);
//
//         // Check if this message is for this system and subsystem
//         // Due to possible bug from QGroundControl, no check of target_component and compid
//         if ((uint8_t)packet.target_system == (uint8_t)sysid)
//         {
//             print_util_dbg_print("Set new home location.\r\n");
//             mission_planner->position_estimation_.local_position.origin.latitude = (double) packet.latitude / 10000000.0f;
//             mission_planner->position_estimation_.local_position.origin.longitude = (double) packet.longitude / 10000000.0f;
//             mission_planner->position_estimation_.local_position.origin.altitude = (float) packet.altitude / 1000.0f;
//
//             print_util_dbg_print("New Home location: (");
//             print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.latitude * 10000000.0f, 10);
//             print_util_dbg_print(", ");
//             print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.longitude * 10000000.0f, 10);
//             print_util_dbg_print(", ");
//             print_util_dbg_print_num(mission_planner->position_estimation_.local_position.origin.altitude * 1000.0f, 10);
//             print_util_dbg_print(")\r\n");
//
//
//             mission_planner->position_estimation_.set_new_fence_origin();
//
//             mavlink_message_t _msg;
//             mavlink_msg_gps_global_origin_pack(mission_planner->mavlink_stream_.sysid(),
//                                                mission_planner->mavlink_stream_.compid(),
//                                                &_msg,
//                                                mission_planner->position_estimation_.local_position.origin.latitude * 10000000.0f,
//                                                mission_planner->position_estimation_.local_position.origin.longitude * 10000000.0f,
//                                                mission_planner->position_estimation_.local_position.origin.altitude * 1000.0f);
//             mission_planner->mavlink_stream_.send(&_msg);
//         }
//     }
// }

mav_result_t Mission_planner::continue_to_next_waypoint(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result;
    bool force_next = false;
    uint32_t time_from_start_wpt = time_keeper_get_ms() - mission_planner->navigation_.start_wpt_time();
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

        mission_planner->navigation_.set_start_wpt_time();
        mission_planner->navigation_.set_waiting_at_waypoint(false);
        mission_planner->state_.nav_plan_active = true;
        mission_planner->waypoint_handler_.advance_to_next_waypoint();
        mission_planner->navigation_.set_goal(mission_planner->waypoint_handler_.current_waypoint());

        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(mission_planner->mavlink_stream_.sysid(),
                                         mission_planner->mavlink_stream_.compid(),
                                         &msg,
                                         mission_planner->waypoint_handler_.current_waypoint_index());
        mission_planner->mavlink_stream_.send(&msg);      

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

void Mission_planner::set_current_waypoint(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_set_current_t packet;

    mavlink_msg_mission_set_current_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        if (mission_planner->waypoint_handler_.set_current_waypoint_index(packet.seq))
        {
            mavlink_message_t _msg;
            mavlink_msg_mission_current_pack(sysid,
                                             mission_planner->mavlink_stream_.compid(),
                                             &_msg,
                                             packet.seq);
            mission_planner->mavlink_stream_.send(&_msg);

            mission_planner->navigation_.set_start_wpt_time();
            mission_planner->state_.nav_plan_active = true;
            mission_planner->navigation_.set_waiting_at_waypoint(false);
            mission_planner->navigation_.set_goal(mission_planner->waypoint_handler_.current_waypoint());
        }
        else
        {
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(mission_planner->mavlink_stream_.sysid(),
                                         mission_planner->mavlink_stream_.compid(),
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         MAV_CMD_ACK_ERR_ACCESS_DENIED);
            mission_planner->mavlink_stream_.send(&_msg);
        }
    } //end of if this message is for this system and subsystem
}

mav_result_t Mission_planner::set_current_waypoint_from_parameter(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result;
    uint16_t new_current = 0;

    print_util_dbg_print("All MAVs: Return to first waypoint.\r\n");

    if ((mission_planner->waypoint_handler_.set_current_waypoint_index(new_current)))
    {
        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(mission_planner->mavlink_stream_.sysid(),
                                         mission_planner->mavlink_stream_.compid(),
                                         &msg,
                                         new_current);
        mission_planner->mavlink_stream_.send(&msg);

        mission_planner->navigation_.set_start_wpt_time();
        mission_planner->state_.nav_plan_active = true;
        mission_planner->navigation_.set_waiting_at_waypoint(false);
        mission_planner->navigation_.set_goal(mission_planner->waypoint_handler_.current_waypoint());

        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

mav_result_t Mission_planner::is_arrived(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (packet->param2 == 32)
    {
        if (mission_planner->navigation_.waiting_at_waypoint())
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
    // If is auto, look to the waypoints
    if (mav_mode.is_auto())
    {
        switch (navigation_.internal_state())
        {
        case STANDBY:
            on_ground_handler_.handle(*this);
            break;

        case PREMISSION:
            takeoff_handler_.handle(*this);
            break;

        case MISSION:
            if (current_mission_handler_ != NULL)
            {
                current_mission_handler_->handle(*this);
            }
            else // If the current mission handler is not set, hold position
            {
                set_internal_state(PAUSED);
                hold_position_handler_.handle(*this);
            }
            break;

        case POSTMISSION:
            landing_handler_.handle(*this);
            break;

        case PAUSED:
            hold_position_handler_.handle(*this);
            break;
        }
    }
    else if (mav_mode.control_mode == Mav_mode::POSITION_HOLD) // Do position hold
    {
        // TODO: Needs to be set
        hold_position_handler_.handle(*this);
    }
    else
    {

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
            ins_.is_healthy(INS::healthy_t::XYZ_REL_POSITION) == false)
    {
        if (navigation_.critical_behavior != Navigation::CRITICAL_LAND)
        {
            navigation_.critical_behavior = Navigation::CRITICAL_LAND;
            critical_next_state_ = false;
        }
    }

    if (!critical_next_state_)
    {
        critical_next_state_ = true;

        local_position_t local_critical_pos;

        waypoint_critical_coordinates_.set_heading(coord_conventions_get_yaw(ahrs_.qe));

        switch (navigation_.critical_behavior)
        {
            case Navigation::CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Climbing to safe alt...\r\n");
                state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;

                local_critical_pos[X] = ins_.position_lf()[X];
                local_critical_pos[Y] = ins_.position_lf()[Y];
                local_critical_pos[Z] = navigation_.safe_altitude;

                break;

            case Navigation::FLY_TO_HOME_WP:
                state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
                state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;

                local_critical_pos[X] = 0.0f;
                local_critical_pos[Y] = 0.0f;
                local_critical_pos[Z] = navigation_.safe_altitude;
                break;

            case Navigation::HOME_LAND:
                state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;
                state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

                local_critical_pos[X] = 0.0f;
                local_critical_pos[Y] = 0.0f;
                local_critical_pos[Z] = navigation_.critical_landing_altitude;
                navigation_.alt_lpf = ins_.position_lf()[Z];
                break;

            case Navigation::CRITICAL_LAND:
                print_util_dbg_print("Critical land...\r\n");
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

                local_critical_pos[X] = ins_.position_lf()[X];
                local_critical_pos[Y] = ins_.position_lf()[Y];
                local_critical_pos[Z] = navigation_.critical_landing_altitude;
                navigation_.alt_lpf = ins_.position_lf()[Z];
                break;
        }

        waypoint_critical_coordinates_.set_local_pos(local_critical_pos);

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_critical_coordinates_.local_pos()[i] - ins_.position_lf()[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    if (navigation_.critical_behavior == Navigation::CRITICAL_LAND || navigation_.critical_behavior == Navigation::HOME_LAND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * navigation_.alt_lpf + (1.0f - navigation_.LPF_gain) * ins_.position_lf()[Z];
        if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - navigation_.alt_lpf) <= 0.2f))
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
                    //waypoint_hold_coordinates = waypoint_critical_coordinates_; TODO
                    navigation_.set_internal_state(Navigation::NAV_STOP_ON_POSITION);
                    stop_there_handler_.stopping_handler(*this);
                    state_.out_of_fence_1 = false;
                    navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
                    state_.mav_state_ = MAV_STATE_ACTIVE;
                    state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;
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
                navigation_.set_internal_state(Navigation::NAV_ON_GND);
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_EMERGENCY;
                break;
        }
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner::Mission_planner(INS& ins, Navigation& navigation, const ahrs_t& ahrs, State& state, const Manual_control& manual_control, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, Mission_handler_on_ground& on_ground_handler, Mission_handler_takeoff& takeoff_handler, Mission_handler_landing& landing_handler, Mission_handler_hold_position& hold_position_handler, Mavlink_waypoint_handler& waypoint_handler, conf_t config):
            on_ground_handler_(on_ground_handler),
            takeoff_handler_(takeoff_handler),
            landing_handler_(landing_handler),
            hold_position_handler_(hold_position_handler),
            registered_mission_handler_count(0),
            waypoint_handler_(waypoint_handler),
            critical_next_state_(false),
            mavlink_stream_(mavlink_stream),
            state_(state),
            navigation_(navigation),
            ins_(ins),
            ahrs_(ahrs),
            manual_control_(manual_control),
            message_handler_(message_handler),
            config_(config)
{
    last_mode_ = state_.mav_mode();
}

bool Mission_planner::init()
{
    bool init_success = true;

    // Create blank critical waypoint
    dubin_t dub;
    for (int i = 0; i < 3; i++)
    {
        dub.circle_center_1[i] = 0.0f;
        dub.tangent_point_1[i] = 0.0f;
        dub.circle_center_2[i] = 0.0f;
        dub.tangent_point_2[i] = 0.0f;
        dub.line_direction[i] = 0.0f;
    }
    dub.sense_1 = 0;
    dub.radius_1 = 0;
    dub.length = 0.0f;
    waypoint_critical_coordinates_ = Waypoint(MAV_FRAME_LOCAL_NED, MAV_CMD_NAV_LOITER_UNLIM, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, navigation_.safe_altitude);

    // Add callbacks for waypoint handler messages requests
    Mavlink_message_handler::msg_callback_t callback;
//
//     callback.message_id     = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
//     callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
//     callback.compid_filter  = MAV_COMP_ID_ALL;
//     callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &set_home;
//     callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
//     init_success &= message_handler_.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_SET_CURRENT; // 41
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &set_current_waypoint;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &continue_to_next_waypoint;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_CONDITION_DISTANCE; // 114
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &is_arrived;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // 20
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_current_waypoint_from_parameter;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER] constructor: ERROR\r\n");
    }

    return init_success;
}

bool Mission_planner::register_mission_handler(Mission_handler& handler)
{
    // Check for duplicates
    for (uint8_t i = 0; i < registered_mission_handler_count_; i++)
    {
        if (&registered_mission_handlers_[i] == &handler)
        {
            return false;
        }
    }

    // Register mission
    registered_mission_handlers_[registered_mission_handler_count_] = handler;
    registered_mission_handler_count_++;
    return true;
}

bool Mission_planner::switch_mission_handler(Waypoint& waypoint)
{
    // Search for the first handler than can accept this waypoint
    for (uint8_t i = 0; i < registered_mission_handler_count_; i++)
    {
        // Check if proper handler
        if (registered_mission_handlers_[i].can_handle(*this, waypoint))
        {
            // Set current handler
            current_mission_handler_ = &registered_mission_handlers_[i];
            current_mission_handler_->setup(*this, waypoint);
            return true;
        }
    }

    return false;
}

bool Mission_planner::update(Mission_planner* mission_planner)
{
    Mav_mode mode_local = mission_planner->state_.mav_mode();


    switch (mission_planner->state_.mav_state_)
    {
        case MAV_STATE_STANDBY:
            mission_planner->navigation_.set_internal_state(Navigation::NAV_ON_GND);
            Mission_handler::reset_hold_waypoint();
            mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            mission_planner->critical_next_state_ = false;
            mission_planner->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
            break;

        case MAV_STATE_ACTIVE:
            mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
            mission_planner->critical_next_state_ = false;

            if (!mission_planner->state_.nav_plan_active && !mission_planner->navigation_.waiting_at_waypoint())
            {
                mission_planner->state_.nav_plan_active = true;
            }

            mission_planner->state_machine();
            break;

        case MAV_STATE_CRITICAL:
            // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
            if (mode_local.is_guided())
            {
                if ((mission_planner->navigation_.internal_state() == Navigation::NAV_NAVIGATING) || (mission_planner->navigation_.internal_state() == Navigation::NAV_LANDING))
                {
                    mission_planner->critical_handler();

                    mission_planner->navigation_.set_goal(mission_planner->waypoint_critical_coordinates_);
                }
            }
            break;

        default:
            mission_planner->navigation_.set_internal_state(Navigation::NAV_ON_GND);
            break;
    }

    mission_planner->waypoint_handler_.control_time_out_waypoint_msg();

    return true;
}

void Mission_planner::set_critical_next_state(bool critical_next_state)
{
    critical_next_state_ = critical_next_state;
}

Mission_planner::internal_state_t Mission_planner::internal_state() const
{
    return internal_state_;
}

bool Mission_planner::set_internal_state(internal_state_t new_internal_state)
{
    bool ret = false;

    // Check if there has been a state change
    if (internal_state_ != new_internal_state)
    {
        // Reset handler
        switch (new_internal_state)
        {
        case NAV_STANDBY: // On ground
            // TODO: Actually check if we are on the ground
            internal_state_waypoint_ = Waypoint();
            ret &= on_ground_handler_.can_handle(*this, internal_state_waypoint_);
            ret &= on_ground_handler_.setup(*this, internal_state_waypoint_);
            break;

        case NAV_PREMISSION: // Landing
            internal_state_waypoint_ = Waypoint(    MAV_CMD_NAV_TAKEOFF,
                                                    MAV_CMD_NAV_TAKEOFF_LOCAL,
                                                    0,
                                                    0.0f,
                                                    0.0f,
                                                    0.0f,
                                                    0.0f,
                                                    INS::position_lf()[X],
                                                    INS::position_lf()[Y],
                                                    navigation_.takeoff_altitude);
            ret &= takeoff_handler_.can_handle(*this, internal_state_waypoint_);
            ret &= takeoff_handler_.setup(*this, internal_state_waypoint_);
            break;

        case NAV_MISSION: // Navigating
            // If changing to mission mode, set to current waypoint
            ret &= switch_mission_handler(waypoint_handler_.current_waypoint());
            break;

        case NAV_POSTMISSION: // Landing
            internal_state_waypoint_ = Waypoint(    MAV_FRAME_LOCAL_NED,
                                                    MAV_CMD_NAV_TAKEOFF_LOCAL,
                                                    0,
                                                    10.0f,
                                                    0.0f,
                                                    0.0f,
                                                    0.0f,
                                                    INS::position_lf()[X],
                                                    INS::position_lf()[Y],
                                                    INS::position_lf()[Z]);
            ret &= landing_handler_.can_handle(*this, internal_state_waypoint_);
            ret &= landing_handler_.setup(*this, internal_state_waypoint_);
            break;

        case NAV_PAUSED: // Hold Position
            internal_state_waypoint_ = Waypoint(    MAV_FRAME_LOCAL_NED,
                                                    MAV_CMD_NAV_LOITER_UNLIM,
                                                    0,
                                                    10.0f,
                                                    0.0f,
                                                    0.0f,
                                                    0.0f,
                                                    INS::position_lf()[X],
                                                    INS::position_lf()[Y],
                                                    INS::position_lf()[Z]);
            ret &= hold_position_handler_.can_handle(*this, internal_state_waypoint_);
            ret &= hold_position_handler_.setup(*this, internal_state_waypoint_);
            break;
        }
    }

    if (ret)
    {
        // Print state change
        print_util_dbg_print("Switching from ");
        switch (internal_state_)
        {
        case NAV_STANDBY:
            print_util_dbg_print("NAV_STANDBY");
            break;
        case NAV_PREMISSION:
            print_util_dbg_print("NAV_PREMISSION");
            break;
        case NAV_MISSION:
            print_util_dbg_print("NAV_MISSION");
            break;
        case NAV_POSTMISSION:
            print_util_dbg_print("NAV_POSTMISSION");
            break;
        case NAV_PAUSED:
            print_util_dbg_print("NAV_PAUSED");
            break;
        }
        print_util_dbg_print(" to ");
        switch (new_internal_state)
        {
        case NAV_STANDBY:
            print_util_dbg_print("NAV_STANDBY");
            break;
        case NAV_PREMISSION:
            print_util_dbg_print("NAV_PREMISSION");
            break;
        case NAV_MISSION:
            print_util_dbg_print("NAV_MISSION");
            break;
        case NAV_POSTMISSION:
            print_util_dbg_print("NAV_POSTMISSION");
            break;
        case NAV_PAUSED:
            print_util_dbg_print("NAV_PAUSED");
            break;
        }
        print_util_dbg_print("\r\n");

        internal_state_ = new_internal_state;
    }

    return ret;
}