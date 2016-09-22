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


#include "mission/mission_planner.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

#include "mission/mission_handler.hpp"
#include "mission/mission_handler_takeoff.hpp"
#include "mission/mission_handler_landing.hpp"
#include "mission/mission_handler_on_ground.hpp"
#include "mission/mission_handler_navigating.hpp"
#include "mission/mission_handler_hold_position.hpp"
#include "util/print_util.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/maths.h"
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Mission_planner::set_current_waypoint(uint16_t index)
{
    bool success = false;
    if ((waypoint_handler_.set_current_waypoint_index(index)))
    {
        print_util_dbg_print("setting current wp\n");
        /* send new index to ground station */
        mavlink_message_t msg;
        mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                         mavlink_stream_.compid(),
                                         &msg,
                                         index);
        mavlink_stream_.send(&msg);

        if (state_.is_auto())
        {
            print_util_dbg_print("changing right now\n");
            navigation_.set_start_wpt_time();
            navigation_.set_waiting_at_waypoint(false);
            switch_mission_handler(waypoint_handler_.current_waypoint());
        }

        success = true;
    }
    return success;
}


mav_result_t Mission_planner::mission_start(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    print_util_dbg_print("[MISSION_PLANNER]: mission_start: ");
    bool success = false;
    if(mission_planner->state_.is_auto())
    {
        print_util_dbg_print("waypoint ");
        print_util_dbg_print_num(packet->param1,10);
        success = mission_planner->set_current_waypoint(packet->param1);    
    }
    print_util_dbg_print("\n");
    return success ? MAV_RESULT_ACCEPTED : MAV_RESULT_DENIED;
}


void Mission_planner::set_current_waypoint_from_parameter(Mission_planner* mission_planner, uint32_t sysid, mavlink_message_t* msg)
{
    print_util_dbg_print("set_current_waypoint_from_parameter:  seq");
    uint8_t new_current = mavlink_msg_mission_set_current_get_seq(msg);
    print_util_dbg_print_num(new_current,10);
    print_util_dbg_print("\n");
    mission_planner->set_current_waypoint(new_current);
}

mav_result_t Mission_planner::set_override_goto(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    if (!mission_planner->state_.is_auto())
    {
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    if (packet->param1 == MAV_GOTO_DO_HOLD)
    {
        if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
        {
            Waypoint hold_wpt = Waypoint(   MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_OVERRIDE_GOTO,
                                            0,
                                            packet->param1,
                                            packet->param2,
                                            MAV_FRAME_LOCAL_NED,
                                            packet->param4,
                                            mission_planner->ins_.position_lf()[X],
                                            mission_planner->ins_.position_lf()[Y],
                                            mission_planner->ins_.position_lf()[Z]);
            mission_planner->insert_ad_hoc_waypoint(hold_wpt);

            result = MAV_RESULT_ACCEPTED;
        }
        else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
        {
            Waypoint hold_wpt = Waypoint(   packet->param3,
                                            MAV_CMD_OVERRIDE_GOTO,
                                            0,
                                            packet->param1,
                                            packet->param2,
                                            packet->param3,
                                            packet->param4,
                                            packet->param5,
                                            packet->param6,
                                            packet->param7);
            mission_planner->insert_ad_hoc_waypoint(hold_wpt);

            result = MAV_RESULT_ACCEPTED;
        }
        else
        {
            result = MAV_RESULT_DENIED;
        }
    }
    else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
    {
        uint16_t new_wp_index = mission_planner->waypoint_handler_.current_waypoint_index();
        if(mission_planner->internal_state_ != PAUSED)
        {
            new_wp_index++;
        }
        mission_planner->set_current_waypoint(new_wp_index % mission_planner->waypoint_handler_.waypoint_count());
        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

mav_result_t Mission_planner::set_auto_takeoff(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result;

    if (mission_planner->internal_state() == Mission_planner::STANDBY)
    {
        Waypoint takeoff_wpt(   MAV_FRAME_LOCAL_NED,
                                MAV_CMD_NAV_TAKEOFF,
                                1,
                                0.0f,
                                0.0f,
                                0.0f,
                                packet->param4,
                                mission_planner->ins_.position_lf()[X],
                                mission_planner->ins_.position_lf()[Y],
                                mission_planner->navigation_.takeoff_altitude); // packet->param7);

        print_util_dbg_print("Starting automatic take-off from button\r\n");
        mission_planner->insert_ad_hoc_waypoint(takeoff_wpt);

        result = MAV_RESULT_ACCEPTED;
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

mav_result_t Mission_planner::set_auto_landing(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result;

    // Only land if we are in an appropriate navigation state already
    if (   (mission_planner->internal_state() == Mission_planner::MISSION)
        || (mission_planner->internal_state() == Mission_planner::PAUSED))
    {
        result = MAV_RESULT_ACCEPTED;

        // Determine landing location
        local_position_t landing_position = mission_planner->ins_.position_lf();
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
        float heading = coord_conventions_get_yaw(mission_planner->ahrs_.qe);
        Waypoint landing_wpt(   MAV_FRAME_LOCAL_NED,
                                MAV_CMD_NAV_LAND,
                                0,
                                0.0f,
                                0.0f,
                                0.0f,
                                heading,
                                landing_position[X],
                                landing_position[Y],
                                landing_position[Z]);
        mission_planner->insert_ad_hoc_waypoint(landing_wpt);

        print_util_dbg_print("Auto-landing procedure initialised.\r\n");
        print_util_dbg_print("Landing at: (");
        local_position_t local_pos = landing_wpt.local_pos();
        print_util_dbg_print_num(local_pos[X], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Y], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num(local_pos[Z], 10);
        print_util_dbg_print(", ");
        print_util_dbg_print_num((int32_t)(heading * 180.0f / 3.14f), 10);
        print_util_dbg_print(")\r\n");
    }
    else
    {
        result = MAV_RESULT_DENIED;
    }

    return result;
}

void Mission_planner::state_machine()
{
    // Reset hold position flag as we are not in hold position mode
    if (state_.mav_mode().ctrl_mode() != Mav_mode::POSITION_HOLD)
    {
        hold_position_set_ = false;
    }

    // Require auto-takeoff if we are in standby
    if (internal_state_ == STANDBY)
    {
        require_takeoff_ = true;
    }

    // If is auto, look to the waypoints
    if (state_.mav_mode().is_auto())
    {
        // Require takeoff if we have switched out of auto, dont take off if on ground
        if (require_takeoff_ &&
           (internal_state_ != STANDBY ||
                (internal_state_ == STANDBY && manual_control_.get_thrust() > -0.7f)))
        {
            inserted_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_TAKEOFF,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            coord_conventions_get_yaw(ahrs_.qe),
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.takeoff_altitude);
            insert_ad_hoc_waypoint(inserted_waypoint_);
            require_takeoff_ = false;
        }

        if (current_mission_handler_ != NULL)
        {
            // Handle current mission
            Mission_handler::update_status_t ret = current_mission_handler_->update();

            // Check if we should be switch states
            // This is for automatic advancement (states can also advance from callback
            // functions from GCS or remote)
            if (ret == Mission_handler::MISSION_FINISHED)
            {
                switch (internal_state())
                {
                case STANDBY:
                    // DONT ADVANCE IF FINISHED ON GROUND
                    break;

                case PREMISSION: // After takeoff, continue with next mission item
                    {
                        // Advance mission item
                        navigation_.set_start_wpt_time();
                        navigation_.set_waiting_at_waypoint(false);
                        waypoint_handler_.advance_to_next_waypoint();

                        // Set mission handler
                        switch_mission_handler(waypoint_handler_.current_waypoint());

                        print_util_dbg_print("Autocontinue from takeoff towards waypoint Nr");
                        print_util_dbg_print_num(waypoint_handler_.current_waypoint_index(),10);
                        print_util_dbg_print("\r\n");

                        // Send message
                        mavlink_message_t msg;
                        mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                         mavlink_stream_.compid(),
                                                         &msg,
                                                         waypoint_handler_.current_waypoint_index());
                        mavlink_stream_.send(&msg);
                        break;
                    }

                case MISSION: // After mission item, continue with next mission item
                    {
                        // Advance to next waypoint
                        navigation_.set_start_wpt_time();
                        navigation_.set_waiting_at_waypoint(false);
                        waypoint_handler_.advance_to_next_waypoint();

                        // Set mission handler
                        switch_mission_handler(waypoint_handler_.current_waypoint());

                        print_util_dbg_print("Autocontinue from mission towards waypoint Nr");
                        print_util_dbg_print_num(waypoint_handler_.current_waypoint_index(),10);
                        print_util_dbg_print("\r\n");

                        // Send message
                        mavlink_message_t msg;
                        mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                         mavlink_stream_.compid(),
                                                         &msg,
                                                         waypoint_handler_.current_waypoint_index());
                        mavlink_stream_.send(&msg);
                        break;
                    }

                case POSTMISSION: // After landing, set mission to standby
                    {
                        inserted_waypoint_ = Waypoint();
                        // Switch, don't insert as the mission is not paused
                        switch_mission_handler(inserted_waypoint_);

                        print_util_dbg_print("Autocontinue from landing to standby");
                        print_util_dbg_print("\r\n");
                        break;
                    }

                case PAUSED: // After paused, continue to current mission item (don't advance)
                case MANUAL_CTRL: // After manual control, continue to current mission item (don't advance)
                    {
                        // Reset but don't advance current waypoint
                        navigation_.set_start_wpt_time();
                        navigation_.set_waiting_at_waypoint(false);

                        // Set mission handler
                        switch_mission_handler(waypoint_handler_.current_waypoint());

                        print_util_dbg_print("Autocontinue towards waypoint Nr");
                        print_util_dbg_print_num(waypoint_handler_.current_waypoint_index(),10);
                        print_util_dbg_print("\r\n");

                        // Send message
                        mavlink_message_t msg;
                        mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                         mavlink_stream_.compid(),
                                                         &msg,
                                                         waypoint_handler_.current_waypoint_index());
                        mavlink_stream_.send(&msg);
                        break;
                    }
                }
            }
        }
    }
    else if (state_.mav_mode().ctrl_mode() == Mav_mode::POSITION_HOLD) // Do position hold
    {
        // Set hold position waypoint
        if (internal_state_ == STANDBY && manual_control_.get_thrust() > -0.7f  && !hold_position_set_) // On ground and desired to take off
        {
            inserted_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_TAKEOFF,
                                            0,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            coord_conventions_get_yaw(ahrs_.qe),
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.takeoff_altitude);
            insert_ad_hoc_waypoint(inserted_waypoint_);
            require_takeoff_ = false;
            hold_position_set_ = true;
        }
        else if (internal_state_ != STANDBY && !hold_position_set_) // In air and desired to hold
        {
            inserted_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_LOITER_UNLIM,
                                            0,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            coord_conventions_get_yaw(ahrs_.qe),
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            ins_.position_lf()[Z]);
            // Insert so we can say to go back to doing last mission item afterwards
            insert_ad_hoc_waypoint(inserted_waypoint_);
            hold_position_set_ = true;
        }

        if (current_mission_handler_ != NULL)
        {
            current_mission_handler_->update();
        }
        // DONT CHECK IF FINISHED POSITION HOLD
    }
    else // Not in auto or position hold (therefore we are in attitude or velocity)
    {
        require_takeoff_ = true;

        // Change only if thrust value is great enough
        if (manual_control_.get_thrust() > -0.7f && current_mission_handler_->handler_mission_state() != MANUAL_CTRL)
        {
            inserted_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_MANUAL_CTRL,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f);
            // Insert so we can say to go back to doing last mission item afterwards
            insert_ad_hoc_waypoint(inserted_waypoint_);
        }
    }
}


void Mission_planner::critical_handler()
{
    float rel_pos[3];

    //Check whether we entered critical mode due to a battery low level or a lost
    // connection with the GND station or are out of fence control
    // If one of these happens, we need to land RIGHT NOW
    if (state_.battery_.is_low() ||
        state_.connection_lost ||
        state_.out_of_fence_2 ||
        ins_.is_healthy(INS::healthy_t::XYZ_REL_POSITION) == false)
    {
        if (critical_behavior_ != CRITICAL_LAND)
        {
            critical_behavior_ = CRITICAL_LAND;
            critical_next_state_ = false;
        }
    }

    // Set new critical waypoint / mission handler if needed
    if (!critical_next_state_)
    {
        critical_next_state_ = true;

        switch (critical_behavior_)
        {
        case CLIMB_TO_SAFE_ALT:
            print_util_dbg_print("Climbing to safe alt...\r\n");
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_CRITICAL_WAYPOINT,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.safe_altitude);

            break;

        case FLY_TO_HOME_WP:
            state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_CRITICAL_WAYPOINT,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            navigation_.safe_altitude);
            break;

        case HOME_LAND:
            state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_CRITICAL_LAND,
                                            0,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            navigation_.critical_landing_altitude);
            break;

        case CRITICAL_LAND:
            print_util_dbg_print("Critical land...\r\n");
            state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_CRITICAL_LAND,
                                            0,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.critical_landing_altitude);
            break;
        }

        // Set this new waypoint
        switch_mission_handler(critical_waypoint_);

        for (uint8_t i = 0; i < 3; i++)
        {
            rel_pos[i] = critical_waypoint_.local_pos()[i] - ins_.position_lf()[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);
    }

    // Handle critical state
    if (current_mission_handler_ != NULL)
    {
        Mission_handler::update_status_t ret = current_mission_handler_->update();

        // If we move to the next state, set the next state
        if (ret == Mission_handler::MISSION_FINISHED)
        {
            critical_next_state_ = false;
            switch (critical_behavior_)
            {
            case CLIMB_TO_SAFE_ALT:
                print_util_dbg_print("Critical State! Flying to home waypoint.\r\n");
                critical_behavior_ = FLY_TO_HOME_WP;
                break;

            case FLY_TO_HOME_WP:
                if (state_.out_of_fence_1)
                {
                    state_.out_of_fence_1 = false;
                    critical_behavior_ = CLIMB_TO_SAFE_ALT;
                    state_.mav_state_ = MAV_STATE_ACTIVE;
                    state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;
                }
                else
                {
                    print_util_dbg_print("Critical State! Performing critical landing.\r\n");
                    critical_behavior_ = HOME_LAND;
                }
                break;

            case HOME_LAND:
            case CRITICAL_LAND:
                print_util_dbg_print("Critical State! Landed, switching off motors, Emergency mode.\r\n");
                critical_behavior_ = CLIMB_TO_SAFE_ALT;
                //state_.mav_mode_custom = CUSTOM_BASE_MODE;
                critical_waypoint_ = Waypoint();
                switch_mission_handler(critical_waypoint_);
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_EMERGENCY;
                break;
            }
        }
    }
}

void Mission_planner::set_internal_state(internal_state_t new_internal_state)
{
    if (internal_state_ != new_internal_state)
    {
        // Print state change
        print_util_dbg_print("Switching from ");
        switch (internal_state_)
        {
        case STANDBY:
            print_util_dbg_print("STANDBY");
            break;
        case PREMISSION:
            print_util_dbg_print("PREMISSION");
            break;
        case MISSION:
            print_util_dbg_print("MISSION");
            break;
        case POSTMISSION:
            print_util_dbg_print("POSTMISSION");
            break;
        case PAUSED:
            print_util_dbg_print("PAUSED");
            break;
        case MANUAL_CTRL:
            print_util_dbg_print("MANUAL_CTRL");
            break;
        }
        print_util_dbg_print(" to ");
        switch (new_internal_state)
        {
        case STANDBY:
            print_util_dbg_print("STANDBY");
            break;
        case PREMISSION:
            print_util_dbg_print("PREMISSION");
            break;
        case MISSION:
            print_util_dbg_print("MISSION");
            break;
        case POSTMISSION:
            print_util_dbg_print("POSTMISSION");
            break;
        case PAUSED:
            print_util_dbg_print("PAUSED");
            break;
        case MANUAL_CTRL:
            print_util_dbg_print("MANUAL_CTRL");
            break;
        }
        print_util_dbg_print("\r\n");

        // Update internal state
        internal_state_ = new_internal_state;
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner::Mission_planner(INS& ins, Navigation& navigation, const ahrs_t& ahrs, State& state, const Manual_control& manual_control, Mavlink_message_handler& message_handler, const Mavlink_stream& mavlink_stream, Mavlink_waypoint_handler& waypoint_handler, Mission_handler_registry& mission_handler_registry, conf_t config):
            waypoint_handler_(waypoint_handler),
            mission_handler_registry_(mission_handler_registry),
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
    critical_behavior_ = CLIMB_TO_SAFE_ALT;
}

bool Mission_planner::init()
{
    bool init_success = true;

    // Set initial standby mission handler
    inserted_waypoint_ = Waypoint();
    switch_mission_handler(inserted_waypoint_);

    // Create blank critical waypoint
    critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                    MAV_CMD_NAV_LAND,
                                    0,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    navigation_.critical_landing_altitude);

    // Manual hold position information
    require_takeoff_ = true;
    hold_position_set_ = false;

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
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &set_current_waypoint_from_parameter;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_NAV_LAND; // 21
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_landing;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_NAV_TAKEOFF; // 22
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_auto_takeoff;
    callbackcmd.module_struct =                                 this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_override_goto;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &mission_start;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER] constructor: ERROR\r\n");
    }

    return init_success;
}

bool Mission_planner::update(Mission_planner* mission_planner)
{
    Mav_mode mode_local = mission_planner->state_.mav_mode();

    // Reset to standby if disarmed
    if (!mission_planner->state_.is_armed() && mission_planner->internal_state() != STANDBY)
    {
        mission_planner->inserted_waypoint_ = Waypoint();
        mission_planner->switch_mission_handler(mission_planner->inserted_waypoint_);
    }

    switch (mission_planner->state_.mav_state_)
    {
    case MAV_STATE_STANDBY:
        if (mission_planner->internal_state() != STANDBY)
        {
            mission_planner->inserted_waypoint_ = Waypoint();
            // Switch, don't insert as the mission isn't paused
            mission_planner->switch_mission_handler(mission_planner->inserted_waypoint_);
        }
        mission_planner->critical_behavior_ = CLIMB_TO_SAFE_ALT;
        mission_planner->critical_next_state_ = false;
        break;

    case MAV_STATE_ACTIVE:
        mission_planner->critical_behavior_ = CLIMB_TO_SAFE_ALT;
        mission_planner->critical_next_state_ = false;

        mission_planner->state_machine();
        break;

    case MAV_STATE_CRITICAL:
        // In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
        if (mode_local.is_guided())
        {
            mission_planner->critical_handler();
        }
        break;

    default:
        if (mission_planner->internal_state() != STANDBY)
        {
            mission_planner->inserted_waypoint_ = Waypoint();
            // Switch, don't insert as the mission isn't paused
            mission_planner->switch_mission_handler(mission_planner->inserted_waypoint_);
        }
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

Mission_planner::critical_behavior_enum Mission_planner::critical_behavior() const
{
    return critical_behavior_;
}

bool Mission_planner::switch_mission_handler(const Waypoint& waypoint)
{
    Mission_handler* handler = mission_handler_registry_.get_mission_handler(waypoint);
    if (handler == NULL)
    {
        return false;
    }

    // Set current handler
    bool ret = handler->setup(waypoint);

    if (ret)
    {
        // Success, set mission state and current mission handler
        set_internal_state(handler->handler_mission_state());
        current_mission_handler_ = handler;

        // Output debug message
        print_util_dbg_print("Switching to waypoint command: ");
        print_util_dbg_print_num(waypoint.command(), 10);
        print_util_dbg_print("\r\n");
    }
    else
    {
        print_util_dbg_print("Cannot setup mission handler\r\n");
    }

    return ret;
}

bool Mission_planner::insert_ad_hoc_waypoint(Waypoint wpt)
{
    bool ret = true;

    // Copy waypoint state in case of failure
    Waypoint old = inserted_waypoint_;
    inserted_waypoint_ = wpt;

    ret &= switch_mission_handler(inserted_waypoint_);

    if (ret)
    {
        // Override what the mission handler set to PAUSED
        set_internal_state(PAUSED);
    }
    else
    {
        // Failed, revert internal state waypoint
        inserted_waypoint_ = old;
    }

    return ret;
}
