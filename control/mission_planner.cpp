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

    if ((mission_planner->waypoint_handler_.waypoint_count() > 0) && (mission_planner->navigation_.waiting_at_waypoint() || force_next))
    {
        print_util_dbg_print("All vehicles: Navigating to next waypoint.\r\n");

        mission_planner->navigation_.set_start_wpt_time();
        mission_planner->navigation_.set_waiting_at_waypoint(false);
        mission_planner->waypoint_handler_.advance_to_next_waypoint();
        mission_planner->switch_mission_handler(mission_planner->waypoint_handler_.current_waypoint());

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
            mission_planner->navigation_.set_waiting_at_waypoint(false);
            mission_planner->switch_mission_handler(mission_planner->waypoint_handler_.current_waypoint());
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
        mission_planner->navigation_.set_waiting_at_waypoint(false);
        mission_planner->switch_mission_handler(mission_planner->waypoint_handler_.current_waypoint());

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

mav_result_t Mission_planner::start_stop_navigation(Mission_planner* mission_planner, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    if (packet->param1 == MAV_GOTO_DO_HOLD)
    {
        if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
        {
            // This is an issue, waypoint will be destroyed before use TODO
            Waypoint hold_wpt = Waypoint(   packet->param3,
                                            MAV_CMD_OVERRIDE_GOTO,
                                            0,
                                            packet->param1,
                                            packet->param2,
                                            packet->param3,
                                            packet->param4,
                                            mission_planner->ins_.position_lf()[X],
                                            mission_planner->ins_.position_lf()[Y],
                                            mission_planner->ins_.position_lf()[Z]);
            mission_planner->insert_mission_waypoint(hold_wpt);

            result = MAV_RESULT_ACCEPTED;
        }
        else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
        {
            // This is an issue, waypoint will be destroyed before use TODO
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
            mission_planner->insert_mission_waypoint(hold_wpt);

            result = MAV_RESULT_ACCEPTED;
        }
    }
    else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
    {
        if (mission_planner->state_.is_auto())
        {
            mission_planner->waypoint_handler_.advance_to_next_waypoint();
            mission_planner->switch_mission_handler(mission_planner->waypoint_handler_.current_waypoint());
        }

        result = MAV_RESULT_ACCEPTED;
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
                                packet->param5,
                                packet->param6,
                                mission_planner->navigation_.takeoff_altitude); // packet->param7);

        print_util_dbg_print("Starting automatic take-off from button\r\n");
        mission_planner->insert_mission_waypoint(takeoff_wpt);

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

        // Change states
        mission_planner->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;

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
                                1,
                                0.0f,
                                0.0f,
                                0.0f,
                                heading,
                                landing_position[X],
                                landing_position[Y],
                                landing_position[Z]);
        mission_planner->insert_mission_waypoint(landing_wpt);

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
    // If is auto, look to the waypoints
    if (state_.mav_mode().is_auto())
    {
        if (current_mission_handler_ != NULL)
        {
            // Handle current mission
            current_mission_handler_->handle(*this);

            // Check if we should be switch states
            // This is for automatic advancement (states can also advance from callback
            // functions from GCS or remote)
            if (current_mission_handler_->is_finished(*this))
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
        else    // If the current handler is NULL, set to paused
        {
            //set_internal_state(PAUSED);
        }
    }
    else if (state_.mav_mode().ctrl_mode() == Mav_mode::POSITION_HOLD) // Do position hold
    {
        // TODO: Needs to be set
        // TODO set to default
        if (current_mission_handler_ != NULL)
        {
            current_mission_handler_->handle(*this);
        }
        // DONT CHECK IF FINISHED POSITION HOLD
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
    // If one of these happens, we need to land RIGHT NOW
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

    // Set new critical waypoint / mission handler if needed
    if (!critical_next_state_)
    {
        critical_next_state_ = true;

        switch (navigation_.critical_behavior)
        {
        case Navigation::CLIMB_TO_SAFE_ALT:
            print_util_dbg_print("Climbing to safe alt...\r\n");
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_WAYPOINT,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.safe_altitude);

            break;

        case Navigation::FLY_TO_HOME_WP:
            state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_CLIMB_TO_SAFE_ALT;
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_WAYPOINT,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            navigation_.safe_altitude);
            break;

        case Navigation::HOME_LAND:
            state_.mav_mode_custom &= ~Mav_mode::CUST_CRITICAL_FLY_TO_HOME_WP;
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_LAND,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            navigation_.critical_landing_altitude);
            navigation_.alt_lpf = ins_.position_lf()[Z];
            break;

        case Navigation::CRITICAL_LAND:
            print_util_dbg_print("Critical land...\r\n");
            state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
            state_.mav_mode_custom |= Mav_mode::CUST_CRITICAL_LAND;

            critical_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                            MAV_CMD_NAV_LAND,
                                            1,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            ins_.position_lf()[X],
                                            ins_.position_lf()[Y],
                                            navigation_.critical_landing_altitude);
            navigation_.alt_lpf = ins_.position_lf()[Z];
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
        current_mission_handler_->handle(*this);
    }

    // Create new check for moving to next critical state
    // This also allows us to bypass any handlers that do not have autocontinue checks enables
    if (navigation_.critical_behavior == Navigation::CRITICAL_LAND || navigation_.critical_behavior == Navigation::HOME_LAND)
    {
        navigation_.alt_lpf = navigation_.LPF_gain * navigation_.alt_lpf + (1.0f - navigation_.LPF_gain) * ins_.position_lf()[Z];
        if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - navigation_.alt_lpf) <= 0.2f))
        {
            // Disarming
            next_state_ = true;
        }
    }
    else if ((navigation_.critical_behavior == Navigation::CLIMB_TO_SAFE_ALT) || (navigation_.critical_behavior == Navigation::FLY_TO_HOME_WP))
    {
        if (navigation_.dist2wp_sqr < 3.0f)
        {
            next_state_ = true;
        }
    }

    // If we move to the next state, set the next state
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
            critical_waypoint_ = Waypoint();
            switch_mission_handler(critical_waypoint_);
            state_.set_armed(false);
            state_.mav_state_ = MAV_STATE_EMERGENCY;
            break;
        }
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
                                    1,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    0.0f,
                                    navigation_.critical_landing_altitude);

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

    callbackcmd.command_id = MAV_CMD_NAV_RETURN_TO_LAUNCH; // 20
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_current_waypoint_from_parameter;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

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

    callbackcmd.command_id = MAV_CMD_CONDITION_DISTANCE; // 114
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &is_arrived;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &start_stop_navigation;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    callbackcmd.command_id = MAV_CMD_MISSION_START; // 300
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_MISSIONPLANNER; // 190
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &continue_to_next_waypoint;
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

    switch (mission_planner->state_.mav_state_)
    {
    case MAV_STATE_STANDBY:
        if (mission_planner->internal_state() != STANDBY)
        {
            mission_planner->inserted_waypoint_ = Waypoint();
            // Switch, don't insert as the mission isn't paused
            mission_planner->switch_mission_handler(mission_planner->inserted_waypoint_);
        }
        mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
        mission_planner->critical_next_state_ = false;
        mission_planner->navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
        break;

    case MAV_STATE_ACTIVE:
        mission_planner->navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
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
        }
        print_util_dbg_print("\r\n");

        // Update internal state
        internal_state_ = new_internal_state;
    }
}

bool Mission_planner::switch_mission_handler(Waypoint& waypoint)
{
    Mission_handler* handler = mission_handler_registry_.get_mission_handler(waypoint);
    if (handler == NULL)
    {
        return false;
    }

    // Set current handler
    bool ret = handler->setup(*this, waypoint);
    if (ret)
    {
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

bool Mission_planner::insert_mission_waypoint(Waypoint wpt)
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