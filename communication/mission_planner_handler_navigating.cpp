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
 * \file mission_planner_handler_navigating.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_navigating.hpp"

#include "communication/mavlink_waypoint_handler.hpp"
#include "communication/mission_planner_handler_landing.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Mission_planner_handler_navigating::waypoint_navigating_handler(Mission_planner& mission_planner, bool reset_hold_wpt, Waypoint& waypoint_coordinates)
{
    // Set current waypoint to the one we are travelling to
    waypoint_coordinates = waypoint_handler_.current_waypoint();

    if (!reset_hold_wpt)
    {
        if (state_.nav_plan_active)
        {
            navigation_.dubin_state = DUBIN_INIT;
        }
        mission_planner.set_hold_waypoint_set(false);
    }

    if (state_.nav_plan_active)
    {
        float rel_pos[3];
        uint16_t i;

        // Determine distance to the waypoint
        for (i = 0; i < 3; i++)
        {
            rel_pos[i] = waypoint_coordinates.local_pos().pos[i] - position_estimation_.local_position.pos[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);

        // Add margin if necessary
        float margin = 0.0f;
        if ((waypoint_handler_.current_waypoint().command() == MAV_CMD_NAV_LAND) || (waypoint_handler_.current_waypoint().param2() == 0.0f))
        //we need to add that since Landing waypoint doesn't have the param2
        //=> the param2 = 0 => never passing next condition
        {
            margin = 16.0f;
        }

        // If we are near the waypoint or are doing dubin circles
        if (navigation_.dist2wp_sqr < (waypoint_handler_.current_waypoint().param2() * waypoint_handler_.current_waypoint().param2() + margin) ||
               (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN && navigation_.dubin_state == DUBIN_CIRCLE2))
        {
            // If we are near the waypoint but the flag has not been set, do this once ...
            if (!mission_planner_.waiting_at_waypoint())
            {
                // Send debug log ...
                print_util_dbg_print("Waypoint Nr");
                print_util_dbg_print_num(waypoint_handler_.current_waypoint_index(), 10);
                print_util_dbg_print(" reached, distance:");
                print_util_dbg_print_num(sqrt(navigation_.dist2wp_sqr), 10);
                print_util_dbg_print(" less than :");
                print_util_dbg_print_num(SQR(waypoint_handler_.current_waypoint().param2()), 10);
                print_util_dbg_print(".\r\n");

                // ... as well as a mavlink message ...
                mavlink_message_t msg;
                mavlink_msg_mission_item_reached_pack(mavlink_stream_.sysid(),
                                                      mavlink_stream_.compid(),
                                                      &msg,
                                                      waypoint_handler_.current_waypoint_index());
                mavlink_stream_.send(&msg);

                // ... and record the travel time ...
                travel_time_ = time_keeper_get_ms() - mission_planner.start_wpt_time();

                // ... and set to waiting at waypoint
                mission_planner_.set_waiting_at_waypoint(true);
            }

            // If we are supposed to land, then land
            if (waypoint_handler_.current_waypoint().command() == MAV_CMD_NAV_LAND)
            {
                print_util_dbg_print("Stop & land\r\n");

                //auto landing is not using the packet,
                //so we can declare a dummy one.
                mavlink_command_long_t dummy_packet;
                dummy_packet.param1 = 1;
                dummy_packet.param5 = waypoint_coordinates.local_pos().pos[X];
                dummy_packet.param6 = waypoint_coordinates.local_pos().pos[Y];
                dummy_packet.param7 = waypoint_coordinates.local_pos().pos[Z];
                Mission_planner_handler_landing::set_auto_landing(&mission_planner_handler_landing_, &dummy_packet);
            }

            // If we can autocontinue...
            if ((waypoint_handler_.current_waypoint().autocontinue() == 1) && (waypoint_handler_.waypoint_count() > 1))
            {
                // Get references for calculations
                //Waypoint& current = mavlink_waypoint_handler_.current_waypoint();
                //Waypoint& next = mavlink_waypoint_handler_.next_waypoint();

                // Set new waypoint to advance
                if (mission_planner_.waiting_at_waypoint())
                {
                    waypoint_handler_.advance_to_next_waypoint();
                    dubin_state_t dubin_state;
                    waypoint_handler_.update_current_waypoint(position_estimation_.local_position.origin, &dubin_state);
                    mission_planner_.set_waiting_at_waypoint(false);

                    // Update output to be the new waypoint
                    waypoint_coordinates = waypoint_handler_.current_waypoint();

                    // Temporary
                    print_util_dbg_print("Autocontinue towards waypoint Nr");
                    print_util_dbg_print_num(waypoint_handler_.current_waypoint_index(),10);
                    print_util_dbg_print("\r\n");
                    navigation_.dubin_state = DUBIN_INIT;
                    mission_planner.set_start_wpt_time();

                    // Send message
                    mavlink_message_t msg;
                    mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                     mavlink_stream_.compid(),
                                                     &msg,
                                                     waypoint_handler_.current_waypoint_index());
                    mavlink_stream_.send(&msg);
                    // End temporary

                }
                /*
                // Determine geometry information between waypoints
                for (i=0;i<3;i++)
                {
                    rel_pos[i] = next.local_pos().pos[i] - waypoint_coordinates_.waypoint.pos[i];
                }

                float rel_pos_norm[3];
                vectors_normalize(rel_pos, rel_pos_norm);

                float outter_pt[3];
                outter_pt[X] = next.local_pos().pos[X] + rel_pos_norm[Y]*next.radius();
                outter_pt[Y] = next.local_pos().pos[Y] - rel_pos_norm[X]*next.radius();
                outter_pt[Z] = 0.0f;

                for (i=0;i<3;i++)
                {
                    rel_pos[i] = outter_pt[i]-position_estimation_.local_position.pos[i];
                }

                // float rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - position_estimation_.local_position.heading);
                float rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - atan2(position_estimation_.vel[Y], position_estimation_.vel[X]));

                if ( (maths_f_abs(rel_heading) < navigation_.heading_acceptance) ||
                    (current_waypoint_.command == MAV_CMD_NAV_LAND) ||
                    (navigation_.navigation_strategy == Navigation::strategy_t::DIRECT_TO) ||
                    (current.param2() == 0.0f) )
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

                    // Send message
                    mavlink_message_t msg;
                    mavlink_msg_mission_current_pack(mavlink_stream_.sysid(),
                                                     mavlink_stream_.compid(),
                                                     &msg,
                                                     current_waypoint_index_);
                    mavlink_stream_.send(&msg);
                }*/
            }
            else // If no autocontinue ...
            {
                // ... turn off nav plan
                state_.nav_plan_active = false;
                print_util_dbg_print("Stop\r\n");
            }
        }
    }
    else    // Nav plan is not active, set hold waypoint if necessary
    {
        if (!mission_planner_.hold_waypoint_set())
        {
            mission_planner_.hold_init(position_estimation_.local_position);
            waypoint_coordinates = mission_planner_.waypoint_hold_coordinates;
            mission_planner_.set_hold_waypoint_set(true);
        }
    }
}

mav_result_t Mission_planner_handler_navigating::start_stop_navigation(Mission_planner_handler_navigating* navigating_handler, mavlink_command_long_t* packet)
{
    mav_result_t result = MAV_RESULT_UNSUPPORTED;

    if (packet->param1 == MAV_GOTO_DO_HOLD)
    {
        if (packet->param2 == MAV_GOTO_HOLD_AT_CURRENT_POSITION)
        {
            navigating_handler->mission_planner_.hold_init(navigating_handler->position_estimation_.local_position);

            navigating_handler->navigation_.internal_state_ = Navigation::NAV_STOP_ON_POSITION;

            result = MAV_RESULT_ACCEPTED;
        }
        else if (packet->param2 == MAV_GOTO_HOLD_AT_SPECIFIED_POSITION)
        {
            navigating_handler->navigation_.internal_state_ = Navigation::NAV_STOP_THERE;

            /*
            waypoint.frame = packet->param3;
            waypoint.param4 = packet->param4;
            waypoint.x = packet->param5;
            waypoint.y = packet->param6;
            waypoint.z = packet->param7;
            */
            Waypoint waypoint(&navigating_handler->mavlink_stream_, (MAV_FRAME)packet->param3, MAV_CMD_NAV_WAYPOINT, 0, packet->param1, packet->param2, packet->param3, packet->param4, packet->param5, packet->param6, packet->param7);

            waypoint.calculate_waypoint_local_structure(navigating_handler->position_estimation_.local_position.origin, &navigating_handler->navigation_.dubin_state);

            navigating_handler->mission_planner_.hold_init(waypoint.local_pos());

            result = MAV_RESULT_ACCEPTED;
        }
    }
    else if (packet->param1 == MAV_GOTO_DO_CONTINUE)
    {
        if ( (navigating_handler->navigation_.internal_state_ == Navigation::NAV_STOP_THERE) || (navigating_handler->navigation_.internal_state_ == Navigation::NAV_STOP_ON_POSITION) )
        {
            navigating_handler->navigation_.dubin_state = DUBIN_INIT;
        }

        if (navigating_handler->mission_planner_.last_mode().is_auto())  // WHY USE LAST_MODE RATHER THAN STATE->MODE?
        {
            navigating_handler->navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
        }
        else if (navigating_handler->mission_planner_.last_mode().ctrl_mode() == Mav_mode::POSITION_HOLD)  // WHY USE LAST_MODE RATHER THAN STATE->MODE?
        {
            navigating_handler->navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
        }

        result = MAV_RESULT_ACCEPTED;
    }

    return result;
}


void Mission_planner_handler_navigating::send_nav_time(const Mavlink_stream* mavlink_stream_, mavlink_message_t* msg)
{
    mavlink_msg_named_value_int_pack(mavlink_stream_->sysid(),
                                     mavlink_stream_->compid(),
                                     msg,
                                     time_keeper_get_ms(),
                                     "travel_time_",
                                     travel_time_);
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_navigating::Mission_planner_handler_navigating( Position_estimation& position_estimation,
                                                                        Navigation& navigation,
                                                                        State& state,
                                                                        Mission_planner& mission_planner,
                                                                        const Mavlink_stream& mavlink_stream,
                                                                        Mavlink_waypoint_handler& waypoint_handler,
                                                                        Mission_planner_handler_landing& mission_planner_handler_landing,
                                                                        Mavlink_message_handler& message_handler):
            position_estimation_(position_estimation),
            navigation_(navigation),
            state_(state),
            mission_planner_(mission_planner),
            mavlink_stream_(mavlink_stream),
            waypoint_handler_(waypoint_handler),
            mission_planner_handler_landing_(mission_planner_handler_landing),
            travel_time_(0)
{
    bool init_success = true;

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_OVERRIDE_GOTO; // 252
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &start_stop_navigation;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler.add_cmd_callback(&callbackcmd);

    if(!init_success)
    {
        print_util_dbg_print("[MISSION_PLANNER_HANDLER_NAVIGATING] constructor: ERROR\r\n");
    }
}

void Mission_planner_handler_navigating::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();
    bool new_mode = true;

    if (!mission_planner.last_mode().is_auto())
    {
        new_mode = mission_planner.mode_change();
    }
    Waypoint waypoint_coordinates;
    waypoint_navigating_handler(mission_planner, new_mode, waypoint_coordinates);

    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
    {
        mission_planner.dubin_state_machine(&waypoint_coordinates);
    }

    navigation_.goal = waypoint_coordinates;

    if (!mode_local.is_auto())
    {
        if (mode_local.is_manual())
        {
            print_util_dbg_print("Switching to NAV_MANUAL_CTRL from NAV_NAVIGATING\r\n");
            navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
        }
        else
        {
            print_util_dbg_print("Switching to NAV_HOLD_POSITION from NAV_NAVIGATING\r\n");
            mission_planner.hold_init(position_estimation_.local_position);
            navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
        }
    }
}
