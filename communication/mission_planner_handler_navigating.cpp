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

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Mission_planner::waypoint_navigating_handler(bool reset_hold_wpt)
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


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_navigating::Mission_planner_handler_navigating( Position_estimation& position_estimation_,
                                                                        Navigation& navigation_,
                                                                        State& state_,
                                                                        const Mavlink_stream& mavlink_stream):
            position_estimation_(position_estimation_),
            state_(state_),
            navigation_(navigation_),
            mavlink_stream_(mavlink_stream)
{

}

Mission_planner_handler_navigating::handle(Mission_planner& mission_planner)
{
    mav_mode_t mode_local = state_.mav_mode();
    bool new_mode = true;

    if (!mav_modes_is_auto(last_mode_))
    {
        new_mode = mode_change();
    }
    waypoint_navigating_handler(new_mode);

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

}
