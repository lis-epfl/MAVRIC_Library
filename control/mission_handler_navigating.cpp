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
 * \file mission_handler_navigating.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the navigating state
 *
 ******************************************************************************/


#include "control/mission_handler_navigating.hpp"

#include "communication/mavlink_waypoint_handler.hpp"
#include "control/mission_handler_landing.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Mission_handler_navigating::send_nav_time(const Mavlink_stream* mavlink_stream_, mavlink_message_t* msg)
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

Mission_handler_navigating::Mission_handler_navigating( const INS& ins,
                                                                        Navigation& navigation,
                                                                        State& state,
                                                                        const Mavlink_stream& mavlink_stream,
                                                                        Mavlink_waypoint_handler& waypoint_handler,
                                                                        Mission_handler_landing& mission_handler_landing,
                                                                        Mavlink_message_handler& message_handler):
            Mission_handler(ins),
            navigation_(navigation),
            state_(state),
            mavlink_stream_(mavlink_stream),
            waypoint_handler_(waypoint_handler),
            mission_handler_landing_(mission_handler_landing),
            message_handler_(message_handler),
            travel_time_(0)
{

}

bool Mission_handler_navigating::can_handle(Mission_planner& mission_planner, Waypoint& wpt)
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_WAYPOINT)
    {
        handleable = true;
    }

    return handleable;
}

bool Mission_handler_navigating::setup(Mission_planner& mission_planner, Waypoint& wpt)
{
    bool success = true;

    start_time_ = time_keeper_get_ms();
    navigation_.set_waiting_at_waypoint(false);
    waypoint_ = wpt;

    return success;
}

void Mission_handler_navigating::handle(Mission_planner& mission_planner)
{
    // Set goal
    navigation_.set_goal(waypoint_.local_pos(), waypoint_.param4(), waypoint_.param2());

    /* Check for flag to set waiting at waypoint */
    // Find distance to waypoint
    local_position_t wpt_pos = waypoint_.local_pos();
    for (i = 0; i < 3; i++)
    {
        rel_pos[i] = wpt_pos[i] - ins_.position_lf()[i];
    }
    navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);

    // Add margin if necessary
    float margin = 0.0f;
    if (waypoint_.param2() == 0.0f) // Is this safe? TODO
    //we need to add that since Landing waypoint doesn't have the param2
    //=> the param2 = 0 => never passing next condition
    {
        margin = 16.0f;
    }

    // If we are near the waypoint or are doing dubin circles
    if (navigation_.dist2wp_sqr < (waypoint_.param2() * waypoint_.param2() + margin) ||
           (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN && navigation_.dubin_state == DUBIN_CIRCLE2))
    {
        // If we are near the waypoint but the flag has not been set, do this once ...
        if (!navigation_.waiting_at_waypoint())
        {
            // Send debug log ...
            if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN && navigation_.dubin_state == DUBIN_CIRCLE2)
            {
                print_util_dbg_print("Waypoint reached by entering dubin circles 2.\r\n");
            }
            else if (navigation_.dist2wp_sqr < (waypoint_.param2() * waypoint_.param2() + margin))
            {
                print_util_dbg_print("Waypoint reached, distance: ");
                print_util_dbg_putfloat(sqrt(navigation_.dist2wp_sqr), 3);
                print_util_dbg_print(" m. Less than acceptable radius:");
                print_util_dbg_putfloat(sqrt(waypoint_.param2() * waypoint_.param2() + margin), 3);
                print_util_dbg_print(" m.\r\n");
            }

            // ... as well as a mavlink message ...
            mavlink_message_t msg;
            mavlink_msg_mission_item_reached_pack(mavlink_stream_.sysid(),
                                                  mavlink_stream_.compid(),
                                                  &msg,
                                                  waypoint_handler_.current_waypoint_index());
            mavlink_stream_.send(&msg);

            // ... and record the travel time ...
            travel_time_ = time_keeper_get_ms() - navigation_.start_wpt_time();

            // ... and set to waiting at waypoint ...
            navigation_.set_waiting_at_waypoint(true);
        }
    }
}

bool Mission_handler_navigating::is_finished(Mission_planner& mission_planner)
{
    // First check if we have reached the waypoint
    if (navigation_.waiting_at_waypoint())
    {
        // Then ensure that we are in autocontinue
        if ((waypoint_.autocontinue() == 1) && (waypoint_handler_.waypoint_count() > 1))
        {
            // Get references for calculations
            Waypoint& current = waypoint_;

            // Differentiate between dubin and direct to
            if (navigation_.navigation_strategy == Navigation::strategy_t::DIRECT_TO || 
                current.param2() == 0.0f)
            {
                return true;
            }
            else if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
            {
                /* Check for correct heading */
                Waypoint& next = waypoint_handler_.next_waypoint();

                // Find the direction of the next waypoint
                for (i=0;i<3;i++)
                {
                    rel_pos[i] = next.local_pos()[i] - current.local_pos()[i];
                }
                float rel_pos_norm[3];
                vectors_normalize(rel_pos, rel_pos_norm);

                // Find the tangent point of the next waypoint
                float outter_pt[3];
                outter_pt[X] = next.local_pos()[X] + rel_pos_norm[Y]*next.radius();
                outter_pt[Y] = next.local_pos()[Y] - rel_pos_norm[X]*next.radius();
                outter_pt[Z] = 0.0f;

                // Determine the relative position to this tangent point
                for (i=0;i<3;i++)
                {
                    rel_pos[i] = outter_pt[i]-ins_.position_lf()[i];
                }

                // Find angle between our x axis and line to next tangent point
                std::array<float,3> vel = ins_.velocity_lf();
                float rel_heading = maths_calc_smaller_angle(atan2(rel_pos[Y],rel_pos[X]) - atan2(vel[Y], vel[X]));

                if (maths_f_abs(rel_heading) < navigation_.heading_acceptance)
                {
                    return true;
                }
            }
        }
    }

    return false;
}
