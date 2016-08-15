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
 * \file mission_handler_takeoff.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the takeoff state
 *
 ******************************************************************************/


#include "control/mission_handler_takeoff.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_handler_takeoff::Mission_handler_takeoff(   const INS& ins,
                                                    Navigation& navigation,
                                                    State& state):
            Mission_handler(),
            ins_(ins),
            navigation_(navigation),
            state_(state)
{
    waypoint_ = Waypoint(   MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_TAKEOFF,
                            1,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            navigation_.takeoff_altitude);
}

bool Mission_handler_takeoff::can_handle(const Waypoint& wpt)
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_TAKEOFF)
    {
        handleable = true;
    }

    return handleable;
}

bool Mission_handler_takeoff::setup(Mission_planner& mission_planner, const Waypoint& wpt)
{
    bool success = true;

    waypoint_ = wpt;

    navigation_.set_waiting_at_waypoint(false);
    mission_planner.set_internal_state(Mission_planner::PREMISSION);

    print_util_dbg_print("Automatic take-off, will hold position at: (");
    print_util_dbg_print_num(wpt.local_pos()[X], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(wpt.local_pos()[Y], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(wpt.local_pos()[Z], 10);
    print_util_dbg_print(")\r\n");

    return success;
}

void Mission_handler_takeoff::handle(Mission_planner& mission_planner)
{
    // Set goal
    navigation_.set_goal(waypoint_);
}

bool Mission_handler_takeoff::is_finished(Mission_planner& mission_planner)
{
    bool finished = false;

    // Determine distance to the waypoint
    if (waypoint_.autocontinue() == 1)
    {
        float waypoint_radius = navigation_.takeoff_altitude*navigation_.takeoff_altitude*0.16f;

        local_position_t wpt_pos = waypoint_.local_pos();
        float rel_pos[3];
        for (int i = 0; i < 3; i++)
        {
            rel_pos[i] = wpt_pos[i] - ins_.position_lf()[i];
        }
        navigation_.dist2wp_sqr = vectors_norm_sqr(rel_pos);

        // Determine if finished
        
        switch(navigation_.navigation_strategy)
        {
        case Navigation::strategy_t::DIRECT_TO:
           if (navigation_.dist2wp_sqr <= waypoint_radius)
            {
                finished = true;
                navigation_.set_waiting_at_waypoint(true);
            }
            break;

        case Navigation::strategy_t::DUBIN:
            if (state_.autopilot_type == MAV_TYPE_QUADROTOR)
            {
                if (navigation_.dist2wp_sqr <= waypoint_radius)
                {
                    finished = true;
                    navigation_.set_waiting_at_waypoint(true);
                }
            }
            else
            {
                if (ins_.position_lf()[Z] <= navigation_.takeoff_altitude)
                {
                    finished = true;
                    navigation_.set_waiting_at_waypoint(true);
                }
            }
            break;
        }
    }

    return finished;
}
