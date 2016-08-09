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
 * \file mission_handler_hold_position.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the hold position state
 *
 ******************************************************************************/


#include "control/mission_handler_hold_position.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_handler_hold_position::Mission_handler_hold_position(   const INS& ins,
                                                                Navigation& navigation,
                                                                State& state):
            Mission_handler(),
            ins_(ins),
            navigation_(navigation),
            state_(state)
{

}

bool Mission_handler_hold_position::init()
{
    return true;
}

bool Mission_handler_hold_position::can_handle(Mssion_planner& mission_planner, Waypoint& wpt)
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LOITER_UNLIM ||
        cmd == MAV_CMD_NAV_LOITER_TIME  ||
        cmd == MAV_CMD_NAV_LOITER_TO_ALT)
    {
        handleable = true;
    }

    return handleable;
}

bool Mission_handler_hold_position::setup(Mssion_planner& mission_planner, Waypoint& wpt)
{
    bool success = false;

    waypoint_ = wpt;
    start_time_ = time_keeper_get_ms();
}

void Mission_handler_hold_position::handle(Mssion_planner& mission_planner)
{
    // Set goal
    switch (waypoint_.command())
    {
    case MAV_CMD_NAV_LOITER_UNLIM:
        navigation_.set_goal(waypoint_.local_pos(), waypoint_.param4(), waypoint_.param3());
        break;

    case MAV_CMD_NAV_LOITER_TIME:
        navigation_.set_goal(waypoint_.local_pos(), waypoint_.param4(), waypoint_.param3());
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        navigation_.set_goal(waypoint_.local_pos(), coord_conventions_get_yaw(ahrs.qe), waypoint_.param2());
        break;

    default:
        navigation_.set_goal(waypoint_.local_pos(), 0.0f, 0.0f);
    }  
}

bool Mission_handler_hold_position::is_finished(Mssion_planner& mission_planner, Waypoint& wpt)
{
    switch (waypoint_.command())
    {
    case MAV_CMD_NAV_LOITER_UNLIM:
        return false;

    case MAV_CMD_NAV_LOITER_TIME:
        if ((time_keeper_get_ms() - start_time_) * 1000.0f > waypoint_.param1())
        {
            return true;
        }
        else
        {
            return false;
        }
        break;

    case MAV_CMD_NAV_LOITER_TO_ALT:
        if (maths_f_abs(ins_.position_lf()[Z] - waypoint_.local_pos()[Z]) < waypoint_.param2()) // TODO: Add check for heading
        {
            return true;
        }
        else
        {
            return false;
        }
        break;

    default:
        return false;
    }  
}
/*
void Mission_handler_hold_position::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    // Set the goal position to be the current hold position.
    // Do not update hold position as that can cause drift
    navigation_.set_goal(hold_waypoint());

    // Change state if necessary
    if (mode_local.is_auto())
    {
        navigation_.set_internal_state(Navigation::NAV_NAVIGATING);
    }
    else if (mode_local.is_manual())
    {
        navigation_.set_internal_state(Navigation::NAV_MANUAL_CTRL);
    }
}
*/