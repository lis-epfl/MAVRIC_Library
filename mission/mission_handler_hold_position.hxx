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
 * \file mission_handler_hold_position.hxx
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler functions for the hold position state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_HOLD_POSITION_HXX__
#define MISSION_HANDLER_HOLD_POSITION_HXX__

template <class T>
Mission_handler_hold_position<T>::Mission_handler_hold_position(T& controller,
                                                                const INS& ins,
                                                                Navigation& navigation):
            Mission_handler(),
            controller_(controller),
            ins_(ins),
            navigation_(navigation)
{
    waypoint_ = Waypoint(   MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_LOITER_UNLIM,
                            0,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
}

template <class T>
bool Mission_handler_hold_position<T>::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LOITER_UNLIM ||
        cmd == MAV_CMD_NAV_LOITER_TIME  ||
        cmd == MAV_CMD_NAV_LOITER_TO_ALT ||
        (cmd == MAV_CMD_OVERRIDE_GOTO && wpt.param1() == MAV_GOTO_DO_HOLD))
    {
        handleable = true;
    }

    return handleable;
}

template <class T>
bool Mission_handler_hold_position<T>::setup(Mission_planner& mission_planner, const Waypoint& wpt)
{
    bool success = true;
    waypoint_ = wpt;
    start_time_ = time_keeper_get_ms();
    within_radius_ = false;
    navigation_.set_waiting_at_waypoint(true);

    return success;
}

template <class T>
int Mission_handler_hold_position<T>::update(Mission_planner& mission_planner)
{
    // Set goal
    bool ret = set_control_command(mission_planner);

    /*******************
    Determine status code 
    ********************/
    // Determine if we have entered the hold position volume
    local_position_t wpt_pos = waypoint_.local_pos();

    float radius;
    if (!waypoint_.radius(radius))
    {
        radius = 0.0f;
    }
    
    // Check if we are doing dubin circles
    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN && navigation_.dubin_state == DUBIN_CIRCLE2)
    {
        within_radius_ = true;
    }
    // Or that we are at the waypoint
    else if (navigation_.navigation_strategy == Navigation::strategy_t::DIRECT_TO &&
            ((wpt_pos[X]-ins_.position_lf()[X])*(wpt_pos[X]-ins_.position_lf()[X]) + (wpt_pos[Y]-ins_.position_lf()[Y])*(wpt_pos[Y]-ins_.position_lf()[Y])) < radius*radius)
    {
        within_radius_ = true;
    }
    else
    {
        within_radius_ = false;

        // Reset start time
        start_time_ = time_keeper_get_ms();
    }

    // Determine if we should advance to the next waypoint
    if (waypoint_.autocontinue() == 1)
    {
        switch (waypoint_.command())
        {
        case MAV_CMD_NAV_LOITER_UNLIM:
            break;

        case MAV_CMD_NAV_LOITER_TIME:
            if (within_radius_ && ((time_keeper_get_ms() - start_time_) > waypoint_.param1() * 1000))
            {
                return 1;
            }
            break;

        case MAV_CMD_NAV_LOITER_TO_ALT:
            if (maths_f_abs(ins_.position_lf()[Z] - waypoint_.local_pos()[Z]) < waypoint_.param2()) // TODO: Add check for heading
            {
                return 1;
            }
            break;
        }
    }

    // Handle control command failed status
    if (!ret)
    {
        return -1;
    }

    return 0;
}

template <class T>
Mission_planner::internal_state_t Mission_handler_hold_position<T>::handler_mission_state() const
{
    return Mission_planner::MISSION;
}

#endif // MISSION_HANDLER_HOLD_POSITION_HXX__
