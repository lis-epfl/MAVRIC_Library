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
 * \file mission_handler_takeoff.hxx
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler functions for the takeoff state
 *
 ******************************************************************************/


#ifndef MISSION_HANDLER_TAKEOFF_HXX__
#define MISSION_HANDLER_TAKEOFF_HXX__

template <class T>
Mission_handler_takeoff<T>::Mission_handler_takeoff<T>( T& controller,
                                                        const INS& ins,
                                                        Navigation& navigation,
                                                        State& state):
            Mission_handler(),
            controller_(controller),
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

template <class T>
bool Mission_handler_takeoff<T>::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_TAKEOFF)
    {
        handleable = true;
    }

    return handleable;
}

template <class T>
bool Mission_handler_takeoff<T>::setup(Mission_planner& mission_planner, const Waypoint& wpt)
{
    bool success = true;

    waypoint_ = wpt;

    navigation_.set_waiting_at_waypoint(false);

    print_util_dbg_print("Automatic take-off, will hold position at: (");
    print_util_dbg_print_num(wpt.local_pos()[X], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(wpt.local_pos()[Y], 10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(wpt.local_pos()[Z], 10);
    print_util_dbg_print(")\r\n");

    return success;
}

template <class T>
int Mission_handler_takeoff<T>::handle(Mission_planner& mission_planner)
{
    // Set goal
    bool ret = set_controller(mission_planner);

    /*********************
     Determine status code 
    **********************/
    bool finished = false;
    

    // Determine distance to the waypoint
    float xy_radius_sqr = navigation_.takeoff_altitude*navigation_.takeoff_altitude*0.16f;

    local_position_t wpt_pos = waypoint_.local_pos();
    float xy_dist2wp_sqr;
    float rel_pos[3];
    for (int i = 0; i < 2; i++)
    {
        rel_pos[i] = wpt_pos[i] - ins_.position_lf()[i];
    }
    rel_pos[2] = 0.0f;
    xy_dist2wp_sqr = vectors_norm_sqr(rel_pos);

    // Determine if finished
    switch(navigation_.navigation_strategy)
    {
    case Navigation::strategy_t::DIRECT_TO:
       if (xy_dist2wp_sqr <= xy_radius_sqr && ins_.position_lf()[Z] <= 0.9f * navigation_.takeoff_altitude)
        {
            finished = true;
            navigation_.set_waiting_at_waypoint(true);
        }
        break;

    case Navigation::strategy_t::DUBIN:
        if (state_.autopilot_type == MAV_TYPE_QUADROTOR)
        {
            if (xy_dist2wp_sqr <= xy_radius_sqr && ins_.position_lf()[Z] <= 0.9f * navigation_.takeoff_altitude)
            {
                finished = true;
                navigation_.set_waiting_at_waypoint(true);
            }
        }
        else
        {
            if (ins_.position_lf()[Z] <= 0.9f * navigation_.takeoff_altitude)
            {
                finished = true;
                navigation_.set_waiting_at_waypoint(true);
            }
        }
        break;
    }

    // Return true if waiting at waypoint and autocontinue
    if (finished && waypoint_.autocontinue() == 1)
    {
        return 1;
    }

    // Handle control command failed status
    if (!ret)
    {
        return -1;
    }

    return 0;
}

template <class T>
Mission_planner::internal_state_t Mission_handler_takeoff<T>::handler_mission_state() const
{
    return Mission_planner::PREMISSION;
}

#endif // MISSION_HANDLER_TAKEOFF_HXX__
