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
 * \file mission_planner_handler_stop_there.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the stop there state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_stop_there.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_stop_there::Mission_planner_handler_stop_there( Position_estimation& position_estimation,
                                                                        Navigation& navigation,
                                                                        State& state):
            position_estimation_(position_estimation),
            navigation_(navigation),
            state_(state)
{

}

void Mission_planner_handler_stop_there::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    stopping_handler(mission_planner);
    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
    {
        mission_planner.dubin_state_machine(&mission_planner.waypoint_hold_coordinates);
    }

    navigation_.goal = mission_planner.waypoint_hold_coordinates;

    if (mode_local.is_manual())
    {
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}

void Mission_planner_handler_stop_there::stopping_handler(Mission_planner& mission_planner)
{
    float dist2wp_sqr;
    float rel_pos[3];

    rel_pos[X] = (float)(mission_planner.waypoint_hold_coordinates.local_pos().pos[X] - position_estimation_.local_position.pos[X]);
    rel_pos[Y] = (float)(mission_planner.waypoint_hold_coordinates.local_pos().pos[Y] - position_estimation_.local_position.pos[Y]);
    rel_pos[Z] = (float)(mission_planner.waypoint_hold_coordinates.local_pos().pos[Z] - position_estimation_.local_position.pos[Z]);

    dist2wp_sqr = vectors_norm_sqr(rel_pos);
    if (dist2wp_sqr < 25.0f)
    {
        navigation_.internal_state_ = Navigation::NAV_STOP_ON_POSITION;
    }
}
