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
 * \file mission_planner_handler.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler
 *
 ******************************************************************************/


#include "communication/mission_planner_handler.hpp"
#include "communication/mission_planner.hpp"

extern "C"
{
}

//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Mission_planner_handler::hold_waypoint_set_ = false;
Waypoint Mission_planner_handler::hold_waypoint_ = Waypoint();

Waypoint& Mission_planner_handler::hold_waypoint()
{
    if (!hold_waypoint_set_)
    {
        hold_waypoint_.set_local_pos(position_estimation_.local_position);
        hold_waypoint_set_ = true;
    }

    return hold_waypoint_;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler::Mission_planner_handler(const Position_estimation& position_estimation) :
        position_estimation_(position_estimation)
{

}

void Mission_planner_handler::reset_hold_waypoint()
{
    hold_waypoint_set_ = false;
}

bool Mission_planner_handler::hold_waypoint_set()
{
    return hold_waypoint_set_;
}

void Mission_planner_handler::set_hold_waypoint(const local_position_t hold_position)
{
    hold_waypoint_.set_local_pos(hold_position);
    hold_waypoint_set_ = true;
}

void Mission_planner_handler::set_hold_waypoint(const Waypoint wpt)
{
    hold_waypoint_ = wpt;
    hold_waypoint_set_ = true;
}
