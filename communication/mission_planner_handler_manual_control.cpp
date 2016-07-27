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
 * \file mission_planner_handler_manual_control.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the manual control state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_manual_control.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_manual_control::Mission_planner_handler_manual_control( const INS& ins,
                                                                                Navigation& navigation,
                                                                                State& state):
            Mission_planner_handler(ins),
            navigation_(navigation),
            state_(state)
{

}

bool Mission_planner_handler_manual_control::init()
{
    return true;
}

void Mission_planner_handler_manual_control::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    // Change state if necessary
    if (mode_local.is_auto())
    {
        navigation_.set_internal_state(Navigation::NAV_NAVIGATING);
    }
    else if (mode_local.ctrl_mode() == Mav_mode::POSITION_HOLD)
    {
        set_hold_waypoint(ins_.position_lf());
        hold_waypoint().set_radius(navigation_.minimal_radius);
        navigation_.set_internal_state(Navigation::NAV_HOLD_POSITION);
    }

    // Reset behaviors of other states
    navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
    mission_planner.set_critical_next_state(false);
    navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
}
