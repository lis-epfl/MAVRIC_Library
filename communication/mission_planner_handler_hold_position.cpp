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
 * \file mission_planner_handler_hold_position.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the hold position state
 *
 ******************************************************************************/


#include "communication/mission_planner_handler_hold_position.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_planner_handler_hold_position::Mission_planner_handler_hold_position(   const Position_estimation& position_estimation,
                                                                                Navigation& navigation,
                                                                                State& state):
            Mission_planner_handler(position_estimation),
            navigation_(navigation),
            state_(state)
{

}

bool Mission_planner_handler_hold_position::init()
{
    return true;
}

void Mission_planner_handler_hold_position::handle(Mission_planner& mission_planner)
{
    Mav_mode mode_local = state_.mav_mode();

    if (navigation_.navigation_strategy == Navigation::strategy_t::DUBIN)
    {
        mission_planner.dubin_state_machine(&hold_waypoint());
    }

    navigation_.goal = hold_waypoint();

    if (mode_local.is_auto())
    {
        print_util_dbg_print("Switching to NAV_NAVIGATING from NAV_HOLD_POSITION\r\n");
        navigation_.dubin_state = DUBIN_INIT;
        navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
    }
    else if (mode_local.is_manual())
    {
        print_util_dbg_print("Switching to Navigation::NAV_MANUAL_CTRL from Navigation::NAV_HOLD_POSITION\r\n");
        navigation_.internal_state_ = Navigation::NAV_MANUAL_CTRL;
    }
}
