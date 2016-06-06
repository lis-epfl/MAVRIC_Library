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

Mission_planner_handler_manual_control::Mission_planner_handler_manual_control( Position_estimation& position_estimation_,
                                                                                Navigation& navigation_,
                                                                                State& state_):
            position_estimation_(position_estimation_),
            state_(state_),
            navigation_(navigation_)
{

}

Mission_planner_handler_manual_control::handle(Mission_planner& mission_planner)
{
    mav_mode_t mode_local = state_.mav_mode();

    if (mav_modes_is_auto(mode_local))
    {
        navigation_.internal_state_ = Navigation::NAV_NAVIGATING;
    }
    else if (mav_modes_is_guided(mode_local))
    {
        print_util_dbg_print("Switching to NAV_HOLD_POSITION from NAV_MANUAL_CTRL\r\n");
        hold_init(position_estimation_.local_position);
        navigation_.internal_state_ = Navigation::NAV_HOLD_POSITION;
    }

    navigation_.critical_behavior = Navigation::CLIMB_TO_SAFE_ALT;
    critical_next_state_ = false;
    navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
}
