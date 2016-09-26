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
 * \file mission_handler_on_ground.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the on ground state
 *
 ******************************************************************************/


#include "mission/mission_handler_on_ground.hpp"

extern "C"
{

}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_handler_on_ground::Mission_handler_on_ground(Rate_controller_I& rate_controller):
            Mission_handler(),
            rate_controller_(rate_controller)
{

}

bool Mission_handler_on_ground::can_handle(const Waypoint& wpt) const
{
    // TODO: Check if actually on ground
    return wpt.command() == MAV_CMD_NAV_ON_GROUND;
}

bool Mission_handler_on_ground::setup(const Waypoint& wpt)
{
    return true;
}

Mission_handler::update_status_t Mission_handler_on_ground::update()
{
	// Set prop speeds to lowest setting
	Rate_controller_I::rate_command_t cmd;
	cmd.rates = {0.0f, 0.0f, 0.0f};
    cmd.thrust = -1.0f;
	rate_controller_.set_rate_command(cmd);
	
    return MISSION_IN_PROGRESS;
}

Mission_planner::internal_state_t Mission_handler_on_ground::handler_mission_state() const
{
    return Mission_planner::STANDBY;
}
