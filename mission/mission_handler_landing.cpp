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
 * \file mission_handler_landing.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the landing state
 *
 ******************************************************************************/


#include "mission/mission_handler_landing.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template <class T2>
bool Mission_handler_landing<INavigation_controller>::set_desc_to_small_alt_control_command(Mission_planner& mission_planner)
{
	INavigation_controller::nav_command_t cmd;
	cmd.local_position = waypoint_.local_pos();
	cmd.local_position[Z] = navigation_.takeoff_altitude/2.0f;
	desc_to_small_alt_controller_.set_navigation_command(cmd);
}

template <class T1>
bool Mission_handler_landing<IPosition_zvelocity_controller>::set_desc_to_ground_control_command(Mission_planner& mission_planner)
{
	IPosition_zvelocity_controller::xy_pos_z_vel_command_t cmd;
	cmd.x_pos = waypoint_.local_pos()[X];
	cmd.y_pos = waypoint_.local_pos()[Y];
	cmd.z_vel = 0.3f;
    controller_.set_position_zvel_command(cmd);
}
