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
#include "control/inavigation_controller.hpp"
 #include "control/ixyposition_zvel_controller.hpp"
extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template <>
bool Mission_handler_landing<INavigation_controller, IXyposition_zvel_controller>::set_desc_to_small_alt_control_command(Mission_planner& mission_planner)
{
	INavigation_controller::nav_command_t cmd;
	cmd.pos = waypoint_.local_pos();
	cmd.pos[Z] = navigation_.takeoff_altitude/2.0f;
    return desc_to_small_alt_controller_.set_navigation_command(cmd);
}

template <>
bool Mission_handler_landing<INavigation_controller,IXyposition_zvel_controller>::set_desc_to_ground_control_command(Mission_planner& mission_planner)
{
	IXyposition_zvel_controller::xypos_zvel_command_t cmd;
	cmd.pos_x = waypoint_.local_pos()[X];
	cmd.pos_y = waypoint_.local_pos()[Y];
	cmd.vel_z = 0.3f;
    return desc_to_ground_controller_.set_xyposition_zvel_command(cmd);
}
