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
 * \file mission_handler_land_on_tag.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The MAVLink mission planner handler for the land on tag state
 *
 ******************************************************************************/


#include "mission/mission_handler_land_on_tag.hpp"
#include "control/navigation_controller_i.hpp"
#include "control/xyposition_zvel_controller_i.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template <>
bool Mission_handler_land_on_tag<Navigation_controller_I, Navigation_controller_I, XYposition_Zvel_controller_I>::set_fly_to_landing_location_control_command()
{
	Navigation_controller_I::nav_command_t cmd;
	cmd.pos = waypoint_.local_pos();
    return fly_to_landing_location_controller_.set_navigation_command(cmd);
}

template <>
bool Mission_handler_land_on_tag<Navigation_controller_I, Navigation_controller_I, XYposition_Zvel_controller_I>::set_desc_to_small_alt_control_command()
{
	Navigation_controller_I::nav_command_t cmd;

	float tag_pos[2];
    float cur_pos[3];

    // Set position vectors to shorten code later
    for (uint8_t i = 0; i < 2; i++)
    {
        tag_pos[i] = offboard_tag_search_.tag_location()[i];
    }
    for (uint8_t i = 0; i < 3; i++)
    {
        cur_pos[i] = ins_.position_lf()[i];
    }

    // If the camera has detected the tag...
    if (offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_FOUND)
    {
        // The hold coordinates has been already been updated during the tag location reading...
        // Changed the z goal to ground if we are positioned directly above the tag
        float horizontal_distance_to_tag_sqr = (cur_pos[0] - tag_pos[0]) * (cur_pos[0] - tag_pos[0]) + (cur_pos[1] - tag_pos[1]) * (cur_pos[1] - tag_pos[1]);

        // Set hold location to tag location
        cmd.pos[X] = offboard_tag_search_.tag_location()[0];
        cmd.pos[Y] = offboard_tag_search_.tag_location()[1];

        // If we are not above tag or are not healthy
        if (horizontal_distance_to_tag_sqr > offboard_tag_search_.allowable_horizontal_tag_offset_sqr() || !offboard_tag_search_.is_healthy())
        {
            // Stay at tag search altitude if in descent to ground state
            cmd.pos[Z] = tag_search_altitude_;
        }
        else // Descend to ground if still healthy and above tag
        {
            cmd.pos[Z] = desc_to_ground_altitude_;

            // Set tag search altitude to current height, so it will reposition itself at this altitude if it drifts away
            tag_search_altitude_ = alt_lpf_;
        }
    }
    else if (offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND)// Else we need to search for the tag ...
    {
        cmd.pos = waypoint_.local_pos();
    }

    return desc_to_small_alt_controller_.set_navigation_command(cmd);
}

template <>
bool Mission_handler_land_on_tag<Navigation_controller_I, Navigation_controller_I, XYposition_Zvel_controller_I>::set_desc_to_ground_control_command()
{
    // If the camera has detected the tag...
    if (offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_FOUND)
    {
        // Descend to ground
        XYposition_Zvel_controller_I::xypos_zvel_command_t cmd;
        cmd.pos_x = offboard_tag_search_.tag_location()[0];
        cmd.pos_y = offboard_tag_search_.tag_location()[1];
        cmd.vel_z = 0.3f;
        return desc_to_ground_controller_.set_xyposition_zvel_command(cmd);
    }
    else if (offboard_tag_search_.land_on_tag_behavior() == Offboard_Tag_Search::land_on_tag_behavior_t::TAG_NOT_FOUND)// Else we need to search for the tag ...
    {
        // If we have not found the tag, switch to fly to waypoint
        // some error had occured
        print_util_dbg_print("[LAND ON TAG HANDLER] ERROR: tried to land without tag detected");
        auto_landing_behavior_ = FLY_TO_LANDING_LOCATION;
        tag_search_altitude_ = waypoint_.local_pos()[Z];
        return set_fly_to_landing_location_control_command();
    }

    return false; // Error
}
