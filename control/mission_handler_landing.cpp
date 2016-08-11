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


#include "control/mission_handler_landing.hpp"

extern "C"
{

}



//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mission_handler_landing::Mission_handler_landing(   const INS& ins,
                                                    Navigation& navigation,
                                                    const ahrs_t& ahrs,
                                                    State& state,
                                                    Mavlink_message_handler& message_handler):
            Mission_handler(ins),
            navigation_(navigation),
            ahrs_(ahrs),
            state_(state),
            message_handler_(message_handler)
{
}

bool Mission_handler_landing::can_handle(Mission_planner& mission_planner, Waypoint& wpt)
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LAND)
    {
        handleable = true;
    }

    return handleable;
}

bool Mission_handler_landing::setup(Mission_planner& mission_planner, Waypoint& wpt)
{
    bool success = true;

    print_util_dbg_print("Automatic-landing: descent_to_small_altitude\r\n");
    navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
    state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
    state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
    navigation_.set_waiting_at_waypoint(false);
    mission_planner.set_internal_state(Mission_planner::PREMISSION);

    waypoint_ = &wpt;

    return success;
}

void Mission_handler_landing::handle(Mission_planner& mission_planner)
{
    // Determine waypoint position
    if (waypoint_ != NULL)
    {
        local_position_t local_pos = waypoint_->local_pos();
        switch (navigation_.auto_landing_behavior)
        {
            case Navigation::DESCENT_TO_SMALL_ALTITUDE:
            {
                local_pos[Z] = navigation_.takeoff_altitude/2.0f;
                break;
            }

            case Navigation::DESCENT_TO_GND:
            {
                local_pos[Z] = 0.0f;
                break;
            }
        }

        // Determine if we should switch between the landing states
        if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_GND)
        {
            navigation_.alt_lpf = navigation_.LPF_gain * (navigation_.alt_lpf) + (1.0f - navigation_.LPF_gain) * ins_.position_lf()[Z];
            if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - navigation_.alt_lpf) <= 0.2f))
            {
                next_state = true;
            }
        }
        else if (navigation_.auto_landing_behavior == Navigation::DESCENT_TO_SMALL_ALTITUDE)
        {
            if (maths_f_abs(ins_.position_lf()[Z] - local_pos[Z]) < 0.5f)
            {
                next_state = true;
            }
        }

        // If we are switching between states, ... then switch
        if (next_state)
        {
            switch (navigation_.auto_landing_behavior)
            {
                case Navigation::DESCENT_TO_SMALL_ALTITUDE:
                    print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                    navigation_.auto_landing_behavior = Navigation::DESCENT_TO_GND;
                    state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                    state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
                    navigation_.alt_lpf = ins_.position_lf()[Z];
                    break;

                case Navigation::DESCENT_TO_GND:
                    print_util_dbg_print("Auto-landing: disarming motors \r\n");
                    navigation_.auto_landing_behavior = Navigation::DESCENT_TO_SMALL_ALTITUDE;
                    //Do not reset custom flag here, to be able to check after landing
                    // in case something went wrong. Is reset while arming
                    state_.set_armed(false);
                    state_.mav_state_ = MAV_STATE_STANDBY;
                    navigation_.set_waiting_at_waypoint(true);
                    break;
            }
        }

        // Set goal
        landing_waypoint = Waypoint(waypoint_->frame,
                                    waypoint_->command,
                                    waypoint_->autocontinue,
                                    waypoint_->param1,
                                    waypoint_->param2,
                                    waypoint_->param3,
                                    waypoint_->param4,
                                    local_pos[X],
                                    local_pos[Y],
                                    local_pos[Z])
        navigation_.set_goal(landing_waypoint);
    }
}

bool Mission_handler_landing::is_finished(Mission_planner& mission_planner)
{
    return mission_planner.internal_state() == Mission_planner::STANDBY;
}

void Mission_handler_landing::modify_control_command(control_command_t& control)
{
    if (navigation_auto_landing_behavior == Navigation::DESCENT_TO_GND)
    {
        // Constant velocity to the ground
        control.tvel[Z] = 0.3f;
    }
}