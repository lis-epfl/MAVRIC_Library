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
 * \author Julien Lecoeur
 *
 * \brief The mission handler for the landing state
 *
 ******************************************************************************/


#include "mission/mission_handler_landing.hpp"
#include "navigation/navigation.hpp"


Mission_handler_landing::Mission_handler_landing(const INS& ins,
                                                 State& state,
                                                 conf_t config):
    Mission_handler(),
    ins_(ins),
    state_(state)
{
    waypoint_ = Waypoint (  MAV_FRAME_LOCAL_NED,
                            MAV_CMD_NAV_LAND,
                            0,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f,
                            0.0f);
    auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
    LPF_gain_ = config.LPF_gain;
    desc_to_ground_altitude_ = config.desc_to_ground_altitude;
    desc_to_ground_range_ = config.desc_to_ground_range;
}


bool Mission_handler_landing::can_handle(const Waypoint& wpt) const
{
    bool handleable = false;

    uint16_t cmd = wpt.command();
    if (cmd == MAV_CMD_NAV_LAND)
    {
        handleable = true;
    }

    return handleable;
}


bool Mission_handler_landing::setup(const Waypoint& wpt)
{
    bool success = true;

    print_util_dbg_print("Automatic-landing: descent_to_small_altitude\r\n");
    auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
    state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
    state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_SMALL_ALTITUDE;
    is_landed_ = false;

    waypoint_ = wpt;

    return success;
}


Mission_handler::update_status_t Mission_handler_landing::update()
{
    /*****************************
    Handle internal landing states
    *****************************/
    // Determine if we should switch between the landing states
    bool next_state = false;
    if (auto_landing_behavior_ == DESCENT_TO_GND)
    {
        alt_lpf_ = LPF_gain_ * (alt_lpf_) + (1.0f - LPF_gain_) * ins_.position_lf()[Z];
        if ((ins_.position_lf()[Z] > -0.1f) && (maths_f_abs(ins_.position_lf()[Z] - alt_lpf_) <= 0.2f))
        {
            next_state = true;
        }
    }
    else if (auto_landing_behavior_ == DESCENT_TO_SMALL_ALTITUDE)
    {
        float z_lim = desc_to_ground_altitude_- desc_to_ground_range_;
        if ((ins_.position_lf()[Z]) > z_lim)                                         // Drone is below desc2gnd waypoint (for cases when we start landing below it)
        {
            next_state = true;
        }
    }

    // If we are switching between states, ... then switch
    if (next_state)
    {
        switch (auto_landing_behavior_)
        {
            case DESCENT_TO_SMALL_ALTITUDE:
                print_util_dbg_print("Automatic-landing: descent_to_GND\r\n");
                auto_landing_behavior_ = DESCENT_TO_GND;
                state_.mav_mode_custom &= static_cast<Mav_mode::custom_mode_t>(0xFFFFFFE0);
                state_.mav_mode_custom |= Mav_mode::CUST_DESCENT_TO_GND;
                alt_lpf_ = ins_.position_lf()[Z];
                break;

            case DESCENT_TO_GND:
                print_util_dbg_print("Auto-landing: disarming motors \r\n");
                auto_landing_behavior_ = DESCENT_TO_SMALL_ALTITUDE;
                //Do not reset custom flag here, to be able to check after landing
                // in case something went wrong. Is reset while arming
                state_.set_armed(false);
                state_.mav_state_ = MAV_STATE_STANDBY;
                is_landed_ = true;
                break;
        }
    }


    /********************
    Determine status code
    ********************/
    if (waypoint_.autocontinue() == 1 && is_landed_)
    {
        return MISSION_FINISHED;
    }

    return MISSION_IN_PROGRESS;
}


bool Mission_handler_landing::write_flight_command(Flight_controller& flight_controller) const
{
    bool ret = false;

    switch (auto_landing_behavior_)
    {
        case DESCENT_TO_SMALL_ALTITUDE:
            ret = write_desc_to_small_alt_flight_command(flight_controller);
            break;

        case DESCENT_TO_GND:
            ret = write_desc_to_ground_flight_command(flight_controller);
            break;
    }

    return ret;
}


Mission_planner::internal_state_t Mission_handler_landing::handler_mission_state() const
{
    return Mission_planner::POSTMISSION;
}


//------------------------------------------------------------------------------
// PROTECTED/PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Mission_handler_landing::write_desc_to_small_alt_flight_command(Flight_controller& flight_controller) const
{
	position_command_t cmd;

    // Set position at configured altitude above landing location
	cmd.xyz    = waypoint_.local_pos();
	cmd.xyz[Z] = desc_to_ground_altitude_;

    // Set heading from waypoint
    cmd.heading = 0.0f;
    waypoint_.heading(cmd.heading);

    return flight_controller.set_command(cmd);
}


bool Mission_handler_landing::write_desc_to_ground_flight_command(Flight_controller& flight_controller) const
{
    position_command_t cmd;

    // Set position 1m bellow the drone
	cmd.xyz    = waypoint_.local_pos();
	cmd.xyz[Z] = ins_.position_lf()[Z] + 1.0f;

    // Set heading from waypoint
    cmd.heading = 0.0f;
    waypoint_.heading(cmd.heading);

    return flight_controller.set_command(cmd);
}
