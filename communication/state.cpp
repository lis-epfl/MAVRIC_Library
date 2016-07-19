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
 * \file state.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Holds the current status of the MAV
 *
 ******************************************************************************/


#include "communication/state.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

State::State(Mavlink_stream& mavlink_stream, Battery& battery, State::conf_t config):
    battery_(battery),
    mavlink_stream_(mavlink_stream)
{
    // Init parameters
    autopilot_type = config.autopilot_type;
    autopilot_name = config.autopilot_name;

    mav_mode_ = config.mav_mode;
    mav_mode_.set_hil_flag(config.simulation_mode);
    mav_mode_.set_armed_flag(false);

    mav_state_ = config.mav_state;
    mav_mode_custom = config.mav_mode_custom;

    sensor_present = config.sensor_present;
    sensor_enabled = config.sensor_enabled;
    sensor_health = config.sensor_health;

    fence_1_xy = config.fence_1_xy;
    fence_1_z = config.fence_1_z;
    fence_2_xy = config.fence_2_xy;
    fence_2_z = config.fence_2_z;
    out_of_fence_1 = false;
    out_of_fence_2 = false;

    nav_plan_active = false;

    reset_position = false;

    last_heartbeat_msg = time_keeper_get_s();
    max_lost_connection = config.max_lost_connection;
    connection_lost = false;
    first_connection_set = false;

    msg_count = 0;
}

void State::switch_to_active_mode(mav_state_t* mav_state_)
{
    *mav_state_ = MAV_STATE_ACTIVE;

    // Tell other modules to reset position and re-compute waypoints
    reset_position = true;
    nav_plan_active = false;

    print_util_dbg_print("Switching to active mode.\r\n");
}

void State::connection_status()
{
    if (((time_keeper_get_s() - last_heartbeat_msg) > max_lost_connection) && (first_connection_set))
    {
        connection_lost = true;
    }
    else
    {
        connection_lost = false;
    }
}

bool State::set_armed(bool arming)
{
    // if already in desired state, return true
    if(is_armed() == arming)
    {
        return true;
    }

    if(arming)
    {
        // check if imu is ready
        if(mav_state_ == MAV_STATE_CALIBRATING)
        {
            print_util_dbg_print("[STATE]: prevented arming because IMU not ready\r\n");
            return false;
        }

        // check if battery not low
        if(battery_.is_low())
        {
            print_util_dbg_print("[STATE]: prevented arming because battery low\r\n");
            return false;
        }

        // set armed flag
        print_util_dbg_print("[STATE]: arming\r\n");
        mav_mode_.set_armed_flag(true);

        //resetting custom flag
        mav_mode_custom = Mav_mode::CUSTOM_BASE_MODE;

        // switch state to active
        switch_to_active_mode(&mav_state_);
    }
    else
    {
        // clear armed flag
        mav_mode_.set_armed_flag(false);
        print_util_dbg_print("[STATE]: disarming\r\n");

        // set state to standby
        mav_state_ = MAV_STATE_STANDBY;
    }

    return true;
}
