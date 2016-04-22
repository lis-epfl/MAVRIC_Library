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
 * \file gps_mocap.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Expose data received from motion capture system as GPS data
 *
 ******************************************************************************/


#include "drivers/gps_mocap.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/constants.h"
#include "util/quick_trig.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Callback function used to update the gps when a mavlink message is received
 *
 * \param   gps_mocap     Pointer to Gps_mocap object
 * \param   sysid         ID of the system
 * \param   msg           Pointer to the incoming message
 */
static void gps_mocap_callback(Gps_mocap* gps_mocap, uint32_t sysid, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
static void gps_mocap_callback(Gps_mocap* gps_mocap, uint32_t sysid, mavlink_message_t* msg)
{
   gps_mocap->callback(sysid, msg);
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Gps_mocap::Gps_mocap(mavlink_message_handler_t& message_handler, gps_mocap_conf_t config):
    message_handler_(message_handler),
    config_(config),
    is_init_(false),
    is_healthy_(false),
    last_update_us_(0.0f)
{
    local_position_.pos[X]  = 0.0f;
    local_position_.pos[Y]  = 0.0f;
    local_position_.pos[Z]  = 0.0f;
    local_position_.heading = 0.0f;
    local_position_.origin  = config_.origin;

    velocity_lf_[X] = 0.0f;
    velocity_lf_[Y] = 0.0f;
    velocity_lf_[Z] = 0.0f;

    heading_ = 0.0f;
}


bool Gps_mocap::init(void)
{
    if (is_init_ == false)
    {
        // Add callbacks for waypoint handler messages requests
        mavlink_message_handler_msg_callback_t callback;

        callback.message_id     = MAVLINK_MSG_ID_ATT_POS_MOCAP; // 69
        callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
        callback.compid_filter  = MAV_COMP_ID_ALL;
        callback.function       = (mavlink_msg_callback_function_t) &gps_mocap_callback;
        callback.module_struct  = (handling_module_struct_t)        this;

        is_init_ = mavlink_message_handler_add_msg_callback(&message_handler_, &callback);
    }

    return is_init_;

}


void Gps_mocap::callback(uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_att_pos_mocap_t packet;
    mavlink_msg_att_pos_mocap_decode(msg, &packet);

    // Get timing
    float t = time_keeper_get_us();

    // Update velocity
    float dt_s = (t - last_update_us_) / 1000000;
    if (dt_s > 0.0f)
    {
        velocity_lf_[X] = (packet.x - local_position_.pos[X]) / dt_s;
        velocity_lf_[Y] = (packet.y - local_position_.pos[Y]) / dt_s;
        velocity_lf_[Z] = (packet.z - local_position_.pos[Z]) / dt_s;

        if (velocity_lf_[X] != 0.0f)
        {
            heading_ = quick_trig_atan(velocity_lf_[Y] / velocity_lf_[X]);
        }
    }

    // Update position
    local_position_.pos[X] = packet.x;
    local_position_.pos[Y] = packet.y;
    local_position_.pos[Z] = packet.z;

    // Update timing
    last_update_us_ = t;
}


bool Gps_mocap::update(void)
{
    float t = time_keeper_get_us();

    if ( (t - last_update_us()) > 1000000 )
    {
         is_healthy_ = false;
    }
    else
    {
        is_healthy_ = true;
    }

    return true;
}


void Gps_mocap::configure(void)
{
    ;
}


const float Gps_mocap::last_update_us(void) const
{
    return last_update_us_;
}


const float Gps_mocap::last_position_update_us(void) const
{
    return last_update_us_;
}


const float Gps_mocap::last_velocity_update_us(void) const
{
    return last_update_us_;
}


const global_position_t Gps_mocap::position_gf(void) const
{
    return coord_conventions_local_to_global_position(local_position_);
}


const float Gps_mocap::horizontal_position_accuracy(void) const
{
    return config_.horizontal_position_accuracy;
}


const float Gps_mocap::vertical_position_accuracy(void) const
{
    return config_.vertical_position_accuracy;
}


const std::array<float, 3> Gps_mocap::velocity_lf(void) const
{
    return velocity_lf_;
}


const float Gps_mocap::velocity_accuracy(void) const
{
    return config_.velocity_accuracy;
}


const float Gps_mocap::heading(void) const
{
    return heading_;
}


const float Gps_mocap::heading_accuracy(void) const
{
    return config_.heading_accuracy;
}


const uint8_t Gps_mocap::num_sats(void) const
{
    return 16;
}


const gps_fix_t Gps_mocap::fix(void) const
{
    gps_fix_t fix = NO_GPS;
    if (healthy())
    {
        fix = GPS_OK;
    }

    return fix;
}


const bool Gps_mocap::healthy(void) const
{
    return is_healthy_;
}
