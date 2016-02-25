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


Gps_mocap::Gps_mocap(mavlink_message_handler_t& message_handler, gps_mocap_conf_t config):
    message_handler_(message_handler),
    config_(config),
    is_init(false)
{

}


bool Gps_mocap::init(void)
{
    return true;
}


bool Gps_mocap::update(void)
{
    return true;
}


void Gps_mocap::configure(void)
{

}


const float Gps_mocap::last_update_us(void) const
{
    return 0.0f;
}


const float Gps_mocap::last_position_update_us(void) const
{
    return 0.0f;
}


const float Gps_mocap::last_velocity_update_us(void) const
{
    return 0.0f;
}


const global_position_t Gps_mocap::position_gf(void) const
{
    return config_.origin;
}


const float Gps_mocap::horizontal_position_accuracy(void) const
{
    return 0.0f;
}


const float Gps_mocap::vertical_position_accuracy(void) const
{
    return 0.0f;
}


const std::array<float, 3> Gps_mocap::velocity_lf(void) const
{
    return std::array<float, 3>{{0.0f, 0.0f, 0.0f}};
}


const float Gps_mocap::velocity_accuracy(void) const
{
    return 0.0f;
}


const float Gps_mocap::heading(void) const
{
    return 0.0f;
}


const float Gps_mocap::heading_accuracy(void) const
{
    return 0.0f;
}


const uint8_t Gps_mocap::num_sats(void) const
{
    return 0;
}


const bool Gps_mocap::fix(void) const
{
    return true;
}


const bool Gps_mocap::healthy(void) const
{
    return true;
}