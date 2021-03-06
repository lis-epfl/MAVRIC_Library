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
 * \file gps_sim.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulation for GPS
 *
 ******************************************************************************/


#include "simulation/gps_sim.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"


Gps_sim::Gps_sim(Dynamic_model& dynamic_model):
    dynamic_model_(dynamic_model),
    last_update_us_(0.0f),
    last_position_update_us_(0.0f),
    last_velocity_update_us_(0.0f),
    global_position_({0.0, 0.0, 0.0f}),
    horizontal_position_accuracy_(0.0f),
    vertical_position_accuracy_(0.0f),
    velocity_lf_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    velocity_accuracy_(0.0f),
    heading_(0.0f),
    heading_accuracy_(0.0f),
    num_sats_(0),
    fix_(FIX_ERR),
    healthy_(false)
{}


bool Gps_sim::init(void)
{
    return true;
}


bool Gps_sim::update(void)
{
    bool success = true;

    // Update dynamic model
    success &= dynamic_model_.update();

    // Copy relevant fields in class members
    last_update_us_          = dynamic_model_.last_update_us();
    last_position_update_us_ = dynamic_model_.last_update_us();
    last_velocity_update_us_ = dynamic_model_.last_update_us();
    global_position_.longitude  = dynamic_model_.position_gf().longitude;
    global_position_.latitude   = dynamic_model_.position_gf().latitude;
    global_position_.altitude   = dynamic_model_.position_gf().altitude;
    horizontal_position_accuracy_   = 1.0f;
    vertical_position_accuracy_     = 3.0f;
    velocity_lf_        = dynamic_model_.velocity_lf();
    velocity_accuracy_  = 0.1f;
    heading_            = coord_conventions_get_yaw(dynamic_model_.attitude());
    heading_accuracy_   = 5.0f;
    num_sats_   = 5;
    healthy_    = true;
    fix_        = FIX_3D;

    return success;
}

void Gps_sim::configure(void)
{
    ;
}

float Gps_sim::last_update_us(void) const
{
    return last_update_us_;
}


float Gps_sim::last_position_update_us(void) const
{
    return last_position_update_us_;
}


float Gps_sim::last_velocity_update_us(void) const
{
    return last_velocity_update_us_;
}


global_position_t Gps_sim::position_gf(void) const
{
    return global_position_;
}


float Gps_sim::horizontal_position_accuracy(void) const
{
    return horizontal_position_accuracy_;
}


float Gps_sim::vertical_position_accuracy(void) const
{
    return vertical_position_accuracy_;
}


std::array<float, 3> Gps_sim::velocity_lf(void) const
{
    return velocity_lf_;
}


float Gps_sim::velocity_accuracy(void) const
{
    return velocity_accuracy_;
}


float Gps_sim::heading(void) const
{
    return heading_;
}


float Gps_sim::heading_accuracy(void) const
{
    return heading_accuracy_;
}


uint8_t Gps_sim::num_sats(void) const
{
    return num_sats_;
}


gps_fix_t Gps_sim::fix(void) const
{
    return fix_;
}


bool Gps_sim::healthy(void) const
{
    return healthy_;
}
