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
 * \file magnetometer_sim.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulated magnetometers
 *
 ******************************************************************************/


#include "simulation/magnetometer_sim.hpp"

extern "C"
{
#include "util/constants.h"
}


Magnetometer_sim::Magnetometer_sim(Dynamic_model& dynamic_model):
    dynamic_model_(dynamic_model),
    mag_field_(std::array<float,3>{{0.0f, 0.0f, 0.0f}}),
    temperature_(24.0f) // Nice day
{}


bool Magnetometer_sim::init(void)
{
    return true;
}


bool Magnetometer_sim::update(void)
{
    bool success = true;

    // Update dynamic model
    success &= dynamic_model_.update();

    // Field pointing 60 degrees down to the north (NED)
    const float mag_field_lf[3]     = { 0.5f, 0.0f, 0.86f };
    float mag_field_bf[3];

    // Get current attitude
    quat_t attitude = dynamic_model_.attitude();

    // Get magnetic field in body frame
    quaternions_rotate_vector(quaternions_inverse(attitude), mag_field_lf, mag_field_bf);
    // quaternions_rotate_vector( attitude, mag_field_lf, mag_field_bf);

    // Save in member array
    mag_field_[X] = mag_field_bf[X];
    mag_field_[Y] = mag_field_bf[Y];
    mag_field_[Z] = mag_field_bf[Z];

    return success;
}


const float& Magnetometer_sim::last_update_us(void) const
{
    return dynamic_model_.last_update_us();
}


const std::array<float, 3>& Magnetometer_sim::mag(void) const
{
    return mag_field_;
}


const float& Magnetometer_sim::mag_X(void) const
{
    return mag_field_[X];
}


const float& Magnetometer_sim::mag_Y(void) const
{
    return mag_field_[Y];
}


const float& Magnetometer_sim::mag_Z(void) const
{
    return mag_field_[Z];
}


const float& Magnetometer_sim::temperature(void) const
{
    return temperature_;
}