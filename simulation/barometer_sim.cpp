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
 * \file barometer_sim.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Simulation for barometers
 *
 ******************************************************************************/


#include "simulation/barometer_sim.hpp"
#include "util/constants.hpp"
#include "hal/common/time_keeper.hpp"



Barometer_sim::Barometer_sim(Dynamic_model& dynamic_model):
    dynamic_model_(dynamic_model)
{
    pressure_           = 0.0f;
    temperature_        = 0.0f;
    altitude_gf_        = 0.0f;
    altitude_filtered   = 0.0f;
    altitude_bias_gf_   = 0.0f;
    speed_lf_           = 0.0f;
    last_update_us_     = 0.0f;
    temperature_        = 24.0f;    // Nice day
}


bool Barometer_sim::init(void)
{
    last_update_us_ = time_keeper_get_us();
    return true;
}


bool Barometer_sim::update(void)
{
    bool success = true;

    // Update dynamic model
    success &= dynamic_model_.update();

    // Get delta t
    float dt_s = (dynamic_model_.last_update_us() - last_update_us_) / 1000000.0f;

    if (dt_s > 0.0f)
    {
        // Get altitude
        float altitude_filtered_old = altitude_filtered;
        altitude_filtered = dynamic_model_.position_gf().altitude;
        
        // Get variation of altitude
        speed_lf_       = - (altitude_filtered - altitude_filtered_old) / dt_s;
        altitude_gf_    = altitude_filtered - altitude_bias_gf_;

        // Get pressure
        pressure_ = 0.0f; // TODO

    }

    // Save timing
    last_update_us_ = dynamic_model_.last_update_us();

    return success;
}
