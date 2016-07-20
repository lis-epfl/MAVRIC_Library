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
 * \file barometer.cpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *
 * \brief Interface class for barometers
 *
 ******************************************************************************/

#include "drivers/barometer.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/maths.h"
}

Barometer::Barometer() :
    has_been_read_(false),
    has_been_calibrated_(false)
{

}

const float& Barometer::last_update_us(void) const
{
    return last_update_us_;
}


const float& Barometer::pressure(void)  const
{
    return pressure_;
}


const float& Barometer::altitude_gf(void) const
{
    return altitude_gf_;
}


const float& Barometer::vertical_speed_lf(void) const
{
    return speed_lf_;
}


const float& Barometer::temperature(void) const
{
    return temperature_;
}

bool Barometer::has_been_calibrated() const
{
    return has_been_calibrated_;
}

void Barometer::calibrate_bias(float current_altitude_gf)
{
    if (has_been_read_)
    {
        altitude_bias_gf_ = altitude_filtered - current_altitude_gf;
        altitude_gf_ = current_altitude_gf;
        has_been_calibrated_ = true;
    }
}


float Barometer::altitude_from_pressure(float pressure, float altitude_bias)
{
    return 44330.0f * (1.0f - pow(pressure / 101325.0f, 0.190295f)) - altitude_bias;
}