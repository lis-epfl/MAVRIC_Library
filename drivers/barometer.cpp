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

// Definition of static member
float Barometer::pressure_at_sea_level_;


Barometer::Barometer(float pressure_at_sea_level)
{
    Barometer::pressure_at_sea_level_ = pressure_at_sea_level;
}

float Barometer::compute_pressure_at_sea_level(void)
{
    return pressure_at_sea_level_;
}


void Barometer::set_pressure_at_sea_level(float pressure_at_sea_level)
{
    Barometer::pressure_at_sea_level_ = pressure_at_sea_level;
};


float Barometer::altitude_from_pressure(float pressure, float pressure_at_sea_level)
{
    //from https://en.wikipedia.org/wiki/Atmospheric_pressure
    return 44330.0f * (1.0f - pow(pressure / pressure_at_sea_level, 0.190295f));
}


float Barometer::compute_pressure_at_sea_level(float pressure, float altitude)
{
    //from https://en.wikipedia.org/wiki/Atmospheric_pressure
    return pressure / pow(1.0f - altitude / 44330.0f, 5.255f);
}
