/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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
 *   
 * \brief This file defines the barometer enum and structure, independently from which sensor is used
 *
 ******************************************************************************/

#include <stdint.h>
#include "barometer.hpp"


Barometer::Barometer( )
{}


 void Barometer::reset_origin_altitude(float origin_altitude)
{
	altitude_offset_ = - (altitude_ - altitude_offset_ - origin_altitude );
}

float Barometer::get_pressure(void) const
{
	return pressure_;
}

float Barometer::get_vario_vz(void) const
{
	return vario_vz_;
}

float Barometer::get_temperature(void) const
{
	return temperature_;
}

uint32_t Barometer::get_last_update(void) const
{
	return last_update_;
}

float Barometer::get_altitude(void) const
{
	return altitude_;
}

float Barometer::get_altitude_offset(void) const
{
	return altitude_offset_;
}

void Barometer::set_vario_vz(float simulated_vario_vz)
{
	vario_vz_ = simulated_vario_vz;
}

void Barometer::set_altitude(float simulated_altitude)
{
	altitude_ = simulated_altitude;
}

void Barometer::set_altitude_offset(float simulated_altitude_offset)
{
	altitude_offset_ = simulated_altitude_offset;
}

void Barometer::set_last_update(uint32_t simulated_last_update)
{
	last_update_ = simulated_last_update;
}
