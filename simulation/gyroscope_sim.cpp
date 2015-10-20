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
 * \file gyroscope_sim.cpp
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Simulated gyroscopes
 * 
 ******************************************************************************/


#include "gyroscope_sim.hpp"

extern "C"
{
	#include "constants.h"
}

Gyroscope_sim::Gyroscope_sim(Dynamic_model& dynamic_model):
	dynamic_model_( dynamic_model ),
	rates_( std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	temperature_(24.0f) // Nice day
{}


bool Gyroscope_sim::init(void)
{
	return true;
}


bool Gyroscope_sim::update(void)
{
	bool success = true;

	// Update dynamic model
	success &= dynamic_model_.update();

	// Get angular velocity
	rates_ = dynamic_model_.angular_velocity_bf();

	return success;
}


const float& Gyroscope_sim::last_update_us(void) const
{
	return dynamic_model_.last_update_us();
}


const std::array<float, 3>& Gyroscope_sim::gyro(void) const
{
	return rates_;
}


const float& Gyroscope_sim::gyro_X(void) const
{
	return rates_[X];
}


const float& Gyroscope_sim::gyro_Y(void) const
{
	return rates_[Y];
}


const float& Gyroscope_sim::gyro_Z(void) const
{
	return rates_[Z];
}


const float& Gyroscope_sim::temperature(void) const
{
	return temperature_;
}
