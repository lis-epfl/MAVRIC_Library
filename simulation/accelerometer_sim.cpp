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
 * \file accelerometer_sim.cpp 
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Simulated accelerometers
 *
 ******************************************************************************/



#include "accelerometer_sim.hpp"


extern "C"
{
	#include "constants.h"
}


Accelerometer_sim::Accelerometer_sim(Dynamic_model& dynamic_model):
	dynamic_model_( dynamic_model ),
	acceleration_( std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	temperature_(24.0f) // Nice day
{}


bool Accelerometer_sim::init(void)
{
	return true;
}


bool Accelerometer_sim::update(void)
{
	bool success = true;

	// Update dynamic model
	success &= dynamic_model_.update();

	// Get linear acceleration
	acceleration_ = dynamic_model_.linear_acceleration_bf();
	
	// Add gravity
	quat_t attitude = dynamic_model_.attitude();
	const quat_t up = { 0.0f, {UPVECTOR_X, UPVECTOR_Y, UPVECTOR_Z} };
	quat_t up_vec  	= quaternions_global_to_local(attitude, up);
	for( uint8_t i = 0; i < 3; ++i )
	{
		acceleration_[i] -= up_vec.v[i] * 9.81f;
	}

	return success;
}


const float& Accelerometer_sim::last_update_us(void) const
{
	return dynamic_model_.last_update_us();
}


const std::array<float, 3>& Accelerometer_sim::acc(void) const
{
	return acceleration_;
}


const float& Accelerometer_sim::acc_X(void) const
{
	return acceleration_[X];
}


const float& Accelerometer_sim::acc_Y(void) const
{
	return acceleration_[Y];
}


const float& Accelerometer_sim::acc_Z(void) const
{
	return acceleration_[Z];
}


const float& Accelerometer_sim::temperature(void) const
{
	return temperature_;
}