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
 * \file dynamic_model.hpp 
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Abstract class for simulation's dynamic model
 *
 ******************************************************************************/


#ifndef DYNAMIC_MODEL_HPP_
#define DYNAMIC_MODEL_HPP_


#include <array>
#include "sensing/imu.hpp"

extern "C"
{
	#include "util/coord_conventions.h"
}

/**
 * \brief 	Abstract class for simulation's dynamic model
 */
class Dynamic_model
{
public:
	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	virtual bool update(void) = 0;

	
	/**
	 * \brief 	Get last update time in microseconds
	 * 
	 * \return 	Update time
	 */
	virtual const float& last_update_us(void) const = 0;


	/**
	 * \brief 	Get X, Y and Z components of acceleration in body frame in m/s^2
	 * 
	 * \return 	Value
	 */	
	virtual const std::array<float, 3>& acceleration_bf(void) const = 0;

	
	/**
	 * \brief 	Get X, Y and Z components of velocity in local frame
	 * 
	 * \return 	Value
	 */	
	virtual const std::array<float, 3>& velocity_lf(void) const = 0;


	/**
	 * \brief 	Get X, Y and Z position in local frame (centered on home)
	 * 
	 * \return 	Value
	 */	
	virtual const local_position_t& position_lf(void) const = 0;


	/**
	 * \brief 	Get X, Y and Z position in global frame
	 * 
	 * \return 	Value
	 */	
	virtual const global_position_t& position_gf(void) const = 0;


	/**
	 * \brief 	Get X, Y and Z components of angular velocity in body frame
	 * 
	 * \return 	Value
	 */	
	virtual const std::array<float, 3>& angular_velocity_bf(void) const = 0;


	/**
	 * \brief 	Get attitude quaternion
	 * 
	 * \return 	Value
	 */	
	virtual const quat_t& attitude(void) const = 0;
};


#endif /* DYNAMIC_MODEL_HPP_ */