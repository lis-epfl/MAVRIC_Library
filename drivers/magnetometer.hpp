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
 * \file magnetometer.hpp
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 * \author Julien Lecoeur
 *   
 * \brief Abstract class for magnetometers
 * 
 ******************************************************************************/


#ifndef MAGNETOMETER_HPP_
#define MAGNETOMETER_HPP_


#include <array>


/**
 * \brief Abstract class for magnetometers
 */
class Magnetometer
{
public:
	/**
	 * \brief   Initialise the sensor
	 * 			
	 * \return 	Success
	 */	
	virtual bool init(void) = 0;


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
	 * \brief 	Get X, Y and Z components of magnetic field
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	virtual const std::array<float, 3>& mag(void) const = 0;


	/**
	 * \brief 	Get X component of magnetic field
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	virtual const float& mag_X(void) const = 0;


	/**
	 * \brief 	Get Y component of magnetic field
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	virtual const float& mag_Y(void) const = 0;


	/**
	 * \brief 	Get Z component of magnetic field
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	virtual const float& mag_Z(void) const = 0;


	/**
	 * \brief 	Get sensor temperature
	 * 
	 * \return 	Value
	 */	
	virtual const float& temperature(void) const = 0;
};


#endif /* MAGNETOMETER_HPP_ */