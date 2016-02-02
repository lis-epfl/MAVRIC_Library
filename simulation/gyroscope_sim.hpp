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
 * \file gyroscope_sim.hpp
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Simulated gyroscopes
 * 
 ******************************************************************************/


#ifndef GYROSCOPE_SIM_HPP_
#define GYROSCOPE_SIM_HPP_


#include <array>

#include "simulation/dynamic_model.hpp"

/**
 * \brief 	Simulated gyroscopes
 */
class Gyroscope_sim: public Gyroscope
{
public:
	/**
	 * \brief 	Constructor
	 * 
	 * \param 	dynamic_model 	Reference to dynamic model
	 */
	Gyroscope_sim(Dynamic_model& dynamic_model);


	/**
	 * \brief   Initialise the sensor
	 * 			
	 * \return 	Success
	 */	
	bool init(void);


	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	bool update(void);

	
	/**
	 * \brief 	Get last update time in microseconds
	 * 
	 * \return 	Update time
	 */
	const float& last_update_us(void) const;


	/**
	 * \brief 	Get X, Y and Z components of angular velocity
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const std::array<float, 3>& gyro(void) const;


	/**
	 * \brief 	Get X component of angular velocity
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& gyro_X(void) const;


	/**
	 * \brief 	Get Y component of angular velocity
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& gyro_Y(void) const;


	/**
	 * \brief 	Get Z component of angular velocity
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& gyro_Z(void) const;


	/**
	 * \brief 	Get sensor temperature
	 * 
	 * \return 	Value
	 */	
	const float& temperature(void) const;


private:
	Dynamic_model& 		dynamic_model_;		///< Reference to dynamic model

	std::array<float,3> rates_;				///< Angular velocity in body frame
	float 				temperature_;		///< Temperature: NOT IMPLEMENTED
};


#endif /* GYROSCOPE_SIM_HPP_ */