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
 * \file lsm330dlc.hpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Geraud L'Eplattenier
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the integrated 3axis gyroscope and 
 * accelerometer LSM330DLC
 * 
 ******************************************************************************/


#ifndef LSM330DLC_H_
#define LSM330DLC_H_

#include <stdint.h>
#include <array>
#include "accelerometer.hpp"		
#include "gyroscope.hpp"
#include "i2c.hpp"


/**
 * \brief 		Driver for sensor LSM330DLC
 * 
 * \details 	This sensor is at the same time a accelerometer and a gyroscope
 */
class Lsm330dlc: public Accelerometer, public Gyroscope
{
public:
	/**
	 * \brief  	Constructor
	 * 
	 * \param 	i2c 	Reference to I2C device 
	 */
	Lsm330dlc(I2c& i2c);


	/**
	 * \brief   Initialise the sensor
	 * \details Sends configuration via I2C, the I2C peripheral must be 
	 * 			activated before this method is called
	 * 			
	 * \return 	true 	Success
	 * \return 	false 	Failed
	 */	
	bool init(void);


	/**
	 * \brief   Main update function
	 * \details Get new data from the sensor
	 * 
	 * \return 	true 	Success
	 * \return 	false 	Failed
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
	 * \brief 	Get X, Y and Z components of acceleration
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const std::array<float, 3>& acc(void) const;


	/**
	 * \brief 	Get X component of acceleration
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& acc_X(void) const;


	/**
	 * \brief 	Get Y component of acceleration
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& acc_Y(void) const;


	/**
	 * \brief 	Get Z component of acceleration
	 * 
	 * \detail 	This is raw data, so X, Y and Z components are biased, not scaled,
	 * 			and given in the sensor frame (not in the UAV frame). 
	 * 			Use an Imu object to handle bias removal, scaling and axis rotations
	 * 
	 * \return 	Value
	 */	
	const float& acc_Z(void) const;


	/**
	 * \brief 	Get sensor temperature
	 * 
	 * \return 	Value
	 */	
	const float& temperature(void) const;


private:
	I2c& 				 i2c_;				///< I2C peripheral
	std::array<float, 3> gyro_data_;		///< Gyroscope data
	std::array<float, 3> acc_data_;			///< Accelerometer data
	float 				 temperature_;		///< Temperature
	float 				 last_update_us_; 	///< Last udate time in microseconds
};

#endif 