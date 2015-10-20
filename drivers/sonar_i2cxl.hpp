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
 * \file sonar_i2cxl.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#ifndef I2CXL_SONAR_H_
#define I2CXL_SONAR_H_

#include "sonar.hpp"
#include "i2c.hpp"

extern "C" {
	#include <stdint.h>
	#include "scheduler.h"
}


/**
 * \brief 	Configuration structure for Sonar_i2cxl
 */
typedef struct
{
	uint8_t 			i2c_address; 		///< I2C address
	float 				min_distance;		///< Minimum distance the sensor can read
	float 				max_distance;		///< Maximum distance the sensor can read
	std::array<float,3>	orientation_bf;		///< Sensor orientation relative to the body frame
} sonar_i2cxl_conf_t;


/**
 * \brief 	Default configuration
 * 
 * \return 	Config structure
 */
static inline sonar_i2cxl_conf_t sonar_i2cxl_default_config();


class Sonar_i2cxl: public Sonar
{
public:
	/**
	 * @brief   Constructor
	 * 
	 * @param 	i2c 		Reference to I2C device
	 * @param 	config 		Configuration
	 */
	Sonar_i2cxl(I2c& i2c, sonar_i2cxl_conf_t config = sonar_i2cxl_default_config() );


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
	 * \brief 	Get sensor orientation relative to the platform (in body frame)
	 * 
	 * \return 	quaternion
	 */
	const std::array<float,3>& orientation_bf(void) const;


	/**
	 * \brief 	Get latest distance measure
	 * 
	 * \return 	Value
	 */
	const float& distance(void) const;


	/**
	 * \brief 	Get velocity estimate from consecutive measurements
	 * 
	 * \return 	Value
	 */
	const float& velocity(void) const;


	/**
	 * \brief 	Indicates whether the measurements can be trusted 
	 * 
	 * \return 	Value
	 */
	const bool& healthy(void) const;


private:
	I2c& 				i2c_;			 ///< Reference to I2C peripheral

	sonar_i2cxl_conf_t 	config_;		 ///< Configuration

	float 				distance_;		 ///< Current distance
	float 				velocity_;		 ///< Current velocity
	bool 				healthy_;		 ///< Sensor status
	float 				last_update_us_; ///< Last update time in microseconds


	/**
	 * \brief	Send range Command for the Sonar_i2cxl
	 * 
	 * \return 	true 	Success
	 * \return 	false 	Failed
	 */
	bool send_range_command(void);


	/**
	 * \brief  	Get the last measurement
	 * 
	 * \return 	true 	Success
	 * \return 	false 	Failed
	 */
	bool get_last_measure(void);	
};


/**
 * \brief 	Compatibility function for scheduler
 * 
 * \details Can be removed once the scheduler accepts runnable objects 
 * 			(ie from a class with an update() fonction)
 * 
 * \param	sonar		Object
 * 
 * \return	Success		The result of the task execution
 */
bool sonar_i2cxl_update(Sonar_i2cxl* sonar);


/**
 * \brief 	Default configuration
 * 
 * \return 	Config structure
 */
static inline sonar_i2cxl_conf_t sonar_i2cxl_default_config()
{
	sonar_i2cxl_conf_t conf = {};

	conf.i2c_address  = 0x70;

	// Correct range between 20cm and 7m, - safety
	conf.min_distance = 0.22f;
	conf.max_distance = 5.0f;

	// Default orientation is looking downwards (NED)
	conf.orientation_bf = {0.0f, 0.0f, 1.0f}; 

	return conf;
}


#endif /* I2CXL_SONAR_H */