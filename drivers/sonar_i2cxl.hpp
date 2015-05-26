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

#include "sonar.h"
#include "i2c.hpp"

extern "C" {
	#include <stdint.h>
	#include "scheduler.h"
}

const uint8_t SONAR_I2CXL_DEFAULT_ADDRESS = 0x70;		///< Address of the device

/**
 * \brief structure of the sonar_i2cxl module
*/
typedef struct 
{
	uint8_t i2c_address;	///< address of the sonar module
	sonar_t data;			///< sensor data	
} sonar_i2cxl_t;


class Sonar_i2cxl
{
public:
	/**
	 * @brief   Constructor
	 * 
	 * @param 	i2c 		Reference to I2C device
	 * @param 	address 	Address of the sonar on the I2C bus
	 */
	Sonar_i2cxl(I2c& i2c, uint8_t address = SONAR_I2CXL_DEFAULT_ADDRESS);


	/**
	 * @brief 	Reads last value from sensor and start new recording
	 * 
	 * @return 	true 	Success
	 * @return 	false 	Failed
	 */
	bool update(void);


	/**
	 * Public members
	 * 
	 */
	sonar_t data;		///< Temporary, TODO: replace by inheritence to abstract class sonar (not yet implemented)

private:
	I2c& 	i2c_;
	uint8_t i2c_address_;


	/**
	 * @brief	Send range Command for the Sonar_i2cxl
	 * 
	 * @return 	true 	Success
	 * @return 	false 	Failed
	 */
	bool send_range_command(void);


	/**
	 * @brief  	Get the last measurement
	 * 
	 * @return 	true 	Success
	 * @return 	false 	Failed
	 */
	bool get_last_measure(void);	
};


/**
 * \brief 	Compatibility function for scheduler
 * 
 * \details Can be removed once the scheduler accepts runnable objects (ie from a class with an update() fonction)
 * 
 * \param	sonar	Object
 * 
 * \return	The result of the task execution
 */
task_return_t sonar_i2cxl_update(Sonar_i2cxl* sonar);


#endif /* I2CXL_SONAR_H */