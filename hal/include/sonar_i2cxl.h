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

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "sonar.h"
#include "tasks.h"
/**
 * \brief structure of the sonar_i2cxl module
*/
typedef struct 
{
	uint8_t i2c_address;	///< address of the sonar module
	sonar_t data;			///< sensor data	
} sonar_i2cxl_t;

/**
 * \brief Initializes the I2CXL sonar data struct and the i2c bus
 * 
 * \param	sonar	Pointer to the sonar Data structure
 * 
 * \return	True if the init succeed, false otherwise
 */
bool sonar_i2cxl_init(sonar_i2cxl_t* sonar);

/**
 * \brief Reads last value from sensor and start new recording
 * \details This function should be called at a frequency lower than 10Hz
 * 
 * \param	sonar	Data struct
 * 
 * \return	The result of the task execution
 */
task_return_t sonar_i2cxl_update(sonar_i2cxl_t* sonar);


#ifdef __cplusplus
	}
#endif

#endif /* I2CXL_SONAR_H */