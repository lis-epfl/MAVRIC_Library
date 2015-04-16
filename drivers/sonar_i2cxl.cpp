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
 * \file sonar_i2cxl.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#include "sonar_i2cxl.h"

extern "C"
{
	#include "twim.h"
	#include "print_util.h"
	#include "time_keeper.h"
}

const uint8_t SONAR_I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

sonar_i2cxl::sonar_i2cxl(i2c& i2c, uint8_t address):
	_i2c(i2c),
	_i2c_address(address)
{
	data.current_distance 	= 0.2f;
	data.orientation.s 	= 1.0f;
	data.orientation.v[0] 	= 0.0f;
	data.orientation.v[0] 	= 0.0f;
	data.orientation.v[0] 	= 0.0f;
	data.min_distance      = 0.22f;
	data.max_distance      = 5.0f;
	data.covariance 	    = 0.01f;
	data.healthy 	        = false;
}

bool sonar_i2cxl::update(void)
{
	bool res = true;

	res &= get_last_measure();
	res &= send_range_command();

	return res;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool sonar_i2cxl::send_range_command(void)
{
	bool res;

	uint8_t buff = SONAR_I2CXL_RANGE_COMMAND;
	res = _i2c.write(&buff, 1, _i2c_address);

	return res;
}


bool sonar_i2cxl::get_last_measure(void)
{
	bool res;
	uint8_t buf[2];
	uint16_t distance_cm = 0;
	float distance_m = 0.0f;
	uint32_t time_us = time_keeper_get_micros();

	res = _i2c.read(buf, 2, _i2c_address);

	distance_cm = (buf[0] << 8) + buf[1];	
	distance_m  = ((float)distance_cm) / 100.0f;
	
	if ( distance_m > data.min_distance && distance_m < data.max_distance )
	{
		data.current_distance  = distance_m;
		data.last_update = time_us;
		data.healthy = true;
	}
	else
	{
		data.healthy = false;
	}

	return res;
}


//------------------------------------------------------------------------------
// GLUE FUNCTION (TEMPORARY)
//------------------------------------------------------------------------------
task_return_t sonar_i2cxl_update(sonar_i2cxl* sonar)
{
	sonar->update();

	return TASK_RUN_SUCCESS;
}