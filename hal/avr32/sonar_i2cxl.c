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
#include "twim.h"
#include "print_util.h"
#include "time_keeper.h"

const uint8_t SONAR_I2CXL_DEFAULT_ADDRESS			= 0x70;		///< Address of the device
const uint8_t SONAR_I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Send range Command for the sonar_i2cxl
 *
 * \param	sonar	Pointer to an object containing the sonar_i2cxl's data
 */
void sonar_i2cxl_send_range_command(sonar_i2cxl_t* sonar);


/**
 * \brief Get the last measurement
 *
 * \param	sonar	Pointer to an object containing the sonar_i2cxl's data
 */
void sonar_i2cxl_get_last_measure(sonar_i2cxl_t* sonar);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void sonar_i2cxl_send_range_command(sonar_i2cxl_t* sonar_i2cxl)
{
	uint8_t buff = SONAR_I2CXL_RANGE_COMMAND;
	twim_write(&AVR32_TWIM1, &buff, 1, sonar_i2cxl->i2c_address, false);
}


void sonar_i2cxl_get_last_measure(sonar_i2cxl_t* sonar_i2cxl)
{
	uint8_t buf[2];
	uint16_t distance_cm = 0;
	float distance_m = 0.0f;
	uint32_t time_us = time_keeper_get_micros();

	twim_read(&AVR32_TWIM1, buf, 2, sonar_i2cxl->i2c_address, false);
	distance_cm = (buf[0] << 8) + buf[1];
	
	distance_m  = ((float)distance_cm) / 100.0f;
	
	if ( distance_m > sonar_i2cxl->data.min_distance && distance_m < sonar_i2cxl->data.max_distance )
	{
		sonar_i2cxl->data.current_distance  = distance_m;
		sonar_i2cxl->data.last_update = time_us;
		sonar_i2cxl->data.healthy = true;
	}
	else
	{
		sonar_i2cxl->data.healthy = false;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool sonar_i2cxl_init(sonar_i2cxl_t* sonar_i2cxl)
{
	bool init_success = true;
	
	///< Init data_struct
	sonar_i2cxl->i2c_address = SONAR_I2CXL_DEFAULT_ADDRESS;
	sonar_i2cxl->data.current_distance 	= 0.2f;
	sonar_i2cxl->data.orientation.s 	= 1.0f;
	sonar_i2cxl->data.orientation.v[0] 	= 0.0f;
	sonar_i2cxl->data.orientation.v[0] 	= 0.0f;
	sonar_i2cxl->data.orientation.v[0] 	= 0.0f;
	
	sonar_i2cxl->data.min_distance  = 0.22f;
	sonar_i2cxl->data.max_distance  = 5.0f;
	sonar_i2cxl->data.covariance 	= 0.01f;
	sonar_i2cxl->data.healthy 	= false;
	
	///< Init I2C bus
	static twi_options_t twi_opt = 
	{
		.pba_hz = 64000000,
		.speed  = 100000,
		.chip   = 1,
		.smbus  = false
	};

	twi_master_init(&AVR32_TWIM1, &twi_opt);
	print_util_dbg_print("[SONAR_I2CXL] Initialised\r\n");
	
	return init_success;
}


task_return_t sonar_i2cxl_update(sonar_i2cxl_t* sonar_i2cxl)
{
	sonar_i2cxl_get_last_measure(sonar_i2cxl);
	sonar_i2cxl_send_range_command(sonar_i2cxl);
	
	return TASK_RUN_SUCCESS;
}
