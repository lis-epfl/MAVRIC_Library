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
 * \file airspeed_i2cxl.c
 * 
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *   
 * \brief Driver for the airspeed module using i2C communication protocol
 *
 ******************************************************************************/


#include "airspeed_i2cxl.h"
#include "twim.h"
#include "print_util.h"
#include "time_keeper.h"
#include <math.h>

const uint8_t AIRSPEED_I2CXL_DEFAULT_ADDRESS			= 0x46;		///< Address of the device
const uint8_t AIRSPEED_I2CXL_RANGE_COMMAND				= 0x8D;		///< Address of the Range Command Register
const uint8_t AIRSPEED_I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t AIRSPEED_I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Send range Command for the airspeed_i2cxl
 *
 * \param	airspeed_i2cxl	Pointer to an object containing the airspeed_i2cxl's data
 */
void airspeed_i2cxl_send_range_command(airspeed_i2cxl_t* airspeed_i2cxl);


/**
 * \brief Get the last measurement
 *
 * \param	airspeed_i2cxl	Pointer to an object containing the airspeed_i2cxl's data
 */
void airspeed_i2cxl_get_last_measure(airspeed_i2cxl_t* airspeed_i2cxl);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void airspeed_i2cxl_send_range_command(airspeed_i2cxl_t* airspeed_i2cxl)
{
	uint8_t buff = AIRSPEED_I2CXL_RANGE_COMMAND;
	twim_write(&AVR32_TWIM1, &buff, 1, airspeed_i2cxl->i2c_address, false);
}


void airspeed_i2cxl_get_last_measure(airspeed_i2cxl_t* airspeed_i2cxl)
{
	// Declare buffer, sensor values, and time
	uint8_t buf[2];
	uint16_t airspeed_raw = 0;
	float airspeed_mps = 0.0f;
	float dt = 0.0f;
	uint32_t time_us = time_keeper_get_micros();

	// Read sensor through I2C
	twim_read(&AVR32_TWIM1, buf, 2, airspeed_i2cxl->i2c_address, false);
	
	// Convert data to one variable
	/*
	* First to bits are 00 if there were no errors, 01 which is reserved, 10 if the data had been recorded already,
	* 11 if there was a fault detected. As 00 is the only acceptable one, if there was an error, the recorded value
	* will be greater than the maximum. Discard all that are greater than the maximum.
	*/
	airspeed_raw = (buf[0] << 8) + buf[1];
	
	// Convert to meters per second from psi
	airspeed_mps  = 106.098f * sqrt((float)airspeed_raw);
	
	// Discard all that are below minimum or greater than maximum
	if ( airspeed_mps > airspeed_i2cxl->data.min_velocity && airspeed_mps < airspeed_i2cxl->data.max_velocity )
	{
		// Update time
		dt = ((float)time_us - airspeed_i2cxl->data.last_update)/1000000.0f;
		
		// Update velocities
		airspeed_i2cxl->data.current_velocity  = airspeed_mps;
		airspeed_i2cxl->data.last_update = time_us;
		airspeed_i2cxl->data.healthy = true;
	}
	else // set health to 0
	{
		airspeed_i2cxl->data.current_velocity = 0.0f;
		airspeed_i2cxl->data.healthy = false;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool airspeed_i2cxl_init(airspeed_i2cxl_t* airspeed_i2cxl)
{
	bool init_success = true;
	
	///< Init data_struct
	airspeed_i2cxl->i2c_address = AIRSPEED_I2CXL_DEFAULT_ADDRESS;
	airspeed_i2cxl->data.current_velocity	= 0.0f;
	airspeed_i2cxl->data.orientation.s		= 1.0f;
	airspeed_i2cxl->data.orientation.v[0] 	= 0.0f;
	airspeed_i2cxl->data.orientation.v[1] 	= 0.0f;
	airspeed_i2cxl->data.orientation.v[2] 	= 0.0f;
	
	airspeed_i2cxl->data.min_velocity  		= 0.0f;
	airspeed_i2cxl->data.max_velocity  	= 106.098f;
	airspeed_i2cxl->data.healthy 				= false;
	
	///< Init I2C bus
	static twi_options_t twi_opt = 
	{
		.pba_hz = 64000000,
		.speed  = 100000,
		.chip   = 1,
		.smbus  = false
	};

	twi_master_init(&AVR32_TWIM1, &twi_opt);
	print_util_dbg_print("[AIRSPEED_I2CXL] Initialised\r\n");
	
	return init_success;
}


task_return_t airspeed_i2cxl_update(airspeed_i2cxl_t* airspeed_i2cxl)
{
	airspeed_i2cxl_get_last_measure(airspeed_i2cxl);
	airspeed_i2cxl_send_range_command(airspeed_i2cxl);
	
	return TASK_RUN_SUCCESS;
}