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
 * \file adxl345_driver.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the Driver for the ADXL345 accelerometer 
 *
 ******************************************************************************/


#include "adxl345_driver.h"
#include "i2c_driver_int.h"
#include "print_util.h"
#include "twim.h"

static volatile acc_data_t acc_outputs;				///< Declare an object containing accelerometer's data

#define CONFIG_POWER_ADDRESS 0x2D					///< Address of the power configuration register

#define SENSOR_REG_ADDRESS 0x32						///< Address of the accelerometer register
#define DATA_SETTING_ADDRESS 0x31					///< Address of the data setting register
enum {RANGE_2G, RANGE_4G, RANGE_8G, RANGE_16G};		///< Define the different range in which you could use the accelerometer
#define FULL_RES 0b1000								///< Define the full resolution for the output of the accelerometer

uint8_t default_configuration[2] ={CONFIG_POWER_ADDRESS, 8};	///< default configuration of the accelerometer

uint8_t data_configuration[2] ={DATA_SETTING_ADDRESS, FULL_RES | RANGE_16G};	///< configuration of the output data

void adxl345_driver_init_slow(void) 
{
	static twim_options_t twi_opt = 
	{
		.pba_hz = 64000000, 
		.speed	= 400000,
		.chip	= ADXL_ALT_SLAVE_ADDRESS, 
		.smbus	= false
	};

	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_write(&AVR32_TWIM0, (uint8_t*)&data_configuration, 2, ADXL_ALT_SLAVE_ADDRESS, false);
}

acc_data_t* adxl345_driver_get_acc_data_slow(void) 
{
	int32_t i;
	uint8_t write_then_read_preamble=SENSOR_REG_ADDRESS;
	
	twim_write(&AVR32_TWIM0, (uint8_t*)&write_then_read_preamble, 1, ADXL_ALT_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&acc_outputs, 6, ADXL_ALT_SLAVE_ADDRESS, false);
	
	for (i = 0; i < 3; i++) 
	{
		acc_outputs.axes[i] = (int16_t)(acc_outputs.raw_data[2 * i]) + (int16_t)(acc_outputs.raw_data[2 * i + 1] << 8);
	}
				
	return (acc_data_t*)&acc_outputs;
}