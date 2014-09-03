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
 * \file itg3200.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the integrated triple axis gyroscope ITG3200
 *
 ******************************************************************************/


#include "itg3200.h"
#include "i2c_driver_int.h"
#include "print_util.h"

#define CONFIG_REG_ADDRESS 21				///< Define the Configuration register address
#define SENSOR_REG_ADDRESS 27				///< Define the Address of the gyroscope sensor as a slave on the i2c bus

#define FULL_SCALE_MASK (_BV(3)|_BV(4))		///< Define the resolution of the sensor
#define DLPF_256HZ 0						///< Define the frequency loop
#define DLPF_188HZ 1						///< Define the frequency loop
#define DLPF_98HZ 2							///< Define the frequency loop
#define DLPF_42HZ 3							///< Define the frequency loop
#define DLPF_20HZ 4							///< Define the frequency loop
#define DLPF_10HZ 5							///< Define the frequency loop
#define DLPF_5HZ 6							///< Define the frequency loop

/**
 * \brief Structure containing the Configuration of the gyroscope
*/
typedef struct 
{
	uint8_t conf_start_reg_address;			///< Define the address of the configuration register
	uint8_t sample_div;						///< Define the sampling divider
	uint8_t DLPF;							///< Define the Derivative (?) part of the Low Pass Filter
	uint8_t interrupts;						///< Define the interruption
} gyro_config;

static volatile gyroscope_t gyro_outputs;		///< Create an object containing the gyroscope's data
gyro_config default_configuration;			///< Declare the object containing the gyroscope configuration structure
uint8_t read_preamble=SENSOR_REG_ADDRESS;	///< Declare the address of the sensor

void itg3200_init_slow(void) 
{
	static twim_options_t twi_opt= 
	{
		.pba_hz = 64000000, 
		.speed = 400000,
		.chip = ITG3200_SLAVE_ADDRESS, 
		.smbus = false
	};
	twim_master_init(&AVR32_TWIM0, &twi_opt);
	twim_write(&AVR32_TWIM0, (uint8_t*)&default_configuration, 4, ITG3200_SLAVE_ADDRESS, false);
}

gyroscope_t* itg3200_get_data_slow(void) 
{
	uint8_t write_then_read_preamble=SENSOR_REG_ADDRESS;
	
	twim_write(&AVR32_TWIM0, (uint8_t*) &write_then_read_preamble, 1, ITG3200_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)&gyro_outputs, 8, ITG3200_SLAVE_ADDRESS, false);
	
	return (gyroscope_t*)&gyro_outputs;
}