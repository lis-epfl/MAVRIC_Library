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
 * \file hmc5883l.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the magnetometer HMC58831
 *
 ******************************************************************************/


#include "hmc5883l.h"
#include "twim.h"
#include "print_util.h"

#define CONF_REG_A 0x00					///< Configuration Register A
#define CONF_REG_B 0x01					///< Configuration Register B
#define MODE_REG 0x02					///< Mode register
#define DATA_REG_BEGIN 0x03				///< Data Register Begin Command

#define MEASUREMENT_CONTINUOUS 0x00		///< Continuous measurement Mode
#define MEASUREMENT_SINGLE_SHOT 0x01		///< Single Shot measurement Mode
#define MEASUREMENT_IDLE 0x03			///< Idle Mode

#define HMC5883_SLAVE_ADDRESS 0x1E		///< HMC5883L


#define HMC_SAMPLE_AVG	HMC_SAMPLE_AVG_4		///< Define the sampling average mode used
#define HMC_RATE		HMC_RATE_15				///< Define the sampling rate mode used
#define HMC_BIAS_MODE	HMC_BIAS_MODE_NORMAL	///< Define the sampling bias mode used
#define HMC_RANGE		HMC_RANGE_1_3_GA		///< Define the sampling range mode used
#define HMC_MODE		HMC_MODE_CONTINUOUS		///< Define the sampling mode used


/**
 * \brief Defines 4 states for the sampling average
*/
enum 
{
	HMC_SAMPLE_AVG_1, 
	HMC_SAMPLE_AVG_2,
	HMC_SAMPLE_AVG_4, 
	HMC_SAMPLE_AVG_8
}; 


/**
 * \brief Defines 7 states for the sampling rate
*/
enum 
{
	HMC_RATE_0_75,
	HMC_RATE_1_5,
	HMC_RATE_3_0, 
	HMC_RATE_7_5, 
	HMC_RATE_15, 
	HMC_RATE_30, 
	HMC_RATE_75
};


/**
 * \brief Defines 3 states for the sampling bias mode
*/
enum
{
	HMC_BIAS_MODE_NORMAL, 
	HMC_BIAS_MODE_POS_BIAS, 
	HMC_BIAS_MODE_NEG_BIAS
};


/**
 * \brief Defines 8 states for the sampling range
*/
enum
{
	HMC_RANGE_0_88_GA,
	HMC_RANGE_1_3_GA,
	HMC_RANGE_1_9_GA,
	HMC_RANGE_2_5_GA,
	HMC_RANGE_4_0_GA,
	HMC_RANGE_4_7_GA,
	HMC_RANGE_5_6_GA,
	HMC_RANGE_8_1_GA
};


/**
 * \brief Defines 3 states for the sampling mode
*/
enum 
{	
	HMC_MODE_CONTINUOUS, 
	HMC_MODE_SINGLE, 
	HMC_MODE_IDLE
};


void hmc5883l_init_slow() 
{
	if(twim_probe(&AVR32_TWIM0, HMC5883_SLAVE_ADDRESS) == STATUS_OK)
	{
		print_util_dbg_print("HMC5883 compass sensor found (0x1E) \r\n");
	}
	else
	{
		print_util_dbg_print("HMC5883 compass sensor not responding (0x1E) \r\n");
		return;
	}
	
	static uint8_t compass_default_configuration[4] =
	{
		CONF_REG_A,
		(HMC_SAMPLE_AVG) << 5 | (HMC_RATE) << 2 | HMC_MODE,
		(HMC_RANGE) << 5,
		(HMC_MODE)
	};
	
	twim_write(&AVR32_TWIM0, (uint8_t*)&compass_default_configuration, 4, HMC5883_SLAVE_ADDRESS, false);
}


void hmc5883l_update(magnetometer_t *compass_outputs) 
{
	uint8_t start_address = DATA_REG_BEGIN;
	uint16_t data[3];
		
	twim_write(&AVR32_TWIM0, (uint8_t*) &start_address, 1, HMC5883_SLAVE_ADDRESS, false);
	twim_read(&AVR32_TWIM0, (uint8_t*)(data), 6, HMC5883_SLAVE_ADDRESS, false);
	
	compass_outputs->data[0] = (float)((int16_t)data[0]);
	compass_outputs->data[1] = (float)((int16_t)data[1]);
	compass_outputs->data[2] = (float)((int16_t)data[2]);
}