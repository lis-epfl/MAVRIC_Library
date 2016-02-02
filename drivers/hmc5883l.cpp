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
 * \file hmc5883l.cpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief 	Driver for the magnetometer HMC58831
 * 
 * \detail 	This driver does not support temperature
 *
 ******************************************************************************/


#include "hmc5883l.hpp"

extern "C"
{
	#include "print_util.h"
	#include "time_keeper.hpp"
}


const uint8_t CONF_REG_A              = 0x00;					///< Configuration Register A
const uint8_t CONF_REG_B              = 0x01;					///< Configuration Register B
const uint8_t MODE_REG                = 0x02;					///< Mode register
const uint8_t DATA_REG_BEGIN          = 0x03;				///< Data Register Begin Command

const uint8_t MEASUREMENT_CONTINUOUS  = 0x00;		///< Continuous measurement Mode
const uint8_t MEASUREMENT_SINGLE_SHOT = 0x01;		///< Single Shot measurement Mode
const uint8_t MEASUREMENT_IDLE        = 0x03;			///< Idle Mode

const uint8_t HMC5883_SLAVE_ADDRESS   = 0x1E;		///< HMC5883L


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


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Hmc5883l::Hmc5883l(I2c& i2c):
	i2c_(i2c),
	data_( std::array<float,3>{{0.0f, 0.0f, 0.0f}} ),
	last_update_us_(0.0f),
	temperature_(0.0f)
{}


bool Hmc5883l::init(void)
{
	bool success = true;

	// test if the sensor if here
	success &= i2c_.probe(HMC5883_SLAVE_ADDRESS);
	
	static uint8_t compass_default_configuration[4] =
	{
		CONF_REG_A,
		(HMC_SAMPLE_AVG) << 5 | (HMC_RATE) << 2 | HMC_MODE,
		(HMC_RANGE) << 5,
		(HMC_MODE)
	};
	
	// Write configuration to the sensor
	success &= i2c_.write(compass_default_configuration, 4, HMC5883_SLAVE_ADDRESS);

	return success;
}


bool Hmc5883l::update(void) 
{
	bool success		= true;	
	uint16_t buffer[3] 	= {0, 0, 0};

	// Read data from sensor
 	success &= i2c_.write( &DATA_REG_BEGIN, 1, HMC5883_SLAVE_ADDRESS );
	success &= i2c_.read( (uint8_t*)buffer, 6, HMC5883_SLAVE_ADDRESS );
	
	// Copy to member data
	data_[0] = (float)((int16_t)buffer[0]);
	data_[1] = (float)((int16_t)buffer[1]);
	data_[2] = (float)((int16_t)buffer[2]);

	// Save last update time
	last_update_us_ = time_keeper_get_us();

	return success;
}


const float& Hmc5883l::last_update_us(void) const
{
	return last_update_us_;
}


const std::array<float, 3>& Hmc5883l::mag(void) const
{
	return data_;
}


const float& Hmc5883l::mag_X(void) const
{
	return data_[0];
}


const float& Hmc5883l::mag_Y(void) const
{
	return data_[1];
}


const float& Hmc5883l::mag_Z(void) const
{
	return data_[2];
}


const float& Hmc5883l::temperature(void) const
{
	return temperature_;
}