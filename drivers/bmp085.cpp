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
 * \file bmp085.cpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief This file is the driver for the barometer module: BMP085
 * 
 ******************************************************************************/


#include "bmp085.hpp"

extern "C"
{
	#include "twim.h"
	#include "maths.h"
	#include "time_keeper.h"
	#include "print_util.h"
}

enum
{
	BMP085_ULTRALOWPOWER,
	BMP085_STANDARD, 	
	BMP085_HIGHRES,		
	BMP085_ULTRAHIGHRES 
};

const uint8_t BARO_ALT_LPF 				= 0.95f;		///< low pass filter factor for altitude measured by the barometer
const uint8_t VARIO_LPF 				= 0.95f;		///< low pass filter factor for the Vario altitude speed

const uint8_t BMP085_SLAVE_ADDRESS 		= 0x77;			///< Address of the barometer sensor on the i2c bus
 
const uint8_t BMP085_CAL_AC1 			= 0xAA;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC2 			= 0xAC;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC3 			= 0xAE;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC4 			= 0xB0;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC5 			= 0xB2;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC6 			= 0xB4;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_B1 			= 0xB6;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_B2 			= 0xB8;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MB 			= 0xBA;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MC 			= 0xBC;			///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MD 			= 0xBE;			///< R Calibration data (16 bits)

const uint8_t BMP085_CONTROL 			= 0xF4;			///< Control register of the barometer sensor 
const uint8_t BMP085_TEMPDATA 			= 0xF6;			///< Temperature register of the barometer sensor 
const uint8_t BMP085_PRESSUREDATA 		= 0xF6;			///< Pressure Data register of the barometer sensor 
const uint8_t BMP085_READTEMPCMD 		= 0x2E;			///< Read temperature Command register of the barometer sensor 
const uint8_t BMP085_READPRESSURECMD	= 0x34;			///< Read Pressure Command register of the barometer sensor 

#define BMP085_OVERSAMPLING_MODE BMP085_HIGHRES			///< Set oversampling mode of the barometer sensor to high resolution mode


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------
Bmp085::Bmp085(	I2c& i2c, barometer_t& data):
		i2c_(i2c),
		data_(data)
{}


bool Bmp085::init(void)
{
	bool res = true;

	// test if the sensor if here
	res &= i2c_.probe(BMP085_SLAVE_ADDRESS);
	
	data_.altitude_offset = 0.0f;

	for (int32_t i = 0; i < 3; i++) 
	{
		data_.last_altitudes[i] = 0.0f;
	}
	
	data_.vario_vz = 0.0f;

	///< Configure with datasheet values
	ac1_ = 408;
	ac2_ = -72;
	ac3_ = -14383;
	ac4_ = 32741;
	ac5_ = 32757;
	ac6_ = 23153;
	//mb_  = 1; //TODO check value in datasheet
	mc_ = -8711;
	md_ = 2868;
	b1_ = 6190;
	b2_ = 4;

	//TODO Reading is not working as it should
/*	///< Address of those configuration values
	uint8_t address[11] = {0xAA, 0xAC, 0xAE, 0xB0, 0xB2, 0xB4, 0xB6, 0xB8, 0xBA, 0xBC, 0xBE};

	///< Try to update with Eprom values of the sensor
	i2c_.write((uint8_t*)&(address[0]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac1_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[1]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac2_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[2]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac3_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[3]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac4_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[4]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac5_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[5]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&ac6_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[6]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&mb_, 2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[6]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&mc_ ,  2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[7]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&md_ ,  2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[8]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&b1_ ,  2, BMP085_SLAVE_ADDRESS );
	i2c_.write((uint8_t*)&(address[9]), 1, BMP085_SLAVE_ADDRESS);
	i2c_.read( (uint8_t*)&b2_ ,  2, BMP085_SLAVE_ADDRESS );
*/
	//reset Bmp085 state
	data_.state = IDLE;

	return res;
}


bool Bmp085::update(void) 
{
	bool res 			= true;	
	
	float altitude, vertical_speed;
	int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
	uint32_t B4, B7;
		
	uint8_t start_address;
	
	uint8_t start_command_temp [] = 
	{ 
		BMP085_CONTROL, 
		BMP085_READTEMPCMD 
	};
	
	uint8_t start_command_pressure [] = 
	{ 
		BMP085_CONTROL, 
		BMP085_READPRESSURECMD + (BMP085_OVERSAMPLING_MODE << 6) 
	};

	int32_t sea_level_pressure = 101325;
	float dt;

	switch (data_.state) 
	{
		case IDLE:
			res &= i2c_.write((uint8_t*) &start_command_temp, 2, BMP085_SLAVE_ADDRESS);
			data_.state = GET_TEMP;
			break;

		case GET_TEMP:
			start_address = BMP085_TEMPDATA;
			res &= i2c_.write((uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS);
			res &= i2c_.read((uint8_t*)&(data_.raw_temperature), 2, BMP085_SLAVE_ADDRESS);
	
			res &= i2c_.write((uint8_t*) &start_command_pressure, 2, BMP085_SLAVE_ADDRESS);
			data_.state = GET_PRESSURE;
			break;

		case GET_PRESSURE:
			start_address = BMP085_PRESSUREDATA;
			res &= i2c_.write((uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS);
			res &= i2c_.read((uint8_t*)&(data_.raw_pressure), 3, BMP085_SLAVE_ADDRESS);
	
			UP = ((uint32_t)data_.raw_pressure[0] << 16 |(uint32_t)data_.raw_pressure[1] << 8 | (uint32_t)data_.raw_pressure[2]) >> (8 - BMP085_OVERSAMPLING_MODE);

			UT = data_.raw_temperature[0] << 8 |data_.raw_temperature[1];
			
			///< step 1
			X1 = (UT - (int32_t)ac6_) * ((int32_t)ac5_) / pow(2,15);
			X2 = ((int32_t)mc_ * pow(2,11)) / (X1 + (int32_t)md_);
			B5 = X1 + X2;
			data_.temperature = (B5 + 8) / pow(2,4);
			data_.temperature /= 10;

			///< do pressure calcs
			B6 = B5 - 4000;
			X1 = ((int32_t)b2_ * ( (B6 * B6) >> 12 )) >> 11;
			X2 = ((int32_t)ac2_ * B6) >> 11;
			X3 = X1 + X2;
			B3 = ((((int32_t)ac1_ * 4 + X3) << BMP085_OVERSAMPLING_MODE) + 2) / 4;


			X1 = ((int32_t)ac3_ * B6) >> 13;
			X2 = ((int32_t)b1_ * ((B6 * B6) >> 12)) >> 16;
			X3 = ((X1 + X2) + 2) >> 2;
			B4 = ((uint32_t)ac4_ * (uint32_t)(X3 + 32768)) >> 15;
			B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> BMP085_OVERSAMPLING_MODE );


			if (B7 < 0x80000000) 
			{
				p = (B7 * 2) / B4;
			} 
			else 
			{
				p = (B7 / B4) * 2;
			}
		
			X1 = (p >> 8) * (p >> 8);
			X1 = (X1 * 3038) >> 16;
			X2 = ( - 7357 * p) >> 16;

			p = p + ((X1 + X2 + (int32_t)3791) >> 4);

			data_.pressure = p;
	
			vertical_speed = data_.altitude;
			altitude = 44330.0f * (1.0f - pow(data_.pressure /sea_level_pressure,0.190295f)) + data_.altitude_offset;
		
			for (int32_t i = 0; i < 2; i++) 
			{
				data_.last_altitudes[i] = data_.last_altitudes[i + 1];
			}
			data_.last_altitudes[2] = altitude;
			altitude = maths_median_filter_3x(data_.last_altitudes[0], data_.last_altitudes[1], data_.last_altitudes[2]);
		
			if (maths_f_abs(altitude-data_.altitude) < 15.0f) 
			{
				data_.altitude = (BARO_ALT_LPF * data_.altitude) + (1.0f - BARO_ALT_LPF) * altitude;
			}
			else 
			{
				data_.altitude = altitude;
			}
		
			dt = (time_keeper_get_micros()-data_.last_update) / 1000000.0f;
			data_.dt = dt;
			vertical_speed = -(data_.altitude-vertical_speed) / dt;
		
			if (abs(vertical_speed) > 20) 
			{
				vertical_speed = 0.0f;
			}

			data_.vario_vz = (VARIO_LPF) * data_.vario_vz + (1.0f - VARIO_LPF) * (vertical_speed);
		
			data_.last_update = time_keeper_get_micros();
			data_.last_update = data_.last_update;
			data_.state = IDLE;
			break;
	}

	data_.last_state_update = time_keeper_get_micros();

	return res;
}
