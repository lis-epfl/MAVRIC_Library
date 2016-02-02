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
 * \file bmp085.hpp
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief 	Driver for the BMP085 barometer
 * 
 ******************************************************************************/


#ifndef BMP085_HPP_
#define BMP085_HPP_

#include <stdint.h>
#include <stdbool.h>

#include "i2c.hpp"
#include "barometer.hpp"


/**
 * \brief 	Sensor state
*/
typedef enum
{
	BMP085_IDLE,			///< Idle state
	BMP085_GET_TEMP,		///< Getting temperature state
	BMP085_GET_PRESSURE		///< Getting pressure state
} bmp085_state_t;


/**
 * \brief 	Driver for the BMP085 barometer
 */
class Bmp085: public Barometer
{
public:
	/**
	 * \brief  	Constructor
	 * 
	 * \param 	i2c 	Reference to I2C device 
	 */
	Bmp085(	I2c& i2c );


	/**
	 * \brief   Initialise the sensor
	 * 			
	 * \return 	Success
	 */	
	bool init(void);


	/**
	 * \brief 	Main update function
	 * \detail 	Reads new values from sensor
	 * 
	 * \return 	Success
	 */
	bool update(void);


private:
	I2c&		i2c_;					///< Reference to I2C peripheral
	
	int16_t 	ac1_;					///< Configuration values for the
	int16_t 	ac2_;					///< barometer, given by the datasheet
	int16_t 	ac3_; 					///< of the sensor
	uint16_t 	ac4_; 					///< ..
	uint16_t 	ac5_; 					///< ..
	uint16_t 	ac6_;					///< TODO: constants?
	int16_t 	mb_; 					///< => move to constants in cpp
	int16_t		mc_; 					///<    or
	int16_t 	md_;					///<    move to configuration structure
	int16_t 	b1_; 					///< ..
	int16_t 	b2_;					///< ..

	uint8_t 	raw_pressure_[3];		///< Raw pressure contained in 3 uint8_t
	uint8_t 	raw_temperature_[2];	///< Raw temperature contained in 2 uint8_t
	
	float 		last_altitudes_[3];		///< Array to store previous value of the altitude for low pass filtering the output
	
	float 		last_state_update_us_;	///< Time of the last state update
	float 		dt_s_;					///< Time step for the derivative
	
	bmp085_state_t 	state_;				///< State of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
};

#endif /* BMP085_HPP_ */