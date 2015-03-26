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
 * \file barometer.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief This file defines the barometer enum and structure, independently from which sensor is used
 *
 ******************************************************************************/


#ifndef BAROMETER_H_
#define BAROMETER_H_

/**
 * \brief bmp085_state_t can get three different state: Idle, get Temperature or get Pressure
*/
typedef enum bmp085_state_t
{
	IDLE,								///< Idle state
	GET_TEMP,							///< Getting temperature state
	GET_PRESSURE						///< Getting pressure state
} barometer_state_t;


/**
 * \brief Define the barometer structure
 */
typedef struct
{
	uint8_t 	raw_pressure[3];		///< Raw pressure contained in 3 uint8_t
	uint8_t 	raw_temperature[2];		///< Raw temperature contained in 2 uint8_t
	
	float 		pressure;				///< Measured pressure as the concatenation of the 3 uint8_t raw_pressure
	float 		temperature;			///< Measured temperature as the concatenation of the 2 uint8_t raw_temperature
	float 		altitude;				///< Measured altitude as the median filter of the 3 last_altitudes
	float 		altitude_offset;		///< Offset of the barometer sensor for matching GPS altitude value
	float 		vario_vz;				///< Vario altitude speed
	
	float 		last_altitudes[3];		///< Array to store previous value of the altitude for low pass filtering the output
	
	uint32_t 	last_update;			///< Time of the last update of the barometer
	uint32_t 	last_state_update;		///< Time of the last state update
	barometer_state_t state;			///< State of the barometer sensor (IDLE, GET_TEMP, GET_PRESSURE)
	float 		dt;						///< Time step for the derivative
} barometer_t;

#endif /* BAROMETER_H_ */
