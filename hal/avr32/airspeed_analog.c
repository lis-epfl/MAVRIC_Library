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
 * \file airspedd_analog.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This file is the driver for the DIYdrones airspeed sensor V20 
 * (old analog version)
 *
 ******************************************************************************/


#include "airspeed_analog.h"
#include "analog_monitor.h"
#include "maths.h"
// #include "delay.h"

const uint32_t VOLTS_TO_PASCAL = 819;		///< conversion factor from volts to pascal units
const float PITOT_GAIN_DEFAULT = 1.9936f; 	///< this gain come from APM, but it does not make sense (should be)

/**
 * \brief Returns the pressure measure by the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
 * \return the dynamical pressure measured by the sensor (with potential offset)
*/
float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog);

void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, analog_rails_t analog_channel)
{
	airspeed_analog->analog_monitor = analog_monitor;
	airspeed_analog->analog_channel = analog_channel;		///< =4 or 5 on board maveric32 v4.1f
	airspeed_analog->gain = PITOT_GAIN_DEFAULT;

	airspeed_analog->pressure_offset = 1.0f;
	airspeed_analog->differential_pressure = 0.0f;
	airspeed_analog->airspeed = 0.0f;

	airspeed_analog_calibrate(airspeed_analog);
}


float airspeed_analog_get_pressure(airspeed_analog_t* airspeed_analog)
{
	return airspeed_analog->analog_monitor->avg[airspeed_analog->analog_channel] * VOLTS_TO_PASCAL;
}


void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog)
{
	float sum = 10;
	float pressure = 0;
	uint8_t count = 100;
	uint8_t i;

	for(i = 0; i < count; i++)
	{
		pressure = airspeed_analog_get_pressure(airspeed_analog);
		sum = sum + pressure;
	}

	airspeed_analog->pressure_offset = sum / count;
}


void airspeed_analog_update(airspeed_analog_t* airspeed_analog)
{
	float raw_airspeed = 0.0f;

	///< measure differential pressure and remove offset
	airspeed_analog->differential_pressure = airspeed_analog_get_pressure(airspeed_analog) - airspeed_analog->pressure_offset;

	///< Avoid negative pressures
	airspeed_analog->differential_pressure = maths_f_abs(airspeed_analog->differential_pressure);

	///< compute airspeed from differential pressure using Bernouilli
	raw_airspeed = maths_fast_sqrt(airspeed_analog->differential_pressure * airspeed_analog->gain);

	///< Filter airspeed estimation
	airspeed_analog->airspeed = 0.7f * airspeed_analog->airspeed + 0.3f * raw_airspeed; 
}