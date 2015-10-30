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
 * \file airspeed_analog.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur, Simon Pyroth
 *   
 * \brief This file is the driver for the DIYdrones airspeed sensor V20 
 * (old analog version)
 *
 ******************************************************************************/


#include "airspeed_analog.h"
#include "analog_monitor.h"
#include "maths.h"

#define AIRSPEED_SENSOR_OFFSET 2.5f			///< Offset of the sensor [V]
#define AIRSPEED_SENSOR_SENSITIVITY 1.0f	///< Sensitivity of the sensor [V/kPa]
#define RHO_AIR 1.293f						///< Air density [kg/m^3]



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Returns the raw differential pressure measured by the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
 * \return the raw differential pressure measured by the sensor (in kPa)
*/
float airspeed_analog_get_raw_differential_pressure(airspeed_analog_t* airspeed_analog);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float airspeed_analog_get_raw_differential_pressure(airspeed_analog_t* airspeed_analog)
{
	/* Sensor: V = S*P + O
	V: Measured voltage [0;5 V]
	S: Sensitivity [1 V/kPa]
	P: Differential pressure [kPa]
	O: Offset [2.5 V]
	
	==> P = (V - O)/S
	*/
	//airspeed_analog->voltage = airspeed_analog->analog_monitor->avg[airspeed_analog->analog_channel];
	//float P = (airspeed_analog->voltage - AIRSPEED_SENSOR_OFFSET)/AIRSPEED_SENSOR_SENSITIVITY;
	
	airspeed_analog->voltage = airspeed_analog->analog_monitor->avg[airspeed_analog->analog_channel];
	float P = (airspeed_analog->voltage - AIRSPEED_SENSOR_OFFSET)/AIRSPEED_SENSOR_SENSITIVITY;
	
	return  P;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, const airspeed_analog_conf_t* config)
{
	bool init_success = true;
	// Dependencies
	airspeed_analog->analog_monitor = analog_monitor;
	airspeed_analog->analog_channel = config->analog_rail;
	airspeed_analog->voltage = 0.0f;

	// Default values to start the calibration correctly !
	airspeed_analog->pressure_offset = config->pressure_offset;
	airspeed_analog->differential_pressure = 0.0f;
	airspeed_analog->raw_airspeed = 0.0f;
	airspeed_analog->airspeed = 0.0f;
	airspeed_analog->alpha = config->filter_gain;
	airspeed_analog->currently_turning = 0;

	// Calibration
	//airspeed_analog_start_calibration(airspeed_analog);
	return init_success;
}

void airspeed_analog_start_calibration(airspeed_analog_t* airspeed_analog)
{
	// Reset calibration data
	airspeed_analog->calibration_counter = 0;
	airspeed_analog->calibration_pressure = 0.0f;
	
	// Trigger the calibration
	airspeed_analog->calibrating = true;
}


task_return_t airspeed_analog_update(airspeed_analog_t* airspeed_analog)
{
	if (airspeed_analog->calibrating)
	{
		// Accumulate data
		airspeed_analog->calibration_counter++;
		airspeed_analog->calibration_pressure += airspeed_analog_get_raw_differential_pressure(airspeed_analog);
		
		// Stop the calibration and update offset value
		if (airspeed_analog->calibration_counter >= 15)
		{
			airspeed_analog->calibrating = false;
			airspeed_analog->pressure_offset = airspeed_analog->calibration_pressure / airspeed_analog->calibration_counter;
			
			// Reset sensor output data and filter
			airspeed_analog->airspeed = 0.0f;
			airspeed_analog->raw_airspeed = 0.0f;
		}
	}
	else
	{
		// Get differential pressure (offset compensated)
		airspeed_analog->differential_pressure = airspeed_analog_get_raw_differential_pressure(airspeed_analog) - airspeed_analog->pressure_offset;

		// Avoid negative pressures
		airspeed_analog->differential_pressure = maths_f_abs(airspeed_analog->differential_pressure);

		// Compute airspeed from differential pressure using Bernouilli
		airspeed_analog->raw_airspeed = maths_fast_sqrt(2*airspeed_analog->differential_pressure*1000.0f/RHO_AIR);

		// Filter airspeed estimation
		airspeed_analog->airspeed = airspeed_analog->alpha * airspeed_analog->airspeed + (1 - airspeed_analog->alpha) * airspeed_analog->raw_airspeed;
	}
	
	return TASK_RUN_SUCCESS;
}