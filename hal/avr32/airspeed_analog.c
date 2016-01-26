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
#include "time_keeper.h"

#define AIRSPEED_SENSOR_OFFSET 2.5f			///< Offset of the sensor [V]
#define AIRSPEED_SENSOR_SENSITIVITY 0.001f	///< Sensitivity of the sensor [V/Pa]



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Returns the raw differential pressure measured by the airspeed sensor in Pascal
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
 * \return the raw differential pressure measured by the sensor (in Pa)
*/
float airspeed_analog_get_raw_differential_pressure(airspeed_analog_t* airspeed_analog);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float airspeed_analog_get_raw_differential_pressure(airspeed_analog_t* airspeed_analog)
{
	/* Sensor: V = S*P + O
	V: Measured voltage [0;5 V]
	S: Sensitivity [0.001 V/Pa]
	P: Differential pressure [Pa]
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

	// Get configuration values
	airspeed_analog->analog_channel = config->analog_rail;
	airspeed_analog->conversion_factor = config->conversion_factor;		// Should be around 2/RHO_AIR
	airspeed_analog->pressure_offset = config->pressure_offset;
	airspeed_analog->correction_gain = config->correction_gain;
	airspeed_analog->correction_offset = config->correction_offset;
	airspeed_analog->alpha = config->filter_gain;
	airspeed_analog->calibration_gain = config->calibration_gain;
	
	// Default values to start the calibration correctly !
	airspeed_analog->voltage = 0.0f;
	airspeed_analog->differential_pressure = 0.0f;
	airspeed_analog->raw_airspeed = 0.0f;
	airspeed_analog->scaled_airspeed = 0.0f;
	airspeed_analog->airspeed = 0.0f;
	airspeed_analog->last_airspeed = 0.0f;
	airspeed_analog->calibrating = false;
	
	// Start calibration
	airspeed_analog_start_calibration(airspeed_analog);

	return init_success;
}

void airspeed_analog_start_calibration(airspeed_analog_t* airspeed_analog)
{
	// Trigger the calibration
	airspeed_analog->calibrating = true;
}

void airspeed_analog_stop_calibration(airspeed_analog_t* airspeed_analog)
{
	// Stop calibration process
	airspeed_analog->calibrating = false;
}


task_return_t airspeed_analog_update(airspeed_analog_t* airspeed_analog)
{
	// Get differential pressure
	float raw_differential_pressure = airspeed_analog_get_raw_differential_pressure(airspeed_analog);

	// Wait before ADC reads correct values for pressure offset (compute it directly on value coming out of the sensor)
	if (airspeed_analog->calibrating && time_keeper_get_millis() >= 3000)
	{
		// Mean value of the raw airspeed --> low-pass filter !
		airspeed_analog->pressure_offset = airspeed_analog->calibration_gain * airspeed_analog->pressure_offset + (1 - airspeed_analog->calibration_gain) * raw_differential_pressure;
	}

	// Correct the raw pressure
	airspeed_analog->differential_pressure = raw_differential_pressure - airspeed_analog->pressure_offset;
	
	// First airspeed estimation, avoiding negative pressure
	// TODO: Plug the tube in the correct way and use max(pres, 0) instead of abs to have better results !
	airspeed_analog->raw_airspeed = maths_fast_sqrt( maths_f_abs(airspeed_analog->differential_pressure) * airspeed_analog->conversion_factor );
	
	// Correct it using fitted relation: Airspeed_measured = gain * Airspeed_true + offset
	airspeed_analog->scaled_airspeed = (airspeed_analog->raw_airspeed - airspeed_analog->correction_offset)/airspeed_analog->correction_gain;
	
	// Filter the airspeed
	if (time_keeper_get_millis() >= 3000)
	{
		airspeed_analog->last_airspeed = airspeed_analog->airspeed;
		airspeed_analog->airspeed = airspeed_analog->alpha * airspeed_analog->airspeed + (1 - airspeed_analog->alpha) * airspeed_analog->scaled_airspeed;
	}

	// TODO: Convert to true airspeed (EAS --> TAS) so that convertion_factor is no more dependent on altitude !
	
	return TASK_RUN_SUCCESS;
}