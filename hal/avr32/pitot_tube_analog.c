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
 * \file pitot_tube_analog.c
 * 
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *   
 * \brief This file is the driver for the pitot tube
 *
 ******************************************************************************/


#include "pitot_tube_analog.h"
#include "analog_monitor.h"
#include "maths.h"
#include "time_keeper.h"

// P = ((V / VS) - OFFSET) / SENSITIVITY
#define PITOT_TUBE_VS 5.0f				///< Source voltage [V]
#define PITOT_TUBE_SENSITIVITY 0.0002f	///< Sensitivity of the sensor [V/Pa]
#define PITOT_TUBE_OFFSET 0.5f			///< Offset, ratio from voltage reading at 0 Pa / Source voltage [V]


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Returns the raw differential pressure measured by the pitot tube sensor in Pascal
 *
 * \param pitot_tube_analog pointer to the structure containing the pitot tube sensor's data
 *
 * \return the raw differential pressure measured by the sensor (in Pa)
*/
float pitot_tube_analog_get_raw_differential_pressure(pitot_tube_analog_t* pitot_tube_analog);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

float pitot_tube_analog_get_raw_differential_pressure(pitot_tube_analog_t* pitot_tube_analog)
{
	/*
	// P = ((V / VS) - OFFSET) / SENSITIVITY
	V: Measured voltage [0;5 V]
	VSS: Sensor offset [V]
	VS: Source voltage [5 V]
	OFFSET: Ratio of voltage reading at 0 Pa / VS [0.5]
	SENSITIVITY: Sensitivity [0.0002 V/Pa]
	P: Differential pressure [Pa]	
	*/
	
	pitot_tube_analog->voltage = pitot_tube_analog->analog_monitor->avg[pitot_tube_analog->analog_channel];
	float P = (((pitot_tube_analog->voltage / PITOT_TUBE_VS) - PITOT_TUBE_OFFSET) / PITOT_TUBE_SENSITIVITY);
	
	return  P;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool pitot_tube_analog_init(pitot_tube_analog_t* pitot_tube_analog, analog_monitor_t* analog_monitor, const pitot_tube_analog_conf_t* config)
{
	bool init_success = true;
	
	// Dependencies
	pitot_tube_analog->analog_monitor = analog_monitor;

	// Get configuration values
	pitot_tube_analog->analog_channel = config->analog_rail;
	pitot_tube_analog->conversion_factor = config->conversion_factor;		// Should be around 2/RHO_AIR
	pitot_tube_analog->pressure_offset = config->pressure_offset;
	pitot_tube_analog->correction_gain = config->correction_gain;
	pitot_tube_analog->correction_offset = config->correction_offset;
	pitot_tube_analog->alpha = config->filter_gain;
	pitot_tube_analog->calibration_gain = config->calibration_gain;
	
	// Default values to start the calibration correctly !
	pitot_tube_analog->voltage = 0.0f;
	pitot_tube_analog->differential_pressure = 0.0f;
	pitot_tube_analog->raw_airspeed = 0.0f;
	pitot_tube_analog->scaled_airspeed = 0.0f;
	pitot_tube_analog->airspeed = 0.0f;
	pitot_tube_analog->last_airspeed = 0.0f;
	pitot_tube_analog->calibrating = false;
	
	// Start calibration
	pitot_tube_analog_start_calibration(pitot_tube_analog);

	return init_success;
}

void pitot_tube_analog_start_calibration(pitot_tube_analog_t* pitot_tube_analog)
{
	// Trigger the calibration
	pitot_tube_analog->calibrating = true;
}

void pitot_tube_analog_stop_calibration(pitot_tube_analog_t* pitot_tube_analog)
{
	// Stop calibration process
	pitot_tube_analog->calibrating = false;
}


task_return_t pitot_tube_analog_update(pitot_tube_analog_t* pitot_tube_analog)
{
	// Get differential pressure
	float raw_differential_pressure = pitot_tube_analog_get_raw_differential_pressure(pitot_tube_analog);

	// Wait before ADC reads correct values for pressure offset (compute it directly on value coming out of the sensor)
	if (pitot_tube_analog->calibrating && time_keeper_get_millis() >= 3000)
	{
		// Mean value of the raw airspeed --> low-pass filter !
		pitot_tube_analog->pressure_offset = pitot_tube_analog->calibration_gain * pitot_tube_analog->pressure_offset + (1 - pitot_tube_analog->calibration_gain) * raw_differential_pressure;
	}

	// Correct the raw pressure
	pitot_tube_analog->differential_pressure = raw_differential_pressure - pitot_tube_analog->pressure_offset;
	
	// First airspeed estimation, avoiding negative pressure
	// TODO: Plug the tube in the correct way and use max(pres, 0) instead of abs to have better results !
	pitot_tube_analog->raw_airspeed = maths_fast_sqrt( maths_f_abs(pitot_tube_analog->differential_pressure) * pitot_tube_analog->conversion_factor );
	
	// Correct it using fitted relation: Airspeed_measured = gain * Airspeed_true + offset
	pitot_tube_analog->scaled_airspeed = (pitot_tube_analog->raw_airspeed - pitot_tube_analog->correction_offset)/pitot_tube_analog->correction_gain;
	
	// Filter the airspeed
	if (time_keeper_get_millis() >= 3000)
	{
		pitot_tube_analog->last_airspeed = pitot_tube_analog->airspeed;
		pitot_tube_analog->airspeed = pitot_tube_analog->alpha * pitot_tube_analog->airspeed + (1 - pitot_tube_analog->alpha) * pitot_tube_analog->scaled_airspeed;
	}

	// TODO: Convert to true airspeed (EAS --> TAS) so that convertion_factor is no more dependent on altitude !
	
	return TASK_RUN_SUCCESS;
}