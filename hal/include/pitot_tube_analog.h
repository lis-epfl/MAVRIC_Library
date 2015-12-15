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
 * \file pitot_tube_analog.h
 * 
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *   
 * \brief This file is the driver for the pitot tube
 *
 ******************************************************************************/


#ifndef PITOT_TUBE_ANALOG_H
#define PITOT_TUBE_ANALOG_H

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "analog_monitor.h"
#include "scheduler.h"



/**
 * \brief  Configuration for the pitot tube sensor
 */
typedef struct 
{
	analog_rails_t analog_rail;		///< Analog rail on which the sensor is connected
	float pressure_offset;			///< Default airspeed offset
	float calibration_gain;			///< Gain used for the calibration of the offset (low-pass)
	float conversion_factor;		///< Factor used for conversion between differential pressure P and square speed v^2. Should be 2/rho_air
	float correction_gain;			///< Gain obtained by the fitted relation (Airspeed_measured = gain * Airspeed_true + offset)
	float correction_offset;		///< Offset obtained by the fitted relation
	float filter_gain;				///< Gain for the low-pass filter
} pitot_tube_analog_conf_t;

/**
 * \brief Structure containing the analog airspeed sensor data
*/
typedef struct {
	analog_monitor_t* analog_monitor;		///< pointer to the structure of analog monitor module
	uint8_t analog_channel;					///< analog channel of the ADC
	float voltage;							///< Voltage read by the ADC
	
	float pressure_offset;					///< Offset of the pressure sensor
	float differential_pressure;			///< True differential pressure in Pa (raw sensor compensated with offset)
	float conversion_factor;				///< Factor used for conversion between differential pressure P and square speed v^2. Is influenced by real sensitivity of the sensor and by air density !
	float correction_gain;					///< Gain used to correct estimation
	float correction_offset;				///< Offset used to correct estimation
	float alpha;							///< Filter coefficient
	
	float raw_airspeed;						///< Unfiltered and uncorrected airspeed
	float scaled_airspeed;					///< Corrected airspeed, using fitted relation
	float airspeed;							///< Filtered corrected airspeed
	float last_airspeed;					///< Airspeed from previous loop
	
	bool calibrating;						///< True if the sensor is currently in calibration
	float calibration_gain;					///< Gain used for the calibration of the offset (low-pass)
} pitot_tube_analog_t;

/**
 * \brief Initialize the pitot tube sensor
 *
 * \param pitot_tube_analog pointer to the structure containing the pitot tube sensor's data
 * \param analog_monitor pointer to the structure of analog monitor module
 * \param pitot_tube_channel set which channel of the ADC is map to the pitot tube sensor
 *
*/
bool pitot_tube_analog_init(pitot_tube_analog_t* pitot_tube_analog, analog_monitor_t* analog_monitor, const pitot_tube_analog_conf_t* config);

/**
 * \brief Calibrates the pitot_tube sensor offset at 0 speed. It will continue calibrating until it is asked to stop.
 * 
 * \param pitot_tube_analog pointer to the structure containing the pitot tube sensor's data
 *
*/
void pitot_tube_analog_start_calibration(pitot_tube_analog_t* pitot_tube_analog);

/**
 * \brief Stop the calibration procedure and keep the last offset.
 * 
 * \param pitot_tube_analog pointer to the structure containing the pitot tube sensor's data
 *
*/
void pitot_tube_analog_stop_calibration(pitot_tube_analog_t* pitot_tube_analog);

/**
 * \brief Updates the values in the pitot tube structure
 *
 * \param pitot_tube_analog pointer to the structure containing the pitot tubes sensor's data
 *
 * \return	The result of the task execution
*/
task_return_t pitot_tube_analog_update(pitot_tube_analog_t* pitot_tube_analog);

#ifdef __cplusplus
}
#endif

#endif /* PITOT_TUBE_ANALOG_H */