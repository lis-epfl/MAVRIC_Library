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
 * \file airspedd_analog.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur, Simon Pyroth
 *   
 * \brief This file is the driver for the DIYdrones airspeed sensor V20 
 * (old analog version)
 *
 ******************************************************************************/


#ifndef AIRSPEED_ANALOG_H_
#define AIRSPEED_ANALOG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "analog_monitor.h"
#include "scheduler.h"



/**
 * \brief  Configuration for the airspeed sensor
 */
typedef struct 
{
	analog_rails_t analog_rail;		///< Analog rail on which the sensor is connected
	float filter_gain;				///< Gain for the low-pass filter
	float airspeed_offset;			///< Default airspeed offset
	float correction_gain;			///< Gain obtained by the fitted relation (Airspeed_measured = gain * Airspeed_true + offset)
	float correction_offset;		///< Offset obtained by the fitted relation
	float calibration_gain;			///< Gain used for the calibration of the offset (low-pass)
} airspeed_analog_conf_t;

/**
 * \brief Structure containing the analog airspeed sensor data
*/
typedef struct {
	analog_monitor_t* analog_monitor;		///< pointer to the structure of analog monitor module
	uint8_t analog_channel;					///< analog channel of the ADC
	float voltage;							///< Voltage read by the ADC
	
	float differential_pressure;			///< True dynamical pressure (in kPa)
	float airspeed_offset;					///< Offset of the airspeed module
	float correction_gain;					///< Gain used to correct estimation
	float correction_offset;				///< Offset used to correct estimation
	float alpha;							///< Filter coefficient
	
	float raw_airspeed;						///< Unfiltered and uncorrected airspeed
	float scaled_airspeed;					///< Corrected airspeed, using offset and fitted relation
	float airspeed;							///< Filtered corrected airspeed
	
	bool calibrating;						///< True if the sensor is currently in calibration
	float calibration_gain;					///< Gain used for the calibration of the offset (low-pass)
} airspeed_analog_t;

/**
 * \brief Initialize the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 * \param analog_monitor pointer to the structure of analog monitor module
 * \param analog_channel set which channel of the ADC is map to the airspeed sensor
 *
*/
bool airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, const airspeed_analog_conf_t* config);

/**
 * \brief Calibrates the airspeed sensor offset at 0 speed. It will continue calibrating until it is asked to stop.
 * 
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_start_calibration(airspeed_analog_t* airspeed_analog);

/**
 * \brief Stop the calibration procedure and keep the last offset.
 * 
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_stop_calibration(airspeed_analog_t* airspeed_analog);

/**
 * \brief Updates the values in the airspeed structure
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
 * \return	The result of the task execution
*/
task_return_t airspeed_analog_update(airspeed_analog_t* airspeed_analog);

#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_ANALOG_H_ */