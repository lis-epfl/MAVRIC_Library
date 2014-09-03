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
 * \author Julien Lecoeur
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

/**
 * \brief Structure containing the analog airspeed sensor datas
*/
typedef struct {
	uint8_t analog_channel;					///< analog channel of the ADC
	float gain;								///< gain factor for the ADC
	float pressure_offset;					///< offset of the pressure sensor
	float differential_pressure;			///< true dynamical pressure (diff between pressure and offset)
	float airspeed;							///< measure airspeed
	analog_monitor_t* analog_monitor;		///< pointer to the structure of analog monitor module
} airspeed_analog_t;

/**
 * \brief Initialize the airspeed sensor
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 * \param analog_monitor pointer to the structure of analog monitor module
 * \param analog_channel set which channel of the ADC is map to the airspeed sensor
 *
*/
void airspeed_analog_init(airspeed_analog_t* airspeed_analog, analog_monitor_t* analog_monitor, analog_rails_t analog_channel);

/**
 * \brief Calibrates the airspeed sensor
 * 
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_calibrate(airspeed_analog_t* airspeed_analog);

/**
 * \brief Updates the values in the airspeed structure
 *
 * \param airspeed_analog pointer to the structure containing the airspeed sensor's data
 *
*/
void airspeed_analog_update(airspeed_analog_t* airspeed_analog);

#ifdef __cplusplus
}
#endif

#endif /* AIRSPEED_ANALOG_H_ */