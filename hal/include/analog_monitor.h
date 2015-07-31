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
 * \file analog_monitor.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 *   
 * \brief The driver for the analog monitor module
 *
 ******************************************************************************/


#ifndef ANALOG_MONITOR_H_
#define ANALOG_MONITOR_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "scheduler.h"

#define MONITOR_CHANNELS 10
#define MONITOR_SAMPLES  10


/**
 * \brief Enumerate the analog rail
 */
typedef enum
{
	ANALOG_RAIL_2,		// ANA connector, pin1
	ANALOG_RAIL_3,		// ANA connector, pin2
	ANALOG_RAIL_4,		// ANA connector, pin3
	ANALOG_RAIL_5,		// ANA connector, pin4
	ANALOG_RAIL_6,		// 6V
	ANALOG_RAIL_7,		// 5V_ANA
	ANALOG_RAIL_10,		// BATTERY_FILTERED
	ANALOG_RAIL_11,		// BATTERY
	ANALOG_RAIL_12,		// P8 connector, pin 1
	ANALOG_RAIL_13  	// P9 connector, pin 1
} analog_rails_t;


/**
 * \brief  Configuration for the module analog monitor
 */
typedef struct 
{
	bool enable[MONITOR_CHANNELS];			///< Activation of each analog channel
	float conv_factor[MONITOR_CHANNELS];	///< Conversion factors
} analog_monitor_conf_t;


/**
 * \brief  Analog monitor structure
 */
typedef struct 
{
	bool enable[MONITOR_CHANNELS];						///< Activation of each analog channel
	float conv_factor[MONITOR_CHANNELS];				///< Conversion factors
	int16_t buffer[MONITOR_CHANNELS][MONITOR_SAMPLES];	///< Buffer of previous data
	float avg[MONITOR_CHANNELS];						///< Averaged voltage
} analog_monitor_t;


/**
 * \brief	Initialisation of the analog monitor
 * 
 * \param	analog_monitor	Pointer to the analog monitor structure
 * \param	config			Pointer to the configuration array for CONV_FACTOR of the ADC
 */
void analog_monitor_init(analog_monitor_t* analog_monitor, const analog_monitor_conf_t* config);


/**
 * \brief	Update of the analog monitor
 * 
 * \param	analog_monitor	The pointer to the analog monitor structure
 *
 * \return	Returns the task result, currently only TASK_RUN_SUCCESS
 */
task_return_t analog_monitor_update(analog_monitor_t* analog_monitor);

#ifdef __cplusplus
	}
#endif

#endif /* ANALOG_MONITOR_H_ */