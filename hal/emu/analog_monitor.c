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
 * \file analog_monitor.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Gregoire Heitz
 *   
 * \brief The driver for the analog monitor module
 *
 ******************************************************************************/


#include "analog_monitor.h"
#include "adc_int.h"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief Trigger the analog monitor
 * Start sampling
 */
void trigger_analog_monitor(void);

/**
 * \brief Compute the average of the analog channels
 *
 * \param	analog_monitor		The pointer to the analog monitor structure
 * \param	rail				The rail structure
 * 
 * \return	Return the average of an analog channel
 */
float analog_compute_avg(analog_monitor_t* analog_monitor, analog_rails_t rail);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void trigger_analog_monitor(void)
{
	// adc_int_start_sampling(MONITOR_SAMPLES, 100,16, 4, false);
}

float analog_compute_avg(analog_monitor_t* analog_monitor, analog_rails_t rail)
{
	float out = 0.0f;
	
	for (int32_t i = 0; i  <MONITOR_SAMPLES; i++)
	{
		out += (float)analog_monitor->buffer[rail][i];
	}
	out = out / MONITOR_SAMPLES;
	return out;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void analog_monitor_init(analog_monitor_t* analog_monitor, const analog_monitor_conf_t* config) 
{	
	// Init buffer and avg outputs
	for (int32_t i = 0; i < MONITOR_CHANNELS; ++i)
	{
		// Init buffer
		for (int32_t j = 0; j < MONITOR_SAMPLES; ++j)
		{
			analog_monitor->buffer[i][j] = 0;
		}
		// Init avg outputs
		analog_monitor->avg[i] = 0;
		
		// Init conv_factor
		analog_monitor->conv_factor[i] = config->conv_factor[i];

		// Init enable flags
		analog_monitor->enable[i] = config->enable[i];
	}

	// Init desired ADC pin
	// adc_int_init(100000, ADCIFA_REF06VDD);

	// if (analog_monitor->enable[ANALOG_RAIL_2])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_2], 
	// 						AVR32_ADCIFA_INP_ADCIN2, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_3])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_3], 
	// 						AVR32_ADCIFA_INP_ADCIN3, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_4])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_4], 
	// 						AVR32_ADCIFA_INP_ADCIN4, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_5])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_5], 
	// 						AVR32_ADCIFA_INP_ADCIN5, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_6])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_6], 
	// 						AVR32_ADCIFA_INP_ADCIN6, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_7])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_7], 
	// 						AVR32_ADCIFA_INP_ADCIN7, 
	// 						AVR32_ADCIFA_INN_GNDANA, 
	// 						ADCIFA_SHG_1);  ///< 5V_ANALOG
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_10])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_10], 
	// 						AVR32_ADCIFA_INP_GNDANA, 
	// 						AVR32_ADCIFA_INN_ADCIN10, 
	// 						ADCIFA_SHG_1);  ///< BAT_FILTERED
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_11])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_11], 
	// 						AVR32_ADCIFA_INP_GNDANA, 
	// 						AVR32_ADCIFA_INN_ADCIN11, 
	// 						ADCIFA_SHG_1); ///< BATTERY
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_12])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_12], 
	// 						AVR32_ADCIFA_INP_GNDANA, 
	// 						AVR32_ADCIFA_INN_ADCIN12, 
	// 						ADCIFA_SHG_1); ///< Analog pin 12
	// }
	// if (analog_monitor->enable[ANALOG_RAIL_13])
	// {
	// 	adc_int_sequencer_add(	analog_monitor->buffer[ANALOG_RAIL_13], 
	// 						AVR32_ADCIFA_INP_GNDANA, 
	// 						AVR32_ADCIFA_INN_ADCIN13, 
	// 						ADCIFA_SHG_1); ///< Analog pin 13
	// }
}

task_return_t analog_monitor_update(analog_monitor_t* analog_monitor)
{

	for (int32_t i = 0; i < MONITOR_CHANNELS; ++i)
	{
		if(analog_monitor->enable[i])
		{
			analog_monitor->avg[i] = analog_compute_avg(analog_monitor, i) * analog_monitor->conv_factor[i];
		}
	}

	trigger_analog_monitor();

	return TASK_RUN_SUCCESS;
}