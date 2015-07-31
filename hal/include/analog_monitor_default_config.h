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
 * \file analog_monitor_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *   
 * \brief The configuration for the analog monitor module
 *
 * in MAVRIC autopilot we have a voltage divider on ADC_6, ADC_7, ADC_10 and ADC_11
 * Voltage divider values change from rev_4 to rev_4_1 and followings
 *
 * For rev_4 the inverse of the voltage divider is
 * 6.6f on ADC_6 and ADC_7 and 11.0f on ADC_10 and ADC_11
 *
 * For rev4_1 and followings
 * 9.1818f on ADC_6 and ADC_7 and 23.0f on ADC_10 and ADC_11
 *
 * You can either 
 * edit this file to change the voltage divider values
 * or 
 * in your project, declare float* config = conf_conv_factor; 
 * and then edit config array to change one or few values accordingly
 * before calling analog_monitor_init(&central_data->analog_monitor, config);
 *
 ******************************************************************************/


#ifndef ANALOG_MONITOR_DEFAULT_CONFIG_H_
#define ANALOG_MONITOR_DEFAULT_CONFIG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "analog_monitor.h"

#define INV_VOLTAGE_DIVIDER_1 9.1818f	// previously 6.6f for V4.0
#define INV_VOLTAGE_DIVIDER_2 23.0f		// previously 11.0f for V4.0

static const analog_monitor_conf_t analog_monitor_default_config =
{
	.enable =
	{
		false,		//ANALOG_RAIL_2,
		false,		//ANALOG_RAIL_3,
		false,		//ANALOG_RAIL_4,
		false,		//ANALOG_RAIL_5,
		false,		//ANALOG_RAIL_6,
		false,		//ANALOG_RAIL_7,
		true,		//ANALOG_RAIL_10
		true,		//ANALOG_RAIL_11
		true,		//ANALOG_RAIL_12
		false,		//ANALOG_RAIL_13
	},
	.conv_factor =
	{
		1.0f,									//.conv_factor_2 =
		1.0f,									//.conv_factor_3 =
		1.0f,									//.conv_factor_4 =
		1.0f,									//.conv_factor_5 =
		(0.00023485f * INV_VOLTAGE_DIVIDER_1),	//.conv_factor_6 =
		(0.00023485f * INV_VOLTAGE_DIVIDER_1),	//.conv_factor_7 =
		(-0.0002409f * INV_VOLTAGE_DIVIDER_2),	//.conv_factor_10 =
		(-0.0002409f * INV_VOLTAGE_DIVIDER_2),	//.conv_factor_11 =
		-0.00025f,								//.conv_factor_12 =
		-1.0f									//.conv_factor_13 =
	}
};

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_MONITOR_DEFAULT_CONFIG_H_ */