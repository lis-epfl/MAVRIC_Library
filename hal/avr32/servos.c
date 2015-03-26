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
 * \file servos.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Abstraction layer for servomotors. This module does not write to 
 * hardware, it just holds data and configuration for servos.
 * 
 ******************************************************************************/


#include "servos.h"
#include "print_util.h"

bool servos_init(servos_t* servos, const servos_conf_t* config)
{
	bool init_success = true;
	
	// Init servo array
	if ( config->servos_count <= MAX_SERVO_COUNT )
	{
		servos->servos_count = config->servos_count;
	
		for (int i = 0; i < servos->servos_count; ++i)
		{
			// Set default parameters for each type of servo
			servos->servo[i] = config->servo[i];

			// Set default value to failsafe
			servos->servo[i].value = servos->servo[i].failsafe;
		}
		
		init_success &= true;
	}
	else
	{
		servos->servos_count = 0;
		print_util_dbg_print("[SERVOS] ERROR! Too many servos\r\n");
		
		init_success &= false;
	}
	
	print_util_dbg_print("[SERVOS] Init.\r\n");
	
	return init_success;
}


void servos_set_value(servos_t* servos, uint32_t servo_id, float value)
{
	if ( servo_id <= servos->servos_count )
	{
		float trimmed_value = value + servos->servo[servo_id].trim;

		if ( trimmed_value < servos->servo[servo_id].min )
		{
			servos->servo[servo_id].value = servos->servo[servo_id].min;
		}
		else if ( trimmed_value > servos->servo[servo_id].max )
		{
			servos->servo[servo_id].value = servos->servo[servo_id].max;
		}
		else
		{
			servos->servo[servo_id].value = trimmed_value;
		}
	}
}


void servos_set_value_failsafe(servos_t* servos)
{
	for (int i = 0; i < servos->servos_count; ++i)
	{
		servos->servo[i].value = servos->servo[i].failsafe;
	}
}