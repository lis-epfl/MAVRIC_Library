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
 * \file servos.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Abstraction layer for servomotors. This module does not write to 
 * hardware, it just holds data and configuration for servos.
 * 
 ******************************************************************************/


#ifndef SERVOS_H_
#define SERVOS_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include <stdint.h>
#include "scheduler.h"

#define MAX_SERVO_COUNT 8

typedef enum
{
	STANDARD_SERVO 	 = 0,
	MOTOR_CONTROLLER = 1,
	CUSTOM_SERVO     = 2
} servo_type_t;


typedef struct
{
	uint32_t servos_count;
	servo_type_t types[MAX_SERVO_COUNT];
} servos_conf_t;


typedef struct
{
	float value;			///< Normalized value of the servo (between -1 and 1)
	float trim;				///< Trim value (between -1 and 1)
	float min;				///< Minimum value (between -1 and 1)
	float max;				///< Max value (between -1 and 1)
	float failsafe;			///< Failsafe position of the servo (between -1 and 1)
	uint32_t repeat_freq;	///< Update frequency of the servo (in Hz)
	servo_type_t type;		///< Type of servo
} servo_entry_t;


typedef struct 
{
	uint32_t servos_count;
	servo_entry_t servo[MAX_SERVO_COUNT];
} servos_t;


void servos_init(servos_t* servos, const servos_conf_t* config);


void servos_set_value(servos_t* servos, uint32_t servo_id, float value);


void servos_set_value_failsafe(servos_t* servos);





#ifdef __cplusplus
	}
#endif

#endif