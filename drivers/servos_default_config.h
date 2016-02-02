/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \brief Default configuration for servos
 * 
 ******************************************************************************/


#ifndef SERVOS_DEFAULT_CONFIG_H_
#define SERVOS_DEFAULT_CONFIG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "servos.h"


static inline servo_entry_t servo_entry_default_standard()
{
	servo_entry_t servo_entry 	= {};
	
	servo_entry.value 			= 0.0f;
	servo_entry.trim 			= 0.0f;
	servo_entry.min 			= -1.0f;
	servo_entry.max 			= 1.0f;
	servo_entry.failsafe 		= 0.0f;
	servo_entry.repeat_freq 	= 50;

	return servo_entry;
};


static inline servo_entry_t servo_entry_default_esc()
{
	servo_entry_t servo_entry 	= {};
	
	servo_entry.value 			= 0.0f;
	servo_entry.trim 			= 0.0f;
	servo_entry.min 			= -0.9f;
	servo_entry.max 			= 1.0f;
	servo_entry.failsafe 		= -1.0f;
	servo_entry.repeat_freq 	= 200;

	return servo_entry;
};


static inline servo_entry_t servo_entry_default_custom()
{
	servo_entry_t servo_entry 	= {};
	
	servo_entry.value 			= 0.0f;
	servo_entry.trim 			= 0.0f;
	servo_entry.min 			= -0.0f;
	servo_entry.max 			= 0.0f;
	servo_entry.failsafe 		= 0.0f;
	servo_entry.repeat_freq 	= 50;

	return servo_entry;
};


static inline servos_conf_t servos_default_config()
{
	servos_conf_t conf 	= {};

	conf.servos_count 	= 4;
	for(uint8_t i = 0; i < MAX_SERVO_COUNT; ++i)
	{
		conf.servo[i] = servo_entry_default_esc();
	}
	
	return conf;
};


#ifdef __cplusplus
	}
#endif

#endif