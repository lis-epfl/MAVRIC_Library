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
 * \brief Default configuration for servos
 * 
 ******************************************************************************/


#ifndef SERVOS_DEFAULT_CONFIG_H_
#define SERVOS_DEFAULT_CONFIG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "servos.h"


static const servo_entry_t servo_entry_default_standard =
{
	.value 			= 0.0f,
	.trim 			= 0.0f,
	.min 			= -1.0f,
	.max 			= 1.0f,
	.failsafe 		= 0.0f,
	.repeat_freq 	= 50,
};


static const servo_entry_t servo_entry_default_esc =
{
	.value 			= 0.0f,
	.trim 			= 0.0f,
	.min 			= -0.9f,
	.max 			= 1.0f,
	.failsafe 		= -1.0f,
	.repeat_freq 	= 200,
};


static const servo_entry_t servo_entry_default_custom =
{
	.value 			= 0.0f,
	.trim 			= 0.0f,
	.min 			= -0.0f,
	.max 			= 0.0f,
	.failsafe 		= 0.0f,
	.repeat_freq 	= 50,
};

servos_conf_t servos_default_config =
{
	.servos_count = 4,
	.servo =
	{
		{ //servo_entry_default_esc
			.value 			= 0.0f,
			.trim 			= 0.0f,
			.min 			= -0.9f,
			.max 			= 1.0f,
			.failsafe 		= -1.1f,
			.repeat_freq 	= 200,
		},
		{ //servo_entry_default_esc
			.value 			= 0.0f,
			.trim 			= 0.0f,
			.min 			= -0.9f,
			.max 			= 1.0f,
			.failsafe 		= -1.1f,
			.repeat_freq 	= 200,
		},
		{ //servo_entry_default_esc
			.value 			= 0.0f,
			.trim 			= 0.0f,
			.min 			= -0.9f,
			.max 			= 1.0f,
			.failsafe 		= -1.1f,
			.repeat_freq 	= 200,
		},
		{ //servo_entry_default_esc
			.value 			= 0.0f,
			.trim 			= 0.0f,
			.min 			= -0.9f,
			.max 			= 1.0f,
			.failsafe 		= -1.1f,
			.repeat_freq 	= 200,
		}
	},
};


#ifdef __cplusplus
	}
#endif

#endif