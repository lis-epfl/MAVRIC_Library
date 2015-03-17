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
 * \file velocity_controller_copter_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief Default configuration for the module velocity_controller_copter
 *
 ******************************************************************************/


#ifndef VELOCITY_CONTROLLER_COPTER_DEFAULT_CONFIG_H_
#define VELOCITY_CONTROLLER_COPTER_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "velocity_controller_copter.h"

static velocity_controller_copter_conf_t velocity_controller_copter_default_config =
{
	.thrust_hover_point = -0.3,
	.pid_config = 
	{
		// -----------------------------------------------------------------
		// ------ X PID -------------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 0.2f,
			.clip_min = -0.5f,
			.clip_max = 0.5f,
			.integrator={
				.gain = 0.1f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 0.1f,
			},
			.differentiator={
				.gain = 0.0f,
				.previous = 0.0f,
				.clip = 0.0f
			},
			.soft_zone_width = 0.2f
		},
		// -----------------------------------------------------------------
		// ------ Y PID ------------------------------------------------
		// -----------------------------------------------------------------
		{
			.p_gain = 0.2f,
			.clip_min = -0.5f,
			.clip_max = 0.5f,
			.integrator={
				.gain = 0.1f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 0.1f,
			},
			.differentiator={
				.gain = 0.0f,
				.previous = 0.0f,
				.clip = 0.0f
			},
			.soft_zone_width = 0.2f
		},
		// ---------------------------------------------------------------------
		// ------ Z PID ---------------------------------------------------
		// ---------------------------------------------------------------------
		{
			.p_gain = 0.20f,
			.clip_min = -0.9f,
			.clip_max = 0.65f,
			.integrator={
				.gain = 0.002f,
				.clip_pre = 1.0f,
				.accumulator = 0.0f,
				.clip = 0.2f,
			},
			.differentiator={
				.gain = 0.04f,
				.previous = 0.0f,
				.clip = 0.04f
			},
			.soft_zone_width = 0.2f
		},
	},
};

#ifdef __cplusplus
}
#endif

#endif // VELOCITY_CONTROLLER_COPTER_DEFAULT_CONFIG_H_