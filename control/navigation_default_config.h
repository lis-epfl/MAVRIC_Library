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
 * \file navigation_default_config.h
 * 
 * \author MAV'RIC Team
 * 
 * \brief  This file configures the PID gains for the navigation speed command
 *   
 ******************************************************************************/


#ifndef NAVIGATION_DEFAULT_CONFIG_H_
#define NAVIGATION_DEFAULT_CONFIG_H_

#ifdef __cplusplus
	extern "C" {
#endif

#include "pid_controller.h"


navigation_config_t navigation_default_config =
{
	.dist2vel_gain = 0.7f,
	.cruise_speed = 3.0f,
	.max_climb_rate = 1.0f,
	.soft_zone_size = 0.0f,
	.alt_lpf = 0.0f,
	.LPF_gain = 0.9f,
	.wpt_nav_controller = 
	{
		.p_gain = 0.7f,
		.clip_min = 0.0f,
		.clip_max = 3.0f,
		.integrator={
			.gain = 0.0f,
			.clip_pre = 0.0f,
			.accumulator = 0.0f,
			.clip = 0.0f,
		},
		.differentiator={
			.gain = 0.14f,
			.previous = 0.0f,
			.clip = 0.46f
		},
		.output = 0.0f,
		.error = 0.0f,
		.last_update = 0.0f,
		.dt = 1,
		.soft_zone_width = 0.0f
	},
	.hovering_controller = 
	{
		.p_gain = 0.2f,
		.clip_min = 0.0f,
		.clip_max = 3.0f,
		.integrator={
			.gain = 0.0f,
			.clip_pre = 0.0f,
			.accumulator = 0.0f,
			.clip = 0.0f,
		},
		.differentiator={
			.gain = 0.28f,
			.previous = 0.0f,
			.clip = 0.46f
		},
		.output = 0.0f,
		.error = 0.0f,
		.last_update = 0.0f,
		.dt = 1,
		.soft_zone_width = 0.0f
	}
};

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_DEFAULT_CONFIG_H_ */
