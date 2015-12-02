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
 * \file conf_stabilisation_hybrid.h
 * 
 * \author MAV'RIC Team
 *   
 * \brief Default values for cascade PID controller 
 *
 ******************************************************************************/


#ifndef CONF_STABILISATION_HYBRID_H_
#define CONF_STABILISATION_HYBRID_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stabilisation_hybrid.h"

static Stabiliser_Stack_hybrid_t stabiliser_defaults_hybrid = 
{
// #############################################################################
// ######  RATE CONTROL  #######################################################
// #############################################################################
	.rate_stabiliser={
		.rpy_controller={
			// -----------------------------------------------------------------
			// ------ ROLL PID -------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 1.0f,
				.clip_min = -1.0f, 
				.clip_max = 1.0f,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			},
			// -----------------------------------------------------------------
			// ------ PITCH PID ------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 3.0f,
				.clip_min = -1.0f, 
				.clip_max = 1.0f,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			},
			// -----------------------------------------------------------------
			// ------ YAW PID --------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 3.0f,
				.clip_min = -1.0f, 
				.clip_max = 1.0f,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			}
		},
		// ---------------------------------------------------------------------
		// ------ THRUST PID ---------------------------------------------------
		// ---------------------------------------------------------------------
		.thrust_controller={
				.p_gain = 1.0f,
				.clip_min = -1000, 
				.clip_max = 1000,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
		},
		// ---------------------------------------------------------------------
		// ------ OUTPUT -------------------------------------------------------
		// ---------------------------------------------------------------------
		.output = {
			.rpy = {0.0f, 0.0f, 0.0f},
			.thrust = 0.0f,
			.tvel = {0.0f, 0.0f, 0.0f},
			.theading = 0.0f,
			.control_mode =  RATE_COMMAND_MODE,
			.yaw_mode = YAW_RELATIVE
			//.yaw_mode = YAW_RELATIVE,
			//.run_mode = MOTORS_OFF
		},
	},
// #############################################################################
// ######  ATTITUDE CONTROL  ###################################################
// #############################################################################
	.attitude_stabiliser={
		.rpy_controller={
			// -----------------------------------------------------------------
			// ------ ROLL PID -------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 1.0f,
				.clip_min = -1000, 
				.clip_max = 1000,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			},
			// -----------------------------------------------------------------
			// ------ PITCH PID ------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 1.5f,
				.clip_min = -1.0f, 
				.clip_max = 1.0f,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			},
			// -----------------------------------------------------------------
			// ------ YAW PID --------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 1.5f,
				.clip_min = -1.0f, 
				.clip_max = 1.0f,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
			}
		},
		// ---------------------------------------------------------------------
		// ------ THRUST PID ---------------------------------------------------
		// ---------------------------------------------------------------------
		.thrust_controller={
				.p_gain = 0.0f,
				.clip_min = -1000, 
				.clip_max = 1000,
				.integrator={
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.0f,
					.maths_clip = 0.0f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f, 
				.dt = 1,
				.soft_zone_width = 0.0f
		},
		// ---------------------------------------------------------------------
		// ------ OUTPUT -------------------------------------------------------
		// ---------------------------------------------------------------------
		.output = {
			.rpy = {0.0f, 0.0f, 0.0f},
			.thrust = 0.0f,
			.tvel = {0.0f, 0.0f, 0.0f},
			.theading = 0.0f,
			.control_mode =  RATE_COMMAND_MODE,
			.yaw_mode = YAW_RELATIVE
			//.yaw_mode = YAW_RELATIVE,
			//.run_mode = MOTORS_OFF
		},
	}
};

#ifdef __cplusplus
}
#endif

#endif /* CONF_STABILISATION_HYBRID_H_ */