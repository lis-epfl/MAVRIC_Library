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
 * \file stabilisation_wing_default_config.h
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Default values for cascade PID controller 
 *
 ******************************************************************************/


#ifndef STABILISATION_WING_DEFAULT_CONFIG_H_
#define STABILISATION_WING_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation_wing.h"


stabilisation_wing_conf_t stabilisation_wing_default_config =
{
	.thrust_apriori = 0.25f,
	.pitch_angle_apriori = 0.0f,
	.pitch_angle_apriori_gain = -0.64f,
	.max_roll_angle = PI/3.0f,
	.take_off_thrust = 0.5f,
	.take_off_pitch = PI/6.0f,
	.landing_pitch = 0.0f,
	.landing_max_roll = PI/12.0f,
	.stabiliser_stack 	= 
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
					.p_gain = 0.065f,
					.clip_min = -1.0f,
					.clip_max = 1.0f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 6.0f,
						.accumulator = 0.0f,
						.clip = 0.7f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 0.7f
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
					.p_gain = 0.055f,
					.clip_min = -1.0f,
					.clip_max = 1.0f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 6.0f,
						.accumulator = 0.0f,
						.clip = 0.7f,
					},
					.differentiator={
						.gain = 0.000f,
						.previous = 0.0f,
						.clip = 0.7f
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
					.p_gain = 0.0f,
					.clip_min = -1000,
					.clip_max = 1000,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 0.0f
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
					.gain = 0.0f,
					.clip_pre = 0.0f,
					.accumulator = 0.0f,
					.clip = 0.0f,
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.clip = 0.0f
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
					.p_gain = 15.0f,
					.clip_min = -100.0f,
					.clip_max = 100.0f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 4.0f,
					},
					.differentiator={
						.gain = 0.15f,
						.previous = 0.0f,
						.clip = 4.0f
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
					.p_gain = 15.0f,
					.clip_min = -100.0f,
					.clip_max = 100.0f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 4.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 4.0f
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
					.clip_min = -1.5f,
					.clip_max = 1.5f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 0.0f
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
				.clip_min = -1,
				.clip_max = 1,
				.integrator={
					.gain = 0.0f,
					.clip_pre = 0.0f,
					.accumulator = 0.0f,
					.clip = 0.0f,
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.clip = 0.0f
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
		// ######  VELOCITY CONTROL  ###################################################
		// #############################################################################
		.velocity_stabiliser={
			.rpy_controller={
				// -----------------------------------------------------------------
				// ------ ROLL PID -------------------------------------------------
				// -----------------------------------------------------------------
				{
					.p_gain = 1.2f,
					.clip_min = -100.0f,
					.clip_max = 100.0f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 1.0f,
						.accumulator = 0.0f,
						.clip = 0.5f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 1.0f
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
					.p_gain = 0.2f,
					.clip_min = -PI/4.0f,		// To avoid stall
					.clip_max = PI/3.0f,
					.integrator={
						.gain = 0.08f,
						.clip_pre = 100.0f,
						.accumulator = 0.0f,
						.clip = PI/6.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 1.0f
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
					.p_gain = 0.0f,
					.clip_min = -1,
					.clip_max = 1,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 0.0f
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
				.p_gain = 0.20f,
				.clip_min = -1.0f,
				.clip_max = 1.0f,
				.integrator={
					.gain = 0.025f,
					.clip_pre = 2.0f,
					.accumulator = 0.0f,
					.clip = 0.3f,
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.clip = 0.04f
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
		// ######  POSITION CONTROL  ###################################################
		// #############################################################################
		.position_stabiliser={ //TODO check gains before using
			.rpy_controller={
				// -----------------------------------------------------------------
				// ------ ROLL PID -------------------------------------------------
				// -----------------------------------------------------------------
				{
					.p_gain = 0.01f,
					.clip_min = -0.5f,
					.clip_max = 0.5f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.00005f,
						.previous = 0.0f,
						.clip = 0.005f
					},
					.output = 0.0f,
					.error = 0.0f,
					.last_update = 0.0f,
					.dt = 1,
					.soft_zone_width = 0.2f
				},
				// -----------------------------------------------------------------
				// ------ PITCH PID ------------------------------------------------
				// -----------------------------------------------------------------
				{
					.p_gain = 0.01f,
					.clip_min = -0.5f,
					.clip_max = 0.5f,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.00005f,
						.previous = 0.0f,
						.clip = 0.005f
					},
					.output = 0.0f,
					.error = 0.0f,
					.last_update = 0.0f,
					.dt = 1,
					.soft_zone_width = 0.2f
				},
				// -----------------------------------------------------------------
				// ------ YAW PID --------------------------------------------------
				// -----------------------------------------------------------------
				{
					.p_gain = 1.0f,
					.clip_min = -1,
					.clip_max = 1,
					.integrator={
						.gain = 0.0f,
						.clip_pre = 0.0f,
						.accumulator = 0.0f,
						.clip = 0.0f,
					},
					.differentiator={
						.gain = 0.0f,
						.previous = 0.0f,
						.clip = 0.0f
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
				.p_gain = 0.05f,
				.clip_min = -0.9f,
				.clip_max = 0.65f,
				.integrator={
					.gain = 0.00005f,
					.clip_pre = 1.0f,
					.accumulator = 0.0f,
					.clip = 0.025f,
				},
				.differentiator={
					.gain = 0.005f,
					.previous = 0.0f,
					.clip = 0.01f
				},
				.output = 0.0f,
				.error = 0.0f,
				.last_update = 0.0f,
				.dt = 1,
				.soft_zone_width = 0.2f
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
	},
};

#ifdef __cplusplus
}
#endif

#endif /* STABILISATION_WING_DEFAULT_CONFIG_H_ */