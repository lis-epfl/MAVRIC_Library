/* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file conf_stabilisation_copter.h
 *
 *  Default values for cascade PID controller 
 */


#ifndef CONF_STABILISATION_COPTER_H_
#define CONF_STABILISATION_COPTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stabilisation_copter.h"

static stabiliser_stack_copter_t stabiliser_defaults_copter = 
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
				.p_gain = 0.07f,
				.clip_min = -0.9f, 
				.clip_max = 0.9f,
				.integrator={
					.pregain = 0.5f, 
					.postgain = 1.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.65f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.15f,
					.previous = 0.0f,
					.LPF = 0.5f,
					.maths_clip = 0.65f
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
				.p_gain = 0.07f,
				.clip_min = -0.9f, 
				.clip_max = 0.9f,
				.integrator={
					.pregain = 0.5f, 
					.postgain = 1.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.65f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.15f,
					.previous = 0.0f,
					.LPF = 0.5f,
					.maths_clip = 0.65f
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
				.p_gain = 0.3f,
				.clip_min = -0.3f, 
				.clip_max = 0.3f,
				.integrator={
					.pregain = 0.5f, 
					.postgain = 0.5f,
					.accumulator = 0.0f,
					.maths_clip = 0.3f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.4f,
					.maths_clip = 0.5f
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
				.p_gain = 4.0f,
				.clip_min = -1.2f, 
				.clip_max = 1.2f,
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
					.LPF = 0.5f,
					.maths_clip = 0.1f
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
				.p_gain = 4.0f,
				.clip_min = -1.2f, 
				.clip_max = 1.2f,
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
					.LPF = 0.5f,
					.maths_clip = 0.1f
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
					.pregain = 0.0f, 
					.postgain = 0.0f,
					.accumulator = 0.0f,
					.maths_clip = 0.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.5f,
					.maths_clip = 0.5f
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
// ######  VELOCITY CONTROL  ###################################################
// #############################################################################
	.velocity_stabiliser={
		.rpy_controller={
			// -----------------------------------------------------------------
			// ------ ROLL PID -------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.2f,
				.clip_min = -0.5f, 
				.clip_max = 0.5f,
				.integrator={
					.pregain = 1.0f, 
					.postgain = 0.5f,
					.accumulator = 0.0f,
					.maths_clip = 0.5f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.5f,
					.maths_clip = 0.5f
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
				.p_gain = 0.2f,
				.clip_min = -0.5f, 
				.clip_max = 0.5f,
				.integrator={
					.pregain = 1.0f, 
					.postgain = 0.5f,
					.accumulator = 0.0f,
					.maths_clip = 0.5f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.0f,
					.previous = 0.0f,
					.LPF = 0.5f,
					.maths_clip = 0.5f
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
				.p_gain = 0.25f,
				.clip_min = -0.9f, 
				.clip_max = 0.65f,
				.integrator={
					.pregain = 1.5f, 
					.postgain = 1.0f,
					.accumulator = 0.0f,
					.maths_clip = 1.0f,
					.leakiness = 0.0f
				},
				.differentiator={
					.gain = 0.4f,
					.previous = 0.0f,
					.LPF = 0.97f,
					.maths_clip = 0.2f
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
	},
// #############################################################################
// ######  MISC  ###############################################################
// #############################################################################
	.yaw_coordination_velocity=1.5f
};

#ifdef __cplusplus
}
#endif

#endif /* CONF_STABILISATION_COPTER_H_ */