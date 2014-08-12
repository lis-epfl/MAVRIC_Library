/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file conf_stabilisation_hybrid.h
 *
 * This file contains the default values for the stabilization
 */

#ifndef CONF_STABILISATION_HYBRID_H_
#define CONF_STABILISATION_HYBRID_H_

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


#endif /* CONF_STABILISATION_HYBRID_H_ */