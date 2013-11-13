/*
 * conf_stabilisation_hybrid.h
 *
 * Default values for stabilisation
 * 
 * Created: 13/11/2013 17:46:00
 *  Author: Julien
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
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			},
			// -----------------------------------------------------------------
			// ------ PITCH PID ------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			},
			// -----------------------------------------------------------------
			// ------ YAW PID --------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			}
		},
		// ---------------------------------------------------------------------
		// ------ THRUST PID ---------------------------------------------------
		// ---------------------------------------------------------------------
		.thrust_controller={
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
		},
		// ---------------------------------------------------------------------
		// ------ OUTPUT -------------------------------------------------------
		// ---------------------------------------------------------------------
		.output = {
			.rpy = {0.0, 0.0, 0.0},
			.thrust = 0.0,
			.tvel = {0.0, 0.0, 0.0},
			.theading = 0.0,
			.control_mode =  RATE_COMMAND_MODE,
			.yaw_mode = YAW_RELATIVE,
			.run_mode = MOTORS_OFF
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
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			},
			// -----------------------------------------------------------------
			// ------ PITCH PID ------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			},
			// -----------------------------------------------------------------
			// ------ YAW PID --------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
			}
		},
		// ---------------------------------------------------------------------
		// ------ THRUST PID ---------------------------------------------------
		// ---------------------------------------------------------------------
		.thrust_controller={
				.p_gain = 0.15,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 0.65,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 1.0,
					.clip = 0.65
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.0
		},
		// ---------------------------------------------------------------------
		// ------ OUTPUT -------------------------------------------------------
		// ---------------------------------------------------------------------
		.output = {
			.rpy = {0.0, 0.0, 0.0},
			.thrust = 0.0,
			.tvel = {0.0, 0.0, 0.0},
			.theading = 0.0,
			.control_mode =  RATE_COMMAND_MODE,
			.yaw_mode = YAW_RELATIVE,
			.run_mode = MOTORS_OFF
		},
	}
};


#endif /* CONF_STABILISATION_HYBRID_H_ */