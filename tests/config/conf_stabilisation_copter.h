/*
 * conf_stabilisation_copter.h
 *
 * Default values for stabilisation
 * 
 * Created: 13/11/2013 17:46:00
 *  Author: Julien
 */

#ifndef CONF_STABILISATION_COPTER_H_
#define CONF_STABILISATION_COPTER_H_

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
				.p_gain = 0.08,
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
					.gain = 0.2,
					.previous = 0.0,
					.LPF = 0.4,
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
				.p_gain = 0.08,
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
					.gain = 0.2,
					.previous = 0.0,
					.LPF = 0.4,
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
				.p_gain = 0.5,
				.clip_min = -0.9, 
				.clip_max = 0.9,
				.integrator={
					.pregain = 0.5, 
					.postgain = 0.5,
					.accumulator = 0.0,
					.clip = 0.15,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.4,
					.clip = 0.5
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
				.p_gain = 1.0,
				.clip_min = -1000, 
				.clip_max = 1000,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.0,
					.clip = 0.0
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
				.p_gain = 1.5,
				.clip_min = -1.2, 
				.clip_max = 1.2,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.5,
					.clip = 0.1
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
				.p_gain = 1.5,
				.clip_min = -1.2, 
				.clip_max = 1.2,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.5,
					.clip = 0.1
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
				.p_gain = 1.5,
				.clip_min = -1.5, 
				.clip_max = 1.5,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.5,
					.clip = 0.5
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
				.p_gain = 1.0,
				.clip_min = -1, 
				.clip_max = 1,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.0,
					.clip = 0.0
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
// ######  VELOCITY CONTROL  ###################################################
// #############################################################################
	.velocity_stabiliser={
		.rpy_controller={
			// -----------------------------------------------------------------
			// ------ ROLL PID -------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.2,
				.clip_min = -0.5, 
				.clip_max = 0.5,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.3,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.5,
					.clip = 0.5
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.3
			},
			// -----------------------------------------------------------------
			// ------ PITCH PID ------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 0.2,
				.clip_min = -0.5, 
				.clip_max = 0.5,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.3,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.5,
					.clip = 0.5
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.3
			},
			// -----------------------------------------------------------------
			// ------ YAW PID --------------------------------------------------
			// -----------------------------------------------------------------
			{
				.p_gain = 1.0,
				.clip_min = -1, 
				.clip_max = 1,
				.integrator={
					.pregain = 0.0, 
					.postgain = 0.0,
					.accumulator = 0.0,
					.clip = 0.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.0,
					.previous = 0.0,
					.LPF = 0.0,
					.clip = 0.0
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
				.p_gain = 0.4,
				.clip_min = -0.9, 
				.clip_max = 0.65,
				.integrator={
					.pregain = 1.5, 
					.postgain = 1.0,
					.accumulator = 0.0,
					.clip = 1.0,
					.leakiness = 0.0
				},
				.differentiator={
					.gain = 0.5,
					.previous = 0.0,
					.LPF = 0.95,
					.clip = 1.0
				},
				.output = 0.0,
				.error = 0.0,
				.last_update = 0.0, 
				.dt = 1,
				.soft_zone_width = 0.2
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
// ######  MISC  ###############################################################
// #############################################################################
	.yaw_coordination_velocity=1.5
};


#endif /* CONF_STABILISATION_COPTER_H_ */