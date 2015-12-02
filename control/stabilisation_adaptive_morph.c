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
 * \file stabilisation_adaptive_morph.c
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "stabilisation_adaptive_morph.h"
#include "print_util.h"
#include "constants.h"



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool stabilisation_adaptive_morph_init(stabilisation_adaptive_morph_t* stabilisation_adaptive_morph, stabilisation_adaptive_morph_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimation_t* pos_est, const airspeed_analog_t* airspeed_analog, servos_t* servos, servo_mix_adaptive_morph_t* servo_mix)
{
	bool init_success = true;
	
	//init dependencies
	stabilisation_adaptive_morph->stabiliser_stack = stabiliser_conf->stabiliser_stack;
	stabilisation_adaptive_morph->controls = controls;
	stabilisation_adaptive_morph->imu = imu;
	stabilisation_adaptive_morph->ahrs = ahrs;
	stabilisation_adaptive_morph->pos_est = pos_est;
	stabilisation_adaptive_morph->airspeed_analog = airspeed_analog;
	stabilisation_adaptive_morph->servos = servos;
	stabilisation_adaptive_morph->servo_mix = servo_mix;
	stabilisation_adaptive_morph->thrust_apriori = stabiliser_conf->thrust_apriori;
	stabilisation_adaptive_morph->tuning = stabiliser_conf->tuning;
	stabilisation_adaptive_morph->tuning_axis = stabiliser_conf->tuning_axis;
	stabilisation_adaptive_morph->tuning_steps = stabiliser_conf->tuning_steps;
	stabilisation_adaptive_morph->pitch_up = stabiliser_conf->pitch_up;
	stabilisation_adaptive_morph->pitch_down = stabiliser_conf->pitch_down;
	stabilisation_adaptive_morph->roll_right = stabiliser_conf->roll_right;
	stabilisation_adaptive_morph->roll_left = stabiliser_conf->roll_left;
	
	//init controller
	controls->control_mode = ATTITUDE_COMMAND_MODE;
	controls->yaw_mode = YAW_RELATIVE;
	
	controls->rpy[ROLL] = 0.0f;
	controls->rpy[PITCH] = 0.0f;
	controls->rpy[YAW] = 0.0f;
	controls->tvel[X] = 0.0f;
	controls->tvel[Y] = 0.0f;
	controls->tvel[Z] = 0.0f;
	controls->theading = 0.0f;
	controls->thrust = -1.0f;

	print_util_dbg_print("[STABILISATION adaptive_morph] initalised.\r\n");
	
	return init_success;
}

void stabilisation_adaptive_morph_cascade_stabilise(stabilisation_adaptive_morph_t* stabilisation_adaptive_morph)
{
	float rpyt_errors[4];
	control_command_t input;
	int32_t i;
	//quat_t qtmp, q_rot;
	//aero_attitude_t attitude_yaw_inverse;

	// set the controller input
	input= *stabilisation_adaptive_morph->controls;
	switch (stabilisation_adaptive_morph->controls->control_mode) 
	{
	case VELOCITY_COMMAND_MODE:
		
		/*
		attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_adaptive_morph->ahrs->qe);
		attitude_yaw_inverse.rpy[0] = 0.0f;
		attitude_yaw_inverse.rpy[1] = 0.0f;
		attitude_yaw_inverse.rpy[2] = attitude_yaw_inverse.rpy[2];
		
		//qtmp=quaternions_create_from_vector(input.tvel);
		//quat_t input_global = quaternions_local_to_global(stabilisation_adaptive_morph->ahrs->qe, qtmp);
		
		q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
		
		quat_t input_global;
		quaternions_rotate_vector(q_rot, input.tvel, input_global.v);
		
		input.tvel[X] = input_global.v[X];
		input.tvel[Y] = input_global.v[Y];
		input.tvel[Z] = input_global.v[Z];
		
		rpyt_errors[X] = input.tvel[X] - stabilisation_adaptive_morph->pos_est->vel[X];
		rpyt_errors[Y] = input.tvel[Y] - stabilisation_adaptive_morph->pos_est->vel[Y];
		rpyt_errors[3] = -(input.tvel[Z] - stabilisation_adaptive_morph->pos_est->vel[Z]);
		
		if (stabilisation_adaptive_morph->controls->yaw_mode == YAW_COORDINATED)
		{
			float rel_heading_coordinated;
			if ((maths_f_abs(stabilisation_adaptive_morph->pos_est->vel_bf[X])<0.001f)&&(maths_f_abs(stabilisation_adaptive_morph->pos_est->vel_bf[Y])<0.001f))
			{
				rel_heading_coordinated = 0.0f;
			}
			else
			{
				rel_heading_coordinated = atan2(stabilisation_adaptive_morph->pos_est->vel_bf[Y], stabilisation_adaptive_morph->pos_est->vel_bf[X]);
			}
			
			float w = 0.5f * (maths_sigmoid(vectors_norm(stabilisation_adaptive_morph->pos_est->vel_bf)-stabilisation_adaptive_morph->stabiliser_stack.yaw_coordination_velocity) + 1.0f);
			input.rpy[YAW] = (1.0f - w) * input.rpy[YAW] + w * rel_heading_coordinated;
		}

		rpyt_errors[YAW]= input.rpy[YAW];
		
		// run PID update on all velocity controllers
		stabilisation_run(&stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser, stabilisation_adaptive_morph->imu->dt, rpyt_errors);
		
		//velocity_stabiliser.output.thrust = maths_f_min(velocity_stabiliser.output.thrust,stabilisation_param.controls->thrust);
		stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output.thrust += stabilisation_adaptive_morph->thrust_hover_point;
		stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output.theading = input.theading;
		input = stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output;
		
		qtmp=quaternions_create_from_vector(stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output.rpy);
		//quat_t rpy_local = quaternions_global_to_local(stabilisation_adaptive_morph->ahrs->qe, qtmp);
		
		quat_t rpy_local;
		quaternions_rotate_vector(quaternions_inverse(q_rot), qtmp.v, rpy_local.v);
		
		input.rpy[ROLL] = rpy_local.v[Y];
		input.rpy[PITCH] = -rpy_local.v[X];

		if ((!stabilisation_adaptive_morph->pos_est->gps->healthy)||(stabilisation_adaptive_morph->pos_est->state->out_of_fence_2))
		{
			input.rpy[ROLL] = 0.0f;
			input.rpy[PITCH] = 0.0f;
		}

		//input.thrust = stabilisation_adaptive_morph->controls->tvel[Z];
		*/
		
		// Compute velocity errors
		rpyt_errors[0] = 0.0f;
		rpyt_errors[1] = 0.0f;
		rpyt_errors[2] = 0.0f;
		rpyt_errors[3] = input.tvel[X] - stabilisation_adaptive_morph->airspeed_analog->airspeed;
		
		// run PID update on all velocity controllers
		stabilisation_run(&stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser, stabilisation_adaptive_morph->imu->dt, rpyt_errors);
		
		// Add thrust a priori
		stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output.thrust += stabilisation_adaptive_morph->thrust_apriori;
		
		// Set input for next layer
		input.thrust = stabilisation_adaptive_morph->stabiliser_stack.velocity_stabiliser.output.thrust;
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// run absolute attitude_filter controller
		rpyt_errors[0]= input.rpy[0] - ( - stabilisation_adaptive_morph->ahrs->up_vec.v[1] ); 
		rpyt_errors[1]= input.rpy[1] - stabilisation_adaptive_morph->ahrs->up_vec.v[0];
		rpyt_errors[2]= 0.0f;	// Yaw
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude_filter controllers
		stabilisation_run(&stabilisation_adaptive_morph->stabiliser_stack.attitude_stabiliser, stabilisation_adaptive_morph->imu->dt, rpyt_errors);
		
		// use output of attitude_filter controller to set rate setpoints for rate controller 
		input = stabilisation_adaptive_morph->stabiliser_stack.attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++)
		{
			rpyt_errors[i]= input.rpy[i]- stabilisation_adaptive_morph->ahrs->angular_speed[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&stabilisation_adaptive_morph->stabiliser_stack.rate_stabiliser, stabilisation_adaptive_morph->imu->dt, rpyt_errors);
	}
}


