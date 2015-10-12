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
 * \file stabilisation_wing.c
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "stabilisation_wing.h"
#include "print_util.h"
#include "constants.h"



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool stabilisation_wing_init(stabilisation_wing_t* stabilisation_wing, stabilisation_wing_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimation_t* pos_est,servos_t* servos, servo_mix_wing_t* servo_mix)
{
	bool init_success = true;
	
	//init dependencies
	stabilisation_wing->stabiliser_stack = stabiliser_conf->stabiliser_stack;
	stabilisation_wing->controls = controls;
	stabilisation_wing->imu = imu;
	stabilisation_wing->ahrs = ahrs;
	stabilisation_wing->pos_est = pos_est;
	stabilisation_wing->servos = servos;
	stabilisation_wing->servo_mix = servo_mix;
	stabilisation_wing->tuning = stabiliser_conf->tuning;
	stabilisation_wing->tuning_axis = stabiliser_conf->tuning_axis;
	stabilisation_wing->tuning_steps = stabiliser_conf->tuning_steps;
	stabilisation_wing->pitch_up = stabiliser_conf->pitch_up;
	stabilisation_wing->pitch_down = stabiliser_conf->pitch_down;
	stabilisation_wing->roll_up = stabiliser_conf->roll_up;
	stabilisation_wing->roll_down = stabiliser_conf->roll_down;
	
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

	print_util_dbg_print("[STABILISATION wing] initalised.\r\n");
	
	return init_success;
}

void stabilisation_wing_cascade_stabilise(stabilisation_wing_t* stabilisation_wing)
{
	float rpyt_errors[4];
	control_command_t input;
	int32_t i;
	//quat_t qtmp, q_rot;
	//aero_attitude_t attitude_yaw_inverse;

	// set the controller input
	input= *stabilisation_wing->controls;
	switch (stabilisation_wing->controls->control_mode) 
	{
	case VELOCITY_COMMAND_MODE:
		
		/*
		attitude_yaw_inverse = coord_conventions_quat_to_aero(stabilisation_wing->ahrs->qe);
		attitude_yaw_inverse.rpy[0] = 0.0f;
		attitude_yaw_inverse.rpy[1] = 0.0f;
		attitude_yaw_inverse.rpy[2] = attitude_yaw_inverse.rpy[2];
		
		//qtmp=quaternions_create_from_vector(input.tvel);
		//quat_t input_global = quaternions_local_to_global(stabilisation_wing->ahrs->qe, qtmp);
		
		q_rot = coord_conventions_quaternion_from_aero(attitude_yaw_inverse);
		
		quat_t input_global;
		quaternions_rotate_vector(q_rot, input.tvel, input_global.v);
		
		input.tvel[X] = input_global.v[X];
		input.tvel[Y] = input_global.v[Y];
		input.tvel[Z] = input_global.v[Z];
		
		rpyt_errors[X] = input.tvel[X] - stabilisation_wing->pos_est->vel[X];
		rpyt_errors[Y] = input.tvel[Y] - stabilisation_wing->pos_est->vel[Y];
		rpyt_errors[3] = -(input.tvel[Z] - stabilisation_wing->pos_est->vel[Z]);
		
		if (stabilisation_wing->controls->yaw_mode == YAW_COORDINATED) 
		{
			float rel_heading_coordinated;
			if ((maths_f_abs(stabilisation_wing->pos_est->vel_bf[X])<0.001f)&&(maths_f_abs(stabilisation_wing->pos_est->vel_bf[Y])<0.001f))
			{
				rel_heading_coordinated = 0.0f;
			}
			else
			{
				rel_heading_coordinated = atan2(stabilisation_wing->pos_est->vel_bf[Y], stabilisation_wing->pos_est->vel_bf[X]);
			}
			
			float w = 0.5f * (maths_sigmoid(vectors_norm(stabilisation_wing->pos_est->vel_bf)-stabilisation_wing->stabiliser_stack.yaw_coordination_velocity) + 1.0f);
			input.rpy[YAW] = (1.0f - w) * input.rpy[YAW] + w * rel_heading_coordinated;
		}

		rpyt_errors[YAW]= input.rpy[YAW];
		
		// run PID update on all velocity controllers
		stabilisation_run(&stabilisation_wing->stabiliser_stack.velocity_stabiliser, stabilisation_wing->imu->dt, rpyt_errors);
		
		//velocity_stabiliser.output.thrust = maths_f_min(velocity_stabiliser.output.thrust,stabilisation_param.controls->thrust);
		stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.thrust += stabilisation_wing->thrust_hover_point;
		stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.theading = input.theading;
		input = stabilisation_wing->stabiliser_stack.velocity_stabiliser.output;
		
		qtmp=quaternions_create_from_vector(stabilisation_wing->stabiliser_stack.velocity_stabiliser.output.rpy);
		//quat_t rpy_local = quaternions_global_to_local(stabilisation_wing->ahrs->qe, qtmp);
		
		quat_t rpy_local;
		quaternions_rotate_vector(quaternions_inverse(q_rot), qtmp.v, rpy_local.v);
		
		input.rpy[ROLL] = rpy_local.v[Y];
		input.rpy[PITCH] = -rpy_local.v[X];

		if ((!stabilisation_wing->pos_est->gps->healthy)||(stabilisation_wing->pos_est->state->out_of_fence_2))
		{
			input.rpy[ROLL] = 0.0f;
			input.rpy[PITCH] = 0.0f;
		}

		//input.thrust = stabilisation_wing->controls->tvel[Z];
		*/
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// run absolute attitude_filter controller
		rpyt_errors[0]= input.rpy[0] - ( - stabilisation_wing->ahrs->up_vec.v[1] ); 
		rpyt_errors[1]= input.rpy[1] - stabilisation_wing->ahrs->up_vec.v[0];
		
		if ((stabilisation_wing->controls->yaw_mode == YAW_ABSOLUTE) ) 
		{
			rpyt_errors[2] =maths_calc_smaller_angle(input.theading- stabilisation_wing->pos_est->local_position.heading);
		}
		else
		{ // relative yaw
			rpyt_errors[2]= input.rpy[2];
		}
		
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude_filter controllers
		stabilisation_run(&stabilisation_wing->stabiliser_stack.attitude_stabiliser, stabilisation_wing->imu->dt, rpyt_errors);
		
		// Rewrite command on axis to apply manual control on it (used for tuning)
		if(stabilisation_wing->tuning == 2)
		{
			if(stabilisation_wing->tuning_axis == PITCH)
			{
				stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[ROLL] = input.rpy[ROLL];
			}
			else if(stabilisation_wing->tuning_axis == ROLL)
			{
				stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[PITCH] = input.rpy[PITCH];
			}
		}
		
		// use output of attitude_filter controller to set rate setpoints for rate controller 
		input = stabilisation_wing->stabiliser_stack.attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++)
		{
			rpyt_errors[i]= input.rpy[i]- stabilisation_wing->ahrs->angular_speed[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&stabilisation_wing->stabiliser_stack.rate_stabiliser, stabilisation_wing->imu->dt, rpyt_errors);
		
		// Rewrite command on axis to apply manual control on it (used for tuning)
		if(stabilisation_wing->tuning == 1)
		{
			if(stabilisation_wing->tuning_axis == PITCH)
			{
				stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[ROLL] = input.rpy[ROLL];
			}
			else if(stabilisation_wing->tuning_axis == ROLL)
			{
				stabilisation_wing->stabiliser_stack.rate_stabiliser.output.rpy[PITCH] = input.rpy[PITCH];
			}
		}
	}
	
	// Mix to servo outputs
	servos_mix_wing_update(stabilisation_wing->servo_mix);
}


