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
 * \file stabilisation_copter.c *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Nicolas Dousse
 *   
 * \brief This file handles the stabilization of the platform
 *
 ******************************************************************************/


#include "stabilisation_copter.h"
#include "print_util.h"

void stabilisation_copter_init(stabilise_copter_t* stabilisation_copter, stabilise_copter_conf_t* stabiliser_conf, control_command_t* controls, const imu_t* imu, const ahrs_t* ahrs, const position_estimator_t* pos_est,servos_t* servos, const mavlink_stream_t* mavlink_stream)
{
	
	stabilisation_copter->stabiliser_stack = stabiliser_conf->stabiliser_stack;
	stabilisation_copter->controls = controls;
	stabilisation_copter->imu = imu;
	stabilisation_copter->ahrs = ahrs;
	stabilisation_copter->pos_est = pos_est;
	stabilisation_copter->servos = servos;
	
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

	stabilisation_copter->stabiliser_stack.rate_stabiliser.mavlink_stream = mavlink_stream;
	stabilisation_copter->stabiliser_stack.attitude_stabiliser.mavlink_stream = mavlink_stream;
	stabilisation_copter->stabiliser_stack.velocity_stabiliser.mavlink_stream = mavlink_stream;
	stabilisation_copter->stabiliser_stack.position_stabiliser.mavlink_stream = mavlink_stream;
	
	
	print_util_dbg_print("Stabilisation copter init.\r\n");
}

void stabilisation_copter_position_hold(stabilise_copter_t* stabilisation_copter, const control_command_t* input, const mavlink_waypoint_handler_t* waypoint_handler, const position_estimator_t* position_estimator)
{

}

void stabilisation_copter_cascade_stabilise(stabilise_copter_t* stabilisation_copter)
{
	float rpyt_errors[4];
	control_command_t input;
	int32_t i;
	
	// set the controller input
	input= *stabilisation_copter->controls;
	switch (stabilisation_copter->controls->control_mode)
	{
	case VELOCITY_COMMAND_MODE:
		
		// Empty controlle
		
		rpyt_errors[X] = 0.0; // ?
		rpyt_errors[Y] = 0.0; // ?
		rpyt_errors[3] = 0.0; // ?
		rpyt_errors[YAW]= 0.0; //?
		
		// run PID update on all velocity controllers
		//stabilisation_run(&stabilisation_copter->stabiliser_stack.velocity_stabiliser, stabilisation_copter->imu->dt, rpyt_errors);
		
		//input = stabilisation_copter->stabiliser_stack.velocity_stabiliser.output;
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// run absolute attitude_filter controller
		rpyt_errors[0]= input.rpy[0] - ( - stabilisation_copter->ahrs->up_vec.v[1] ); 
		rpyt_errors[1]= input.rpy[1] - stabilisation_copter->ahrs->up_vec.v[0];
		
		if ((stabilisation_copter->controls->yaw_mode == YAW_ABSOLUTE) ) {
			rpyt_errors[2] =maths_calc_smaller_angle(input.theading- stabilisation_copter->pos_est->local_position.heading);
		}
		else
		{ // relative yaw
			rpyt_errors[2]= input.rpy[2];
		}
		
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude_filter controllers
		stabilisation_run(&stabilisation_copter->stabiliser_stack.attitude_stabiliser, stabilisation_copter->imu->dt, rpyt_errors);
		
		// use output of attitude_filter controller to set rate setpoints for rate controller 
		input = stabilisation_copter->stabiliser_stack.attitude_stabiliser.output;
	
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++)
		{
			rpyt_errors[i]= input.rpy[i]- stabilisation_copter->ahrs->angular_speed[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&stabilisation_copter->stabiliser_stack.rate_stabiliser, stabilisation_copter->imu->dt, rpyt_errors);
	}
	
	// mix to servo outputs depending on configuration
	#ifdef CONF_DIAG
	stabilisation_copter_mix_to_servos_diag_quad(&stabilisation_copter->stabiliser_stack.rate_stabiliser.output, stabilisation_copter->servos);
	#else
	#ifdef CONF_CROSS
	stabilisation_copter_mix_to_servos_cross_quad(&stabilisation_copter->stabiliser_stack.rate_stabiliser.output, stabilisation_copter->servos);
	#endif
	#endif
}

void stabilisation_copter_mix_to_servos_diag_quad(control_command_t *control, servos_t* servos)
{
	int32_t i;
	float motor_command[4];
	
	motor_command[M_FRONT_RIGHT]= control->thrust + ( - control->rpy[ROLL] + control->rpy[PITCH]) + M_FR_DIR * control->rpy[YAW];
	motor_command[M_FRONT_LEFT] = control->thrust + ( control->rpy[ROLL] + control->rpy[PITCH]) + M_FL_DIR * control->rpy[YAW];
	motor_command[M_REAR_RIGHT] = control->thrust + ( - control->rpy[ROLL] - control->rpy[PITCH]) + M_RR_DIR * control->rpy[YAW];
	motor_command[M_REAR_LEFT]  = control->thrust + ( control->rpy[ROLL] - control->rpy[PITCH]) + M_RL_DIR * control->rpy[YAW];
	
	for (i=0; i<4; i++)
	{
		if (motor_command[i]<MIN_THRUST)
		{
			motor_command[i]=MIN_THRUST;
		}
		if (motor_command[i]>MAX_THRUST)
		{
			motor_command[i]=MAX_THRUST;
		}
	}

	for (i=0; i<4; i++) 
	{
		servos_set_value( servos, i, motor_command[i]);
	}
}

void stabilisation_copter_mix_to_servos_cross_quad(control_command_t *control, servos_t* servos)
{
	int32_t i;
	float motor_command[4];
	
	motor_command[M_FRONT]= control->thrust + control->rpy[PITCH] + M_FRONT_DIR * control->rpy[YAW];
	motor_command[M_RIGHT] = control->thrust - control->rpy[ROLL] + M_RIGHT_DIR * control->rpy[YAW];
	motor_command[M_REAR] = control->thrust - control->rpy[PITCH] + M_REAR_DIR * control->rpy[YAW];
	motor_command[M_LEFT]  = control->thrust + control->rpy[ROLL] + M_LEFT_DIR * control->rpy[YAW];
	for (i=0; i<4; i++) 
	{
		if (motor_command[i]<MIN_THRUST)
		{
			motor_command[i]=MIN_THRUST;
		}
		if (motor_command[i]>MAX_THRUST)
		{
			motor_command[i]=MAX_THRUST;
		}
	}
	for (i=0; i<4; i++) 
	{
		servos_set_value( servos, i, motor_command[i]);
	}
}