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
 * \file stabilisation_hybrid.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *   
 * \brief This file handles the stabilization of hybrid platforms
 *
 ******************************************************************************/


#include "stabilisation_hybrid.h"
#include "central_data.h"
#include "conf_stabilisation_hybrid.h"
#include "quick_trig.h"

central_data_t *central_data;

void stabilisation_hybrid_init(Stabiliser_Stack_hybrid_t* stabiliser_stack)
{
	central_data = central_data_get_pointer_to_struct();
	central_data->run_mode = MOTORS_OFF;
	central_data->controls.control_mode = RATE_COMMAND_MODE;

	*stabiliser_stack = stabiliser_defaults_hybrid;
}

void stabilisation_hybrid_cascade_stabilise_hybrid(imu_t *imu, position_estimation_t *pos_est, control_command_t *control_input)
{
	float rpyt_errors[4];
	control_command_t input;
	int32_t i;
	
	// set the controller input
	input = *control_input;

	float target_global[3];
	float target_loc[3];
	float reference_loc[3];

	switch (control_input->control_mode) {
	case VELOCITY_COMMAND_MODE:
		/*
		 * Disabled for now
		 */
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case ATTITUDE_COMMAND_MODE:
		// reference vector	in local frame
		reference_loc[0] = 1.0f;	// front vector
		reference_loc[1] = 0.0f;
		reference_loc[2] = 0.0f;	// norm = 1

		// get target vector in global frame
		target_global[0] = input.rpy[1];
		target_global[1] = input.rpy[2];
		target_global[2] = -1;

		// target vector in local frame
		quat_t qtarget = maths_quat_from_vector(&target_global);
		qtarget = maths_quat_global_to_local(imu->attitude.qe, qtarget);
		target_loc[0] = qtarget.v[0];
		target_loc[1] = qtarget.v[1];
		target_loc[2] = qtarget.v[2];
		maths_vector_normalize(target_loc, target_loc);

		// get rotation axis
		float axis[3];
		maths_cross_product(reference_loc, target_loc, axis);
		maths_vector_normalize(axis, axis);

		// get angle
		float angle = acosf(maths_scalar_product(reference_loc, target_loc));
		// float angle = quick_trig_acos(maths_scalar_product(reference_loc, target_loc));
		
		// get errors
		rpyt_errors[0]= input.rpy[0];
		rpyt_errors[1]= axis[1] * angle;
		rpyt_errors[2]= axis[2] * angle;
		
		rpyt_errors[3]= input.thrust;       // no feedback for thrust at this level
		
		// run PID update on all attitude controllers
		stabilisation_run(&central_data->stabiliser_stack.attitude_stabiliser, central_data->imu.dt, &rpyt_errors);
		
		// use output of attitude controller to set rate setpoints for rate controller 
		input = central_data->stabiliser_stack.attitude_stabiliser.output;
		
	// -- no break here  - we want to run the lower level modes as well! -- 
	
	case RATE_COMMAND_MODE: // this level is always run
		// get rate measurements from IMU (filtered angular rates)
		for (i=0; i<3; i++) {
			rpyt_errors[i]= input.rpy[i] - imu->attitude.om[i];
		}
		rpyt_errors[3] = input.thrust ;  // no feedback for thrust at this level
		
		// run PID update on all rate controllers
		stabilisation_run(&central_data->stabiliser_stack.rate_stabiliser, central_data->imu.dt, &rpyt_errors );
	}

	// mix to servos 
	stabilisation_hybrid_mix_to_servos_xwing(&central_data->stabiliser_stack.rate_stabiliser.output);
}

void stabilisation_hybrid_mix_to_servos_xwing(control_command_t *control)
{
	int32_t i;
	float motor_command;
	float servo_command[4];

	float FLAP_FRONT_DIR 	= 1;
	float FLAP_RIGHT_DIR 	= 1;
	float FLAP_REAR_DIR 	= 1;
	float FLAP_LEFT_DIR 	= 1;

	int MAIN_ENGINE = 4;
	int FLAP_FRONT 	= 1;
	int FLAP_RIGHT 	= 2;
	int FLAP_REAR 	= 3;
	int FLAP_LEFT 	= 0;

	// mix
	motor_command = control->thrust;
	servo_command[FLAP_FRONT] = FLAP_FRONT_DIR * ( + control->rpy[ROLL] 
												   + control->rpy[YAW] );
	servo_command[FLAP_RIGHT] = FLAP_RIGHT_DIR * ( + control->rpy[ROLL] 
												   + control->rpy[PITCH] ); 
	servo_command[FLAP_REAR] = FLAP_REAR_DIR * ( + control->rpy[ROLL] 
												 - control->rpy[YAW] );
	servo_command[FLAP_LEFT] = FLAP_LEFT_DIR * ( + control->rpy[ROLL] 
												 - control->rpy[PITCH] ); 

	// maths_clip
	if (motor_command < MIN_THRUST) motor_command = MIN_THRUST;
	if (motor_command > MAX_THRUST) motor_command = MAX_THRUST;
	for (i=0; i<4; i++) 
	{
		if (servo_command[i] < -1.0f) 
		{
			servo_command[i] = -1.0f;
		}
		if (servo_command[i] > 1.0f)
		{
			servo_command[i] = 1.0f;
		}
	}

	// scale and write values
	central_data->servos[FLAP_FRONT].value 	= servo_command[FLAP_FRONT];
	central_data->servos[FLAP_RIGHT].value 	= servo_command[FLAP_RIGHT];
	central_data->servos[FLAP_REAR].value 	= servo_command[FLAP_REAR];
	central_data->servos[FLAP_LEFT].value 	= servo_command[FLAP_LEFT];
	central_data->servos[MAIN_ENGINE].value = motor_command;
}