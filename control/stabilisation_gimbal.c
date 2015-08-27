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
 * \file stabilisation_gimbal.c
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *   
 * \brief This file handles the stabilization of the gimbal
 *
 ******************************************************************************/

#include "stabilisation_gimbal.h"
#include "constants.h"
#include "print_util.h"
#include "conf_platform.h"

void stabilisation_gimbal(stabilisation_copter_t* stabilisation_copter)
{
	float rpy_errors[3];
	control_command_t input;
	
	// set the controller input
	input = *stabilisation_copter->controls;
	switch (stabilisation_copter->controls->control_mode)
	{	
		case ATTITUDE_COMMAND_MODE:
		{	
			//update errors - the user body frame is considered aligned with the drone body frame
			rpy_errors[PITCH]	= input.gimbal_rpy[PITCH]	- (0); //pitch error (tilt)
			rpy_errors[YAW]		= input.gimbal_rpy[YAW]		- (0); //yaw error (pan)
			
			//run PID update on all attitudes controllers (only pitch and yaw so far)
			gimbal_stabilisation_run(&stabilisation_copter->stabiliser_stack.gimbal_attitude_stabiliser, stabilisation_copter->imu->dt, rpy_errors);
			
			//compute the mix and send to servo outputs
			gimbal_stabilisation_mix_to_servos_quad(&stabilisation_copter->stabiliser_stack.gimbal_attitude_stabiliser.output, stabilisation_copter->servos);
		
			break;
		}
		case RATE_COMMAND_MODE:
		{
			;
			break;
		}
		case VELOCITY_COMMAND_MODE:
		{
			;
		}
	}
}
	
	
void gimbal_stabilisation_mix_to_servos_quad(control_command_t *control, servos_t* servos)
{
	int32_t i;
	float gimbal_servo_command[3]; //only gimbal stabilization in pitch and yaw
	
	gimbal_servo_command[GIMBAL_SERVO_PITCH]	= control->gimbal_rpy[PITCH];
	gimbal_servo_command[GIMBAL_SERVO_YAW]		= control->gimbal_rpy[YAW];
	
	for (i=4; i<6; i++)
	{ 
		servos_set_value( servos, i, gimbal_servo_command[i-4]);
	}
}