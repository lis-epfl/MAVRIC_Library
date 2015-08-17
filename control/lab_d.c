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
 * \file lab_d.c
 * 
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief This file is the file to modify in lab D
 *
 ******************************************************************************/


#include "lab_d.h"
#include "constants.h"
#include "vectors.h"
#include "maths.h"

void lab_d_velocity_vector(float velocity_vector[3], const float joystick_input[3])
{
	// Output
	velocity_vector[X] = 0.0f; // sign * joystick_input[X, Y, Z]
	velocity_vector[Y] = 0.0f;
	velocity_vector[Z] = 0.0f;
}

void lab_d_run_PID(float rpyt_errors[4], const float velocity_cmd[3], const float current_velocity[3], stabiliser_t *stabiliser, float dt)
{
	// rpyt_errors stands for the errors on the
	// roll
	// pitch
	// yaw
	// thrust
	rpyt_errors[X] = 0.0f; // sign * (velocity_cmd[XXX] - current_velocity[XXX]);
	rpyt_errors[Y] = 0.0f;
	rpyt_errors[Z] = 0.0f;
	rpyt_errors[3] = 0.0f;


	// run PID update on all velocity controllers
	stabilisation_run(stabiliser, dt, rpyt_errors);
}

void lab_d_velocity_to_attitude(float rpy_cmd[3], float *thrust_cmd, const float velocity_controller_output[3],const float thrust_output, const float thrust_hover_point)
{
	rpy_cmd[ROLL] = 0.0f; // sign * velocity_controller_output[XXX];
	rpy_cmd[PITCH] = 0.0f;
	*thrust_cmd = 0.0f;
}


void lab_d_direct_to_navigation(float velocity_vector[3], const float goal_position[3], const float current_position[3])
{
	float cruise_speed = 3.0f, desired_speed;
	float relative_position[3];
	float relative_distance;

	// Put your code here

	for (int i = 0; i < 3; ++i)
	{
		velocity_vector[i] = 0.0f;
	}
}
