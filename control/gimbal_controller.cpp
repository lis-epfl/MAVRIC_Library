/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file gimbal_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief   Gimbal control management
 *
 ******************************************************************************/


#include "control/gimbal_controller.hpp"
extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Gimbal_controller::gimbal_controller_mix_to_servos(void)
{
    float pwm_output[3];

    //pwm range = [-1;1] which is set to corresponds to angles [-90°;90°]
    for (int i = 0; i < 3; i++)
    {
    	//print_util_dbg_putfloat(attitude_output_.rpy[i],3);
        pwm_output[i] = attitude_output_.rpy[i] / 90.0f;
    }

	/*print_util_dbg_print("gimbal output\r\n");
	print_util_dbg_print(" pitch ");
	print_util_dbg_putfloat(attitude_output_.rpy[PITCH],3);
	print_util_dbg_print(" val ");
	print_util_dbg_putfloat(pwm_output[PITCH],3);
	print_util_dbg_print(" yaw ");
	print_util_dbg_putfloat(attitude_output_.rpy[YAW],3);
	print_util_dbg_print(" val ");
	print_util_dbg_putfloat(pwm_output[YAW],3);
	print_util_dbg_print("\r\n");*/

    servo_pitch_.write(-pwm_output[PITCH], true); //[-1;1]
    servo_yaw_.write(pwm_output[YAW], true);   //[-1;1]
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Gimbal_controller::Gimbal_controller(Navigation& navigation, Servo& servo_pitch, Servo& servo_yaw, Gimbal_controller::conf_t config):
    navigation_(navigation),
    servo_pitch_(servo_pitch),
    servo_yaw_(servo_yaw)
{
    // Init variables from config
    attitude_command_desired_ 				  = config.attitude_command_desired_config;
    attitude_output_ 						  = config.attitude_output_config;
    attitude_command_range_[MIN_RANGE_GIMBAL] = config.attitude_command_range_config[MIN_RANGE_GIMBAL];
    attitude_command_range_[MAX_RANGE_GIMBAL] = config.attitude_command_range_config[MAX_RANGE_GIMBAL];
}


bool Gimbal_controller::update(Gimbal_controller *gimbal_controller)
{

	attitude_command_t 	att_user_head;
	attitude_command_t	att_mimick_plane;
	float 				semilocal_vel[3];

	// get user head movements
	att_user_head = gimbal_controller->attitude_command_desired_;

	//get semilocal velocity
	gimbal_controller->navigation_.position_estimation_get_semilocal_velocity(semilocal_vel);

	//compute the angle according to the velocity
	att_mimick_plane.rpy[0] = 0.0f;
	att_mimick_plane.rpy[1] = -maths_rad_to_deg(atan2(semilocal_vel[2], semilocal_vel[0])); // - maths_rad_to_deg(aero_attitude.rpy[PITCH]);
	att_mimick_plane.rpy[2] = 0.0f;

	//apply the angle correction only if the quad as a given forward velocity
	if(semilocal_vel[0] < 0.5f) //0.5m/s
		att_mimick_plane.rpy[1] = 0.0f;

	/*print_util_dbg_print("semilocal_vel\r\n");
	print_util_dbg_print(" x ");
	print_util_dbg_putfloat(semilocal_vel[0],3);
	print_util_dbg_print(" y ");
	print_util_dbg_putfloat(semilocal_vel[1],3);
	print_util_dbg_print(" z ");
	print_util_dbg_putfloat(semilocal_vel[2],3);
	print_util_dbg_print(" corr angle ");
	print_util_dbg_putfloat(att_mimick_plane.rpy[1],3);
	print_util_dbg_print("\r\n");*/


    //Clip the desired value and set them as commands value (no controller here)
    for (int i = 0; i < 3; i++)
    {
    	gimbal_controller->attitude_command_desired_.rpy[i] = att_mimick_plane.rpy[i] + att_user_head.rpy[i];

        if (gimbal_controller->attitude_command_desired_.rpy[i] < gimbal_controller->attitude_command_range_[MIN_RANGE_GIMBAL].rpy[i])
        	gimbal_controller->attitude_output_.rpy[i] = gimbal_controller->attitude_command_range_[MIN_RANGE_GIMBAL].rpy[i];
        else if (gimbal_controller->attitude_command_desired_.rpy[i] > gimbal_controller->attitude_command_range_[MAX_RANGE_GIMBAL].rpy[i])
        	gimbal_controller->attitude_output_.rpy[i] = gimbal_controller->attitude_command_range_[MAX_RANGE_GIMBAL].rpy[i];
        else
        	gimbal_controller->attitude_output_.rpy[i] = gimbal_controller->attitude_command_desired_.rpy[i];
    }

    // send attitude output to servos (pwm)
    gimbal_controller->gimbal_controller_mix_to_servos();

    return true;
}
