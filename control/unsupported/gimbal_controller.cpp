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
#include "util/print_util.hpp"


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Gimbal_controller::gimbal_controller_mix_to_servos(void)
{
    float pwm_output[3];

    //pwm range = [-1;1] which is set to corresponds to angles [-90�;90�]
    for (int i = 1; i < 3; i++)
    {
    	print_util_dbg_putfloat(attitude_output_.rpy[i],3);
        pwm_output[i] = attitude_output_.rpy[i] / 90.0f;
    }

    servo_pitch_.write(pwm_output[1], true);
    servo_yaw_.write(pwm_output[2], true);
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Gimbal_controller::Gimbal_controller(Servo& servo_pitch, Servo& servo_yaw, const gimbal_controller_conf_t config):
    servo_pitch_(servo_pitch),
    servo_yaw_(servo_yaw)
{
    // Init variables from config
    attitude_command_desired_ 				  = config.attitude_command_desired_config;
    attitude_output_ 						  = config.attitude_output_config;
    attitude_command_range_[MIN_RANGE_GIMBAL] = config.attitude_command_range_config[MIN_RANGE_GIMBAL];
    attitude_command_range_[MAX_RANGE_GIMBAL] = config.attitude_command_range_config[MAX_RANGE_GIMBAL];
}


bool Gimbal_controller::update(void)
{

    //Set directly the desired input as output commands ensuring that they are in the allowed range (no controller here)
    for (int i = 0; i < 3; i++)
    {
        if (attitude_command_desired_.rpy[i] < attitude_command_range_[MIN_RANGE_GIMBAL].rpy[i])
        {
            attitude_output_.rpy[i] = attitude_command_range_[MIN_RANGE_GIMBAL].rpy[i];
        }
        else if (attitude_command_desired_.rpy[i] > attitude_command_range_[MAX_RANGE_GIMBAL].rpy[i])
        {
            attitude_output_.rpy[i] = attitude_command_range_[MAX_RANGE_GIMBAL].rpy[i];
        }
        else
        {
            attitude_output_.rpy[i] = attitude_command_desired_.rpy[i];
        }
    }

    // send attitude output to servos (pwm)
    gimbal_controller_mix_to_servos();

    return true;
}
