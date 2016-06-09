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
 * \file servos_mix_wing.cpp
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Links between regulation output and PWM commands for a wing aircraft
 *
 ******************************************************************************/


#include "control/servos_mix_wing.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/constants.h"
}

bool servos_mix_wing_init(servos_mix_wing_t* mix, 
                            const servos_mix_wing_conf_t config, 
                            const torque_command_t* torque_command,
                            const thrust_command_t* thrust_command,
                            Servo* servo_left,
                            Servo* servo_right,
                            Servo* motor)
{
    bool init_success = true;
    
    // Init dependencies
    mix->torque_command     = torque_command;
    mix->thrust_command     = thrust_command;
    mix->servo_left         = servo_left;
    mix->servo_right        = servo_right;
    mix->motor          = motor;

    // Init parameters
    mix->config.servo_right     = config.servo_right;
    mix->config.servo_left      = config.servo_left;
    mix->config.motor           = config.motor;
    
    mix->config.servo_right_dir = config.servo_right_dir;
    mix->config.servo_left_dir  = config.servo_left_dir;
    
    mix->config.min_amplitude   = config.min_amplitude;
    mix->config.max_amplitude   = config.max_amplitude;
    mix->config.min_thrust      = config.min_thrust;
    mix->config.max_thrust      = config.max_thrust;
    
    mix->config.trim_roll       = config.trim_roll;
    mix->config.trim_pitch      = config.trim_pitch;
    
    // Debug and return
    print_util_dbg_print("[SERVOS MIX WING] initialised \r\n");
    return init_success;
}


void servos_mix_wing_update(servos_mix_wing_t* mix)
{
    // Calculate value to be sent to the motors
    float tmp_right_servo   = mix->config.servo_right_dir * ( (mix->torque_command->xyz[1] + mix->config.trim_pitch) + (mix->torque_command->xyz[0] + mix->config.trim_roll) );
    float tmp_left_servo    = mix->config.servo_left_dir  * ( (mix->torque_command->xyz[1] + mix->config.trim_pitch) - (mix->torque_command->xyz[0] + mix->config.trim_roll) );
    float tmp_motor         = mix->thrust_command->thrust;
    
    // Clip values
    if (tmp_right_servo < mix->config.min_amplitude)
    {
        tmp_right_servo = mix->config.min_amplitude;
    }
    else if (tmp_right_servo > mix->config.max_amplitude)
    {
        tmp_right_servo = mix->config.max_amplitude;
    }
    
    if (tmp_left_servo < mix->config.min_amplitude)
    {
        tmp_left_servo = mix->config.min_amplitude;
    }
    else if (tmp_left_servo > mix->config.max_amplitude)
    {
        tmp_left_servo = mix->config.max_amplitude;
    }
    
    if (tmp_motor < mix->config.min_thrust)
    {
        tmp_motor = mix->config.min_thrust;
    }
    else if (tmp_motor > mix->config.max_thrust)
    {
        tmp_motor = mix->config.max_thrust;
    }

    // Set the calculated values to each motors
    mix->servo_left->write(tmp_left_servo);
    mix->servo_right->write(tmp_right_servo);
    mix->motor->write(tmp_motor);
    
}