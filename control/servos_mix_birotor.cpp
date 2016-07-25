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
 * \file servos_mix_birotor.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in diagonal configuration
 *
 ******************************************************************************/


#include "control/servos_mix_birotor.hpp"


bool servo_mix_birotor_init(servo_mix_birotor_t* mix, const servo_mix_birotor_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command,  Servo* motor_left, Servo* motor_right, Servo* servo_left, Servo* servo_right, daler_dc_motor_ctrl_t* dc_motors)
{
    // Init dependencies
    mix->torque_command = torque_command;
    mix->thrust_command = thrust_command;
    mix->motor_left     = motor_left;
    mix->motor_right    = motor_right;
    mix->servo_left     = servo_left;
    mix->servo_right    = servo_right;
    mix->dc_motors      = dc_motors;

    // Init parameters
    mix->motor_left_dir     = config->motor_left_dir;
    mix->motor_right_dir    = config->motor_right_dir;
    mix->servo_left_dir     = config->servo_left_dir;
    mix->servo_right_dir    = config->servo_right_dir;

    mix->min_thrust         = config->min_thrust;
    mix->max_thrust         = config->max_thrust;

    mix->min_servo          = config->min_servo;
    mix->max_servo          = config->max_servo;

    return true;
}


bool servos_mix_birotor_update(servo_mix_birotor_t* mix)
{
    //int32_t i;
    float motor[4];

    // torque_command->xyz[0] ==== ROLL
    // torque_command->xyz[1] ==== PITCH
    // torque_command->xyz[2] ==== YAW

    // Motor left
    motor[0] =  mix->thrust_command->thrust +
                (+ mix->torque_command->xyz[2]);

    // Motor right
    motor[1] =  mix->thrust_command->thrust +
                (- mix->torque_command->xyz[2]);

    // Servo left
    motor[2]  = mix->servo_left_dir * (+ mix->torque_command->xyz[0]
                                       + mix->torque_command->xyz[1]);

    // Servo right
    motor[3]  = mix->servo_right_dir * (- mix->torque_command->xyz[0]
                                        + mix->torque_command->xyz[1]);


    // Clip values
    /*for (i=0; i<2; i++)
    {
        if ( motor[i] < mix->min_thrust )
        {
            motor[i] = mix->min_thrust;
        }
        else if ( motor[i] > mix->max_thrust )
        {
            motor[i] = mix->max_thrust;
        }
    }

    int i=0;
    for (i=2; i<4; i++)
    {
        if ( motor[i] < mix->min_servo )
        {
            motor[i] = mix->min_servo;
        }
        else if ( motor[i] > mix->max_servo )
        {
            motor[i] = mix->max_servo;
        }
    }
    */

    mix->motor_left->write(motor[0]);
    mix->motor_right->write(motor[1]);
    mix->servo_left->write(motor[2]);
    mix->servo_right->write(motor[3]);

    mix->dc_motors->wingrons_angle[0] =  motor[2];
    mix->dc_motors->wingrons_angle[1] =  motor[3];

    return true;
}
