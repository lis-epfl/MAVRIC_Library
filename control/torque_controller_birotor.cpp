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
 * \file torque_controller_birotor.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for birotoers
 *
 ******************************************************************************/


#include "control/torque_controller_birotor.hpp"


Torque_controller_birotor::Torque_controller_birotor(args_t& args, const conf_t& config) : 
    motor_left_dir_(config.motor_left_dir),
    motor_right_dir_(config.motor_right_dir),
    servo_left_dir_(config.servo_left_dir),
    servo_right_dir_(config.servo_right_dir),
    min_servo_(config.min_servo),
    max_servo_(config.max_servo),
    min_thrust_(config.min_thrust),
    max_thrust_(config.max_thrust),
    motor_left_(args.motor_left),
    motor_right_(args.motor_right),
    servo_left_(args.servo_left),
    servo_right_(args.servo_right),
    dc_motors_(args.dc_motors)
{

}



void Torque_controller_birotor::update()
{
    float motor[4];

    // torque_command->xyz[0] ==== ROLL
    // torque_command->xyz[1] ==== PITCH
    // torque_command->xyz[2] ==== YAW

    // Motor left
    motor[0] =  torq_command_.thrust +
                (+ torq_command_.torq[2]);

    // Motor right
    motor[1] =  torq_command_.thrust +
                (- torq_command_.torq[2]);

    // Servo left
    motor[2]  = mix->servo_left_dir * (+ torq_command_.torq[0]
                                       + torq_command_.torq[1]);

    // Servo right
    motor[3]  = mix->servo_right_dir * (- torq_command_.torq[0]
                                        + torq_command_.torq[1]);


    // Clip values
    /*for (i=0; i<2; i++)
    {
        if ( motor[i] < min_thrust_ )
        {
            motor[i] = min_thrust_;
        }
        else if ( motor[i] > max_thrust_ )
        {
            motor[i] = max_thrust_;
        }
    }

    int i=0;
    for (i=2; i<4; i++)
    {
        if ( motor[i] < min_servo_ )
        {
            motor[i] = min_servo_;
        }
        else if ( motor[i] > max_servo_ )
        {
            motor[i] = max_servo_;
        }
    }
    */

    motor_left.write(motor[0]);
    motor_right.write(motor[1]);
    servo_left.write(motor[2]);
    servo_right.write(motor[3]);

    dc_motors.wingrons_angle[0] =  motor[2];
    dc_motors.wingrons_angle[1] =  motor[3];

    return true;
}
