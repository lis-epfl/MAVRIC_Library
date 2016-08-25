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
 * \file torque_controller_quadcopter_diag.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in diagonal configuration
 *
 ******************************************************************************/

#include "control/torque_controller_quadcopter_diag.hpp"

#include "util/print_util.h"



Torque_controller_quadcopter_diag::Torque_controller_quadcopter_diag(args_t& args, const conf_t& config) : 
    motor_rear_left_dir_(config.motor_rear_left_dir),
    motor_front_left_dir_(config.motor_front_left_dir),
    motor_front_right_dir_(config.motor_front_right_dir),
    motor_rear_right_dir_(config.motor_rear_right_dir),
    min_thrust_(config.min_thrust),
    max_thrust_(config.max_thrust),
    motor_rear_left_(args.motor_rear_left),
    motor_front_left_(args.motor_front_left),
    motor_front_right_(args.motor_front_right),
    motor_rear_right_(args.motor_rear_right)
{

}

void Torque_controller_quadcopter_diag::update()
{
    float motor[4];

    // print_util_dbg_print("thrust: ");
    // print_util_dbg_putfloat(torq_command_.thrust,3);
    // print_util_dbg_print("\n");

    // Front Right motor
    motor[0] =  torq_command_.thrust +
                (- torq_command_.torq[0]) +
                (+ torq_command_.torq[1]) +
                motor_front_right_dir_ * torq_command_.torq[2];

    // Front Left motor
    motor[1] =  torq_command_.thrust +
                (+ torq_command_.torq[0]) +
                (+ torq_command_.torq[1]) +
                motor_front_left_dir_ * torq_command_.torq[2];

    // Rear Right motor
    motor[2]  = torq_command_.thrust +
                (- torq_command_.torq[0]) +
                (- torq_command_.torq[1]) +
                motor_rear_right_dir_ * torq_command_.torq[2];

    // Rear Left motor
    motor[3]  = torq_command_.thrust +
                (+ torq_command_.torq[0]) +
                (- torq_command_.torq[1]) +
                motor_rear_left_dir_ * torq_command_.torq[2];

    // Clip values
    for (int32_t i = 0; i < 4; i++)
    {
        if (motor[i] < min_thrust_)
        {
            motor[i] = min_thrust_;
        }
        else if (motor[i] > max_thrust_)
        {
            motor[i] = max_thrust_;
        }
    }

    motor_front_right_.write(motor[0]);
    motor_front_left_.write(motor[1]);
    motor_rear_right_.write(motor[2]);
    motor_rear_left_.write(motor[3]);
}
