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
 * \file torque_controller_quadcopter.cpp
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

#include "control/torque_controller_quadcopter.hpp"

Torque_controller_quadcopter::Torque_controller_quadcopter(args_t args, const conf_t& config) :
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
    /* set initial torque command */
    torq_command_t initial_torque_command;
    initial_torque_command.torq = {0.0f, 0.0f, 0.0f};
    initial_torque_command.thrust = {-1.0f};
    set_torque_command(initial_torque_command);
}