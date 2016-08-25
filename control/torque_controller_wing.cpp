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
 * \file torque_controller_wing.cpp
 *
 * \author MAV'RIC Team
 * \author Simon Pyroth
 * \author Basil Huber
 *
 * \brief Links between regulation output and PWM commands for a wing aircraft
 *
 ******************************************************************************/


#include "control/torque_controller_wing.hpp"

Torque_controller_wing::Torque_controller_wing(args_t& args, const conf_t& config) :
    config_(config),
    servo_left_(args.servo_left),
    servo_right_(args.servo_right),
    motor_(args.motor)
    {}


void Torque_controller_wing::update()
{
    // Calculate value to be sent to the motors
    float tmp_right_servo   = config_.servo_right_dir * ( (torq_command_.torq[1] + config_.trim_pitch) + (torq_command_.torq[0] + config_.trim_roll) );
    float tmp_left_servo    = config_.servo_left_dir  * ( (torq_command_.torq[1] + config_.trim_pitch) - (torq_command_.torq[0] + config_.trim_roll) );
    float tmp_motor         = torq_command_.thrust;

    // Clip values
    if (tmp_right_servo < config_.min_amplitude)
    {
        tmp_right_servo = config_.min_amplitude;
    }
    else if (tmp_right_servo > config_.max_amplitude)
    {
        tmp_right_servo = config_.max_amplitude;
    }

    if (tmp_left_servo < config_.min_amplitude)
    {
        tmp_left_servo = config_.min_amplitude;
    }
    else if (tmp_left_servo > config_.max_amplitude)
    {
        tmp_left_servo = config_.max_amplitude;
    }

    if (tmp_motor < config_.min_thrust)
    {
        tmp_motor = config_.min_thrust;
    }
    else if (tmp_motor > config_.max_thrust)
    {
        tmp_motor = config_.max_thrust;
    }

    // Set the calculated values to each motors
    servo_left_.write(tmp_left_servo);
    servo_right_.write(tmp_right_servo);
    motor_.write(tmp_motor);
}
