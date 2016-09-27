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
 * \file torque_controller_ywing.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief Links between torque commands and servos PWM command for Ywing
 *
 ******************************************************************************/


#include "control/servos_mix_ywing.hpp"

Servos_mix_ywing::Servos_mix_ywing(args_t& args, const conf_t& config) :
    flap_top_dir_(config.flap_top_dir),
    flap_right_dir_(config.flap_right_dir),
    flap_left_dir_(config.flap_left_dir),
    min_thrust_(config.min_thrust),
    max_thrust_(config.max_thrust),
    min_deflection_(config.min_deflection),
    max_deflection_(config.max_deflection),
    motor_(args.servo_motor),
    flap_top_(args.servo_flap_top),
    flap_right_(args.servo_flap_right),
    flap_left_(args.servo_flap_left)
{

}


void Servos_mix_ywing::update()
{
    float servos[4];

    // Main motor
    servos[0] = torq_command_.thrust;

    // Clip values
    if (servos[0] < min_thrust_)
    {
        servos[0] = min_thrust_;
    }
    else if (servos[0] > max_thrust_)
    {
        servos[0] = max_thrust_;
    }

    // Top flap
    servos[1] = flap_top_dir_ * (torq_command_.torq[ROLL]
                                     - torq_command_.torq[YAW]);

    // Right flap
    servos[2]  = flap_right_dir_ * (torq_command_.torq[ROLL]
                                        + 0.86f * torq_command_.torq[PITCH]
                                        + 0.50f * torq_command_.torq[YAW]);

    // Left flap
    servos[3]  = flap_left_dir_ * (torq_command_.torq[ROLL]
                                       - 0.86f * torq_command_.torq[PITCH]
                                       + 0.50f * torq_command_.torq[YAW]);

    // Clip values
    for (int32_t i = 1; i < 4; i++)
    {
        if (servos[i] < min_deflection_)
        {
            servos[i] = min_deflection_;
        }
        else if (servos[i] > max_deflection_)
        {
            servos[i] = max_deflection_;
        }
    }

    motor_.write(servos[0]);
    flap_top_.write(servos[1]);
    flap_right_.write(servos[2]);
    flap_left_.write(servos[3]);
}