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
 * \file rate_controller.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief A controller for rate control
 *
 * \details It takes a command in attitude (roll/pitch/yaw or quaternion) as
 * input, and computes a torque command on roll pitch and yaw.
 * The inner PID loop controls the angular speed around the 3 axis. This inner
 * loop is fed by the outer PID loop which controls the attitude.
 * The error of the outer loop is computed using quaternion arithmetic, and thus
 * avoids gimbal locks as long as the attitude error is smaller than 90 degrees
 * on the pitch axis.
 *
 ******************************************************************************/

#include "control/rate_controller.hpp"
#include "hal/common/time_keeper.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Rate_controller::Rate_controller(const args_t& args, const conf_t& config) :
    ahrs_(args.ahrs),
    rate_command_(args.rate_command),
    torque_command_(args.torque_command),
    dt_s_(0.0f),
    last_update_s_(0.0f)
{
    // set initial rate command
    rate_command_.xyz[X]  = 0.0f;
    rate_command_.xyz[Y]  = 0.0f;
    rate_command_.xyz[Z]  = 0.0f;

    // set initial torque command
    torque_command_.xyz[X]  = 0.0f;
    torque_command_.xyz[Y]  = 0.0f;
    torque_command_.xyz[Z]  = 0.0f;

    // Init rate gains
    pid_controller_init(&pid_[ROLL],  &config.pid_config[ROLL]);
    pid_controller_init(&pid_[PITCH], &config.pid_config[PITCH]);
    pid_controller_init(&pid_[YAW],   &config.pid_config[YAW]);
}


bool Rate_controller::update(void)
{
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    // Get errors on rate
    float errors[3];
    errors[ROLL]  = rate_command_.xyz[ROLL]  - ahrs_.angular_speed()[ROLL];
    errors[PITCH] = rate_command_.xyz[PITCH] - ahrs_.angular_speed()[PITCH];
    errors[YAW]   = rate_command_.xyz[YAW]   - ahrs_.angular_speed()[YAW];

    // Update PIDs
    torque_command_.xyz[ROLL]  = pid_controller_update_dt(&pid_[ROLL],  errors[ROLL],  dt_s_);
    torque_command_.xyz[PITCH] = pid_controller_update_dt(&pid_[PITCH], errors[PITCH], dt_s_);
    torque_command_.xyz[YAW]   = pid_controller_update_dt(&pid_[YAW],   errors[YAW],   dt_s_);

    return true;
}


bool Rate_controller::set_command(const rate_command_t& command)
{
    rate_command_ = command;
    return true;
}


bool Rate_controller::get_command(rate_command_t& command) const
{
    command = rate_command_;
    return true;
}


bool Rate_controller::get_output(torque_command_t& command) const
{
    command = torque_command_;
    return true;
}
