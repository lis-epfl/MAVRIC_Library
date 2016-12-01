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
 * \file attitude_controller.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief A controller for attitude control
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

#include "control/attitude_controller.hpp"
#include "hal/common/time_keeper.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Attitude_controller::Attitude_controller(const args_t& args, const conf_t& config) :
    ahrs_(args.ahrs),
    attitude_command_(args.attitude_command),
    rate_command_(args.rate_command),
    dt_s_(0.0f),
    last_update_s_(0.0f)
{
    // set initial attitude command
    attitude_command_.s     = 1.0f;
    attitude_command_.v[X]  = 0.0f;
    attitude_command_.v[Y]  = 0.0f;
    attitude_command_.v[Z]  = 0.0f;

    // set initial rate command
    rate_command_.xyz[X]  = 0.0f;
    rate_command_.xyz[Y]  = 0.0f;
    rate_command_.xyz[Z]  = 0.0f;

    // Init rate gains
    pid_controller_init(&pid_[ROLL],  &config.pid_config[ROLL]);
    pid_controller_init(&pid_[PITCH], &config.pid_config[PITCH]);
    pid_controller_init(&pid_[YAW],   &config.pid_config[YAW]);

    // Init attitude error estimator
    attitude_error_estimator_init(&attitude_error_estimator_, &ahrs_);
}


bool Attitude_controller::update(void)
{
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    // Get attitude command
    attitude_error_estimator_set_quat_ref(&attitude_error_estimator_,
                                          attitude_command_);

    // Get local angular errors
    attitude_error_estimator_update(&attitude_error_estimator_);
    float errors[3];
    errors[ROLL]    = attitude_error_estimator_.rpy_errors[ROLL];
    errors[PITCH]   = attitude_error_estimator_.rpy_errors[PITCH];
    errors[YAW]     = attitude_error_estimator_.rpy_errors[YAW];

    // Update PIDs
    rate_command_.xyz[ROLL]  = pid_controller_update_dt(&pid_[ROLL],  errors[ROLL],  dt_s_);
    rate_command_.xyz[PITCH] = pid_controller_update_dt(&pid_[PITCH], errors[PITCH], dt_s_);
    rate_command_.xyz[YAW]   = pid_controller_update_dt(&pid_[YAW],   errors[YAW],   dt_s_);

    return true;
}


bool Attitude_controller::set_command(const attitude_command_t& command)
{
    attitude_command_ = command;
    return true;
}


bool Attitude_controller::get_command(attitude_command_t& command) const
{
    command = attitude_command_;
    return true;
}


bool Attitude_controller::get_output(rate_command_t& command) const
{
    command = rate_command_;
    return true;
}


pid_controller_t& Attitude_controller::get_pid_X(void)
{
    return pid_[X];
}


pid_controller_t& Attitude_controller::get_pid_Y(void)
{
    return pid_[Y];
}


pid_controller_t& Attitude_controller::get_pid_Z(void)
{
    return pid_[Z];
}
