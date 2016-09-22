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
 *
 * \brief A cascaded controller for attitude & rate control.
 *
 ******************************************************************************/


#include "control/attitude_controller.hpp"
#include "util/constants.hpp"
#include "hal/common/time_keeper.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Attitude_controller::Attitude_controller(const ahrs_t& ahrs, const attitude_command_t& attitude_command, rate_command_t& rate_command, torque_command_t& torque_command, conf_t config) :
    ahrs_(ahrs),
    attitude_command_(attitude_command),
    rate_command_(rate_command),
    torque_command_(torque_command)
{
    // Init mode
    dt_s_          = 0.0f;
    last_update_s_ = 0.0f;

    // Init attitude error estimator 
    attitude_error_estimator_init(&attitude_error_estimator_, &ahrs_);

    // Init rate gains
    pid_controller_init(&rate_pid_[ROLL],  &config.rate_pid_config[ROLL]);
    pid_controller_init(&rate_pid_[PITCH], &config.rate_pid_config[PITCH]);
    pid_controller_init(&rate_pid_[YAW],   &config.rate_pid_config[YAW]);

    // Init angle gains
    pid_controller_init(&angle_pid_[ROLL],  &config.angle_pid_config[ROLL]);
    pid_controller_init(&angle_pid_[PITCH], &config.angle_pid_config[PITCH]);
    pid_controller_init(&angle_pid_[YAW],   &config.angle_pid_config[YAW]);
}


void Attitude_controller::update()
{
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    switch (mode_)
    {
        case mode_t::ATTITUDE_RATE:
            update_angles();
            break;

        case mode_t::RATE:
            break;
    }

    update_rates();
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Attitude_controller::update_rates()
{
    float errors[3];

    // Get errors on rate
    errors[ROLL]  = rate_command_.xyz[ROLL]  - ahrs_.angular_speed[ROLL];
    errors[PITCH] = rate_command_.xyz[PITCH] - ahrs_.angular_speed[PITCH];
    errors[YAW]   = rate_command_.xyz[YAW]   - ahrs_.angular_speed[YAW];

    // Update PIDs
    torque_command_.xyz[ROLL]  = pid_controller_update_dt(&rate_pid_[ROLL],  errors[ROLL],  dt_s_);
    torque_command_.xyz[PITCH] = pid_controller_update_dt(&rate_pid_[PITCH], errors[PITCH], dt_s_);
    torque_command_.xyz[YAW]   = pid_controller_update_dt(&rate_pid_[YAW],   errors[YAW],   dt_s_);
}


void Attitude_controller::update_angles()
{
    float errors[3];

    // Get attitude command
    attitude_error_estimator_set_quat_ref(&attitude_error_estimator_,
                                          attitude_command_.quat);

    // Get local angular errors
    attitude_error_estimator_update(&attitude_error_estimator_);
    errors[ROLL]    = attitude_error_estimator_.rpy_errors[ROLL];
    errors[PITCH]   = attitude_error_estimator_.rpy_errors[PITCH];
    errors[YAW]     = attitude_error_estimator_.rpy_errors[YAW];

    // Update PIDs
    rate_command_.xyz[ROLL]  = pid_controller_update_dt(&angle_pid_[ROLL],  errors[ROLL],  dt_s_);
    rate_command_.xyz[PITCH] = pid_controller_update_dt(&angle_pid_[PITCH], errors[PITCH], dt_s_);
    rate_command_.xyz[YAW]   = pid_controller_update_dt(&angle_pid_[YAW],   errors[YAW],   dt_s_);
}