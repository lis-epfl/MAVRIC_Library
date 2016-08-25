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
 * \file attitude_controller.hxx
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief A controller for attitude control, to be use within a cascade controller structure
 *
 * TODO: update this text
 * \details It takes a command in attitude (roll/pitch/yaw or quaternion) as
 * input, and computes a torque command on roll pitch and yaw.
 * The inner PID loop controls the angular speed around the 3 axis. This inner
 * loop is fed by the outer PID loop which controls the attitude.
 * The error of the outer loop is computed using quaternion arithmetic, and thus
 * avoids gimbal locks as long as the attitude error is smaller than 90 degrees
 * on the pitch axis.
 *
 ******************************************************************************/

#include "hal/common/time_keeper.hpp"

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TRate_controller>
Attitude_controller<TRate_controller>::Attitude_controller(args_t args, conf_t config) : 
    TRate_controller(args.rate_controller_args, config.rate_controller_config),
    ahrs_(args.ahrs)
{
    // Init mode
    dt_s_          = 0.0f;
    last_update_s_ = 0.0f;

    // Init rate gains
    pid_controller_init(&pid_[ROLL],  &config.pid_config[ROLL]);
    pid_controller_init(&pid_[PITCH], &config.pid_config[PITCH]);
    pid_controller_init(&pid_[YAW],   &config.pid_config[YAW]);

    // Init attitude error estimator 
    attitude_error_estimator_init(&attitude_error_estimator_, &ahrs_);


    // set initial attitude command
    att_command_t initial_command;
    float initial_rpy[3] = {0.0f, 0.0f, 0.0f};
    initial_command.att = coord_conventions_quaternion_from_rpy(initial_rpy);
    initial_command.thrust = -1.0f;
    set_attitude_command(initial_command);
}


template<class TRate_controller>
void Attitude_controller<TRate_controller>::update()
{
    /* check whether this is the highest active level of the cascade */
    if(TRate_controller::cascade_command_ == &att_command_)
    {
        /* calculate torque_command and propagate down the cascade */
        typename TRate_controller::rate_command_t rate_command = calc_rate_command(att_command_);
        TRate_controller::update_cascade(rate_command);
    }else
    {
        /* propagate update() down the cascade until reaching the highest active level */
        TRate_controller::update();
    }
}


template<class TRate_controller>
bool Attitude_controller<TRate_controller>::set_attitude_command(const att_command_t& att_command)
{
    att_command_ = att_command;
    TRate_controller::cascade_command_ = &att_command_;
    return true;
}


//------------------------------------------------------------------------------
// PROTECTED FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TRate_controller>
void Attitude_controller<TRate_controller>::update_cascade(const att_command_t& att_command)
{
    att_command_ = att_command;
    typename TRate_controller::rate_command_t rate_command = calc_rate_command(att_command);
    TRate_controller::update_cascade(rate_command);
}

template<class TRate_controller>
typename TRate_controller::rate_command_t Attitude_controller<TRate_controller>::calc_rate_command(const att_command_t& att_command)
{
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    float errors[3];
    typename TRate_controller::rate_command_t rate_command;

    // Get attitude command
    attitude_error_estimator_set_quat_ref(&attitude_error_estimator_,
                                          att_command.att);

    // Get local angular errors
    attitude_error_estimator_update(&attitude_error_estimator_);
    errors[ROLL]    = attitude_error_estimator_.rpy_errors[ROLL];
    errors[PITCH]   = attitude_error_estimator_.rpy_errors[PITCH];
    errors[YAW]     = attitude_error_estimator_.rpy_errors[YAW];

    // Update PIDs
    rate_command.rates[ROLL]  = pid_controller_update_dt(&pid_[ROLL],  errors[ROLL],  dt_s_);
    rate_command.rates[PITCH] = pid_controller_update_dt(&pid_[PITCH], errors[PITCH], dt_s_);
    rate_command.rates[YAW]   = pid_controller_update_dt(&pid_[YAW],   errors[YAW],   dt_s_);

    rate_command.thrust = att_command.thrust;
    return rate_command;
}