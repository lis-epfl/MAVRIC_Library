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
 * \file rate_controller.hxx
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Basil Huber
 *
 * \brief A controller for rate control, to be use within a cascade controller structure
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

template<class TTorque_controller>
Rate_controller<TTorque_controller>::Rate_controller(args_t args, conf_t config) : 
    TTorque_controller(args.torque_controller_args, config.torque_controller_config),
    ahrs_(args.ahrs)
{
    // Init mode
    dt_s_          = 0.0f;
    last_update_s_ = 0.0f;

    // Init rate gains
    pid_controller_init(&pid_[ROLL],  &config.pid_config[ROLL]);
    pid_controller_init(&pid_[PITCH], &config.pid_config[PITCH]);
    pid_controller_init(&pid_[YAW],   &config.pid_config[YAW]);

    // set initial rate command
    rate_command_t initial_rate_command;
    initial_rate_command.rates = {0.0f,0.0f,0.0f};
    initial_rate_command.thrust = {-1.0f};
    set_rate_command(initial_rate_command);
}


template<class TTorque_controller>
void Rate_controller<TTorque_controller>::update()
{
    /* check whether this is the highest active level of the cascade */
    if(TTorque_controller::cascade_command_ == &rate_command_)
    {
        /* calculate torque_command and propagate down the cascade */
        ITorque_controller::torq_command_t torq_command = calc_torque_command(rate_command_);
        TTorque_controller::update_cascade(torq_command);
    }else
    {
        /* propagate update() down the cascade until reaching the highest active level */
        TTorque_controller::update();
    }
}


template<class TTorque_controller>
bool Rate_controller<TTorque_controller>::set_rate_command(const rate_command_t& rate_command)
{
    rate_command_ = rate_command;
    TTorque_controller::cascade_command_ = &rate_command_;
    return true;
}


//------------------------------------------------------------------------------
// PROTECTED FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

template<class TTorque_controller>
void Rate_controller<TTorque_controller>::update_cascade(const rate_command_t& rate_command)
{
    rate_command_ = rate_command;
    ITorque_controller::torq_command_t torq_command = calc_torque_command(rate_command);
    TTorque_controller::update_cascade(torq_command);
}

template<class TTorque_controller>
ITorque_controller::torq_command_t Rate_controller<TTorque_controller>::calc_torque_command(const rate_command_t& rate_command)
{
    float now      = time_keeper_get_s();
    dt_s_          = now - last_update_s_;
    last_update_s_ = now;

    float errors[3];
    ITorque_controller::torq_command_t torque_command;

    // Get errors on rate
    errors[ROLL]  = rate_command.rates[ROLL]  - ahrs_.angular_speed[ROLL];
    errors[PITCH] = rate_command.rates[PITCH] - ahrs_.angular_speed[PITCH];
    errors[YAW]   = rate_command.rates[YAW]   - ahrs_.angular_speed[YAW];

    // Update PIDs
    torque_command.torq[ROLL]  = pid_controller_update_dt(&pid_[ROLL],  errors[ROLL],  dt_s_);
    torque_command.torq[PITCH] = pid_controller_update_dt(&pid_[PITCH], errors[PITCH], dt_s_);
    torque_command.torq[YAW]   = pid_controller_update_dt(&pid_[YAW],   errors[YAW],   dt_s_);

    torque_command.thrust = rate_command.thrust;

    return torque_command;
}