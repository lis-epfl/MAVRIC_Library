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
 * \file rate_controller.hpp
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


#ifndef RATE_CONTROLLER_HPP_
#define RATE_CONTROLLER_HPP_

#include "control/irate_controller.hpp"
#include "control/itorque_controller.hpp"
#include "control/pid_controller.hpp"
#include "util/constants.hpp"
#include "sensing/ahrs.hpp"


template<class TTorque_controller>
class Rate_controller : public IRate_controller, public TTorque_controller
{
public:

    /**
     * \brief Rate controller configuration
     */
    struct conf_t
    {
        pid_controller_conf_t pid_config[3];   ///< Angular rate PID controller for roll, pitch and yaw
        typename TTorque_controller::conf_t torque_controller_config;
    };

    /**
     * \brief Rate controller constructor arguments
     */
    struct args_t
    {
        const ahrs_t& ahrs;
        typename TTorque_controller::args_t torque_controller_args;
    };    

    /**
     * \brief                       Constructor
     *
     * \param   args                containing constructor arguments for rate controller and TTorque_controller
     * \param   config              Configuration     
     */
    Rate_controller(args_t args, conf_t config = default_config());

    virtual void update();

    /*
     * \brief   Set rate command and set controller cascade to "rate mode" 
     * \details Sets the rate_command_ and sets cascade_command_ to point to rate_command, signaling that this is the command mode
     *          This function should NOT be called from higher level controllers if they provide a command, use update_cascade instead
     * \param rate_command  rate command to be set and used to calculate the torque command
     * 
     * \return  success     indicates whether the command was accepted
     */
    bool set_rate_command(const rate_command_t& rate_command);

    static conf_t default_config();

protected:
    /*
     * \brief   calc torque commands based on given rate command and update underlaying cascade level (TTorque)
     * \details Sets the internal rate_command_ to the provided one, without modifiying cascade_command_
     *          Calls calc_torque_command followed by TTorque::update_cascade
     *          This function should be called from higher level controllers if they provide a command
     * \param rate_command  rate command to be set
     */
    void update_cascade(const rate_command_t& rate_command);

    /*
     * \brief   calc torque commands based on given rate command
     * \details Sets the internal rate_command_ to the provided one, without modifiying cascade_command_
     *          This function should be called from higher level controllers if they provide a command
     * \param rate_command  rate_command
     */
    ITorque_controller::torq_command_t calc_torque_command(const rate_command_t& rate_command);


private:
    pid_controller_t            pid_[3];                ///< Angular rate PID controller for roll, pitch and yaw
    float                       dt_s_;                  ///< The time interval between two updates
    float                       last_update_s_;         ///< The time of the last update in s
    const ahrs_t&               ahrs_;                  ///< Ref to attitude estimation (input)
    rate_command_t              rate_command_;          ///< Rate command (input/output)
};

template<class TTorque_controller>
typename Rate_controller<TTorque_controller>::conf_t Rate_controller<TTorque_controller>::default_config()
{
    conf_t conf = {};

    // -----------------------------------------------------------------
    // ------ ROLL RATE PID --------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[ROLL]                           = {};
    conf.pid_config[ROLL].p_gain                    = 0.07f;
    conf.pid_config[ROLL].clip_min                  = -0.9f;
    conf.pid_config[ROLL].clip_max                  = 0.9f;
    conf.pid_config[ROLL].integrator                = {};
    conf.pid_config[ROLL].integrator.gain           = 0.125f;
    conf.pid_config[ROLL].integrator.clip_pre       = 6.0f;
    conf.pid_config[ROLL].integrator.clip           = 0.3f;
    conf.pid_config[ROLL].differentiator            = {};
    conf.pid_config[ROLL].differentiator.gain       = 0.008f;
    conf.pid_config[ROLL].differentiator.clip       = 0.14f;
    conf.pid_config[ROLL].soft_zone_width           = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH RATE PID -------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[PITCH]                          = {};
    conf.pid_config[PITCH].p_gain                   = 0.07f;
    conf.pid_config[PITCH].clip_min                 = -0.9f;
    conf.pid_config[PITCH].clip_max                 = 0.9f;
    conf.pid_config[PITCH].integrator               = {};
    conf.pid_config[PITCH].integrator.gain          = 0.125f,
    conf.pid_config[PITCH].integrator.clip_pre      = 6.0f;
    conf.pid_config[PITCH].integrator.clip          = 0.3f;
    conf.pid_config[PITCH].differentiator           = {};
    conf.pid_config[PITCH].differentiator.gain      = 0.008f;
    conf.pid_config[PITCH].differentiator.clip      = 0.14f;
    conf.pid_config[PITCH].soft_zone_width          = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW RATE PID ---------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[YAW]                            = {};
    conf.pid_config[YAW].p_gain                     = 0.3f;
    conf.pid_config[YAW].clip_min                   = -0.3f;
    conf.pid_config[YAW].clip_max                   = 0.3f;
    conf.pid_config[YAW].integrator                 = {};
    conf.pid_config[YAW].integrator.gain            = 0.075f;
    conf.pid_config[YAW].integrator.clip_pre        = 1.0f;
    conf.pid_config[YAW].integrator.clip            = 0.045f;
    conf.pid_config[YAW].differentiator             = {};
    conf.pid_config[YAW].differentiator.gain        = 0.0f;
    conf.pid_config[YAW].differentiator.clip        = 0.0f;
    conf.pid_config[YAW].soft_zone_width            = 0.0;

    conf.torque_controller_config = TTorque_controller::default_config();

    return conf;
};

#include "control/rate_controller.hxx"

#endif /* RATE_CONTROLLER_HPP_ */
