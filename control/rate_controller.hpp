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
 * \brief A controller for rate control
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

#include "control/controller.hpp"
#include "control/pid_controller.hpp"
#include "util/constants.hpp"
#include "sensing/ahrs.hpp"


class Rate_controller : public Controller<rate_command_t, torque_command_t>
{
public:

    /**
     * \brief   Rate controller configuration
     */
    struct conf_t
    {
        pid_controller_conf_t pid_config[3];   ///< Angular rate PID controller for roll, pitch and yaw
    };


    /**
     * \brief   Default Configuration
     *
     * /return  config
     */
    static inline conf_t default_config();


    /**
     * \brief   Required arguments
     */
    struct args_t
    {
        const AHRS&         ahrs;                  ///< Ref to attitude estimation (input)
        rate_command_t&       rate_command;          ///< Reference to rate command (input)
        torque_command_t&     torque_command;        ///< Reference to torque command (output)
    };


    /**
     * \brief                 Constructor
     *
     * \param   args          Required arguments
     * \param   config        Configuration
     */
    Rate_controller(const args_t& args, const conf_t& config = default_config());


    /**
     * \brief   Main update function
     *
     * \return  success
     */
    bool update(void);


    /**
     * \brief   Sets the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool set_command(const rate_command_t& command);


    /**
     * \brief   Returns the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool get_command(rate_command_t& command) const;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    bool get_output(torque_command_t& command) const;


    /**
     * \brief   Gives access to internal pid_controller
     *
     * \return  pid     reference to pid controller
     */
    pid_controller_t& get_pid_X(void);
    pid_controller_t& get_pid_Y(void);
    pid_controller_t& get_pid_Z(void);

private:
    const AHRS&         ahrs_;                  ///< Ref to attitude estimation (input)
    rate_command_t&       rate_command_;          ///< Reference to rate command (input)
    torque_command_t&     torque_command_;        ///< Reference to torque command (output)

    pid_controller_t      pid_[3];                ///< Angular rate PID controller for roll, pitch and yaw
    float                 dt_s_;                  ///< The time interval between two updates
    float                 last_update_s_;         ///< The time of the last update in s
};

Rate_controller::conf_t Rate_controller::default_config()
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

    return conf;
};

#endif /* RATE_CONTROLLER_HPP_ */
