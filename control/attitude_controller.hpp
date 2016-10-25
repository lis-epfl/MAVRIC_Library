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
 * \file attitude_controller.hpp
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


#ifndef ATTITUDE_CONTROLLER_HPP_
#define ATTITUDE_CONTROLLER_HPP_

#include "control/attitude_error_estimator.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ahrs.hpp"
#include "util/constants.hpp"
#include "control/control_command.hpp"
#include "control/controller.hpp"


class Attitude_controller : public Controller<attitude_command_t, rate_command_t>
{
public:

    /**
     * \brief Attitude controller configuration
     */
    struct conf_t
    {
        pid_controller_conf_t pid_config[3];   ///< Attitude PID controller for roll, pitch and yaw
    };


    /**
     * \brief   Default Configuration
     *
     * /return  config
     */
    static inline conf_t default_config();


    /**
     * \brief                       Constructor
     *
     * \param   ahrs                Reference to estimated attitude
     * \param   config              Configuration
     */
    Attitude_controller(const ahrs_t& ahrs, const conf_t& config = default_config());


    /**
     * \brief   Main update function
     *
     * \return  success
     */
    bool update();


    /**
     * \brief   Sets the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool set_command(const attitude_command_t& command);


    /**
     * \brief   Returns the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool get_command(attitude_command_t& command) const;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    bool get_output(rate_command_t& command) const;


protected:
    const ahrs_t&               ahrs_;                       ///< Ref to attitude estimation (input)

    attitude_command_t          attitude_command_;           ///< Attitude command
    rate_command_t              rate_command_;               ///< Attitude command

    attitude_error_estimator_t  attitude_error_estimator_;   ///< Attitude error estimator
    pid_controller_t            pid_[3];                     ///< Attitude PID controller for roll, pitch and yaw
    float                       dt_s_;                       ///< The time interval between two updates
    float                       last_update_s_;              ///< The time of the last update in s
};


Attitude_controller::conf_t Attitude_controller::default_config()
{
    conf_t conf = {};

    // -----------------------------------------------------------------
    // ------ ROLL ANGLE PID -------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[ROLL]                          = {};
    conf.pid_config[ROLL].p_gain                   = 4.0f;
    conf.pid_config[ROLL].clip_min                 = -12.0f;
    conf.pid_config[ROLL].clip_max                 = 12.0f;
    conf.pid_config[ROLL].integrator               = {};
    conf.pid_config[ROLL].integrator.gain          = 0.0f;
    conf.pid_config[ROLL].integrator.clip_pre      = 0.0f;
    conf.pid_config[ROLL].integrator.clip          = 0.0f;
    conf.pid_config[ROLL].differentiator           = {};
    conf.pid_config[ROLL].differentiator.gain      = 0.0f;
    conf.pid_config[ROLL].differentiator.clip      = 0.0f;
    conf.pid_config[ROLL].soft_zone_width          = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH ANGLE PID ------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[PITCH]                         = {};
    conf.pid_config[PITCH].p_gain                  = 4.0f;
    conf.pid_config[PITCH].clip_min                = -12.0f;
    conf.pid_config[PITCH].clip_max                = 12.0f;
    conf.pid_config[PITCH].integrator              = {};
    conf.pid_config[PITCH].integrator.gain         = 0.0f;
    conf.pid_config[PITCH].integrator.clip_pre     = 0.0f;
    conf.pid_config[PITCH].integrator.clip         = 0.0f;
    conf.pid_config[PITCH].differentiator          = {};
    conf.pid_config[PITCH].differentiator.gain     = 0.0f;
    conf.pid_config[PITCH].differentiator.clip     = 0.0f;
    conf.pid_config[PITCH].soft_zone_width         = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW ANGLE PID --------------------------------------------
    // -----------------------------------------------------------------
    conf.pid_config[YAW]                           = {};
    conf.pid_config[YAW].p_gain                    = 3.0f;
    conf.pid_config[YAW].clip_min                  = -1.5f;
    conf.pid_config[YAW].clip_max                  = 1.5f;
    conf.pid_config[YAW].integrator                = {};
    conf.pid_config[YAW].integrator.gain           = 0.0f;
    conf.pid_config[YAW].integrator.clip_pre       = 0.0f;
    conf.pid_config[YAW].integrator.clip           = 0.0f;
    conf.pid_config[YAW].differentiator            = {};
    conf.pid_config[YAW].differentiator.gain       = 0.0f;
    conf.pid_config[YAW].differentiator.clip       = 0.0f;
    conf.pid_config[YAW].soft_zone_width           = 0.0f;

    return conf;
};

#endif /* ATTITUDE_CONTROLLER_HPP_ */
