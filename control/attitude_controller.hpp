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
 *
 * \brief A cascaded controller for attitude & rate control.
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
#include "control/control_command.h"



class Attitude_controller
{
public:

    /**
     * \brief Control mode (attitude or rate)
     */
    enum class mode_t
    {
        ATTITUDE_RATE  = 0,    ///< Angles + rate control
        RATE           = 1,    ///< Rate control only
    };

    /**
     * \brief Attitude controller configuration
     */
    struct conf_t
    {
        mode_t                mode;                 ///< Default mode of controller (rate/attitude)
        pid_controller_conf_t rate_pid_config[3];   ///< Angular rate PID controller for roll, pitch and yaw
        pid_controller_conf_t angle_pid_config[3];  ///< Attitude PID controller for roll, pitch and yaw
    };


    /**
     * \brief                       Constructor
     *
     * \param   ahrs                Pointer to the estimated attitude
     * \param   config              Configuration     
     */
    Attitude_controller(const ahrs_t& ahrs, const attitude_command_t& attitude_command, rate_command_t& rate_command, torque_command_t& torque_command, conf_t config = default_config());

    void update();

    static inline conf_t default_config();

private:
    attitude_error_estimator_t  attitude_error_estimator_;   ///< Attitude error estimator
    pid_controller_t            rate_pid_[3];                ///< Angular rate PID controller for roll, pitch and yaw
    pid_controller_t            angle_pid_[3];               ///< Attitude PID controller for roll, pitch and yaw
    mode_t                      mode_;                       ///< Control mode: Angle (default), or rate
    float                       dt_s_;                       ///< The time interval between two updates
    float                       last_update_s_;              ///< The time of the last update in s
    const ahrs_t&               ahrs_;                       ///< Ref to attitude estimation (input)
    const attitude_command_t&   attitude_command_;           ///< Ref to attitude command (input)
    rate_command_t&             rate_command_;               ///< Ref to rate command (input/output)
    torque_command_t&           torque_command_;             ///< Ref to torque command (output)

    /**
     * \brief               Performs outer stabilisation loop on attitude angles
     */
     void update_angles();

     /**
     * \brief               Performs inner stabilisation loop on angular rate
     */
    void update_rates();
};


Attitude_controller::conf_t Attitude_controller::default_config()
{
    conf_t conf = {};

    conf.mode = mode_t::ATTITUDE_RATE;

    // #########################################################################
    // ######  RATE CONTROL  ###################################################
    // #########################################################################
    // -----------------------------------------------------------------
    // ------ ROLL RATE PID --------------------------------------------
    // -----------------------------------------------------------------
    conf.rate_pid_config[ROLL]                           = {};
    conf.rate_pid_config[ROLL].p_gain                    = 0.03f;
    conf.rate_pid_config[ROLL].clip_min                  = -0.9f;
    conf.rate_pid_config[ROLL].clip_max                  = 0.9f;
    conf.rate_pid_config[ROLL].integrator                = {};
    conf.rate_pid_config[ROLL].integrator.gain           = 0.015f;
    conf.rate_pid_config[ROLL].integrator.clip_pre       = 1.0f;
    conf.rate_pid_config[ROLL].integrator.accumulator    = 0.0f;
    conf.rate_pid_config[ROLL].integrator.clip           = 0.02f;
    conf.rate_pid_config[ROLL].differentiator            = {};
    conf.rate_pid_config[ROLL].differentiator.gain       = 0.0045f;
    conf.rate_pid_config[ROLL].differentiator.previous   = 0.0f;
    conf.rate_pid_config[ROLL].differentiator.clip       = 0.02f;
    conf.rate_pid_config[ROLL].soft_zone_width           = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH RATE PID -------------------------------------------
    // -----------------------------------------------------------------
    conf.rate_pid_config[PITCH]                          = {};
    conf.rate_pid_config[PITCH].p_gain                   = 0.03f;
    conf.rate_pid_config[PITCH].clip_min                 = -0.9f;
    conf.rate_pid_config[PITCH].clip_max                 = 0.9f;
    conf.rate_pid_config[PITCH].integrator               = {};
    conf.rate_pid_config[PITCH].integrator.gain          = 0.015f,
    conf.rate_pid_config[PITCH].integrator.clip_pre      = 1.0f;
    conf.rate_pid_config[PITCH].integrator.accumulator   = 0.0f;
    conf.rate_pid_config[PITCH].integrator.clip          = 0.02f;
    conf.rate_pid_config[PITCH].differentiator           = {};
    conf.rate_pid_config[PITCH].differentiator.gain      = 0.0045f;
    conf.rate_pid_config[PITCH].differentiator.previous  = 0.0f;
    conf.rate_pid_config[PITCH].differentiator.clip      = 0.02f;
    conf.rate_pid_config[PITCH].soft_zone_width          = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW RATE PID ---------------------------------------------
    // -----------------------------------------------------------------
    conf.rate_pid_config[YAW]                            = {};
    conf.rate_pid_config[YAW].p_gain                     = 0.3f;
    conf.rate_pid_config[YAW].clip_min                   = -0.3f;
    conf.rate_pid_config[YAW].clip_max                   = 0.3f;
    conf.rate_pid_config[YAW].integrator                 = {};
    conf.rate_pid_config[YAW].integrator.gain            = 0.15f;
    conf.rate_pid_config[YAW].integrator.clip_pre        = 1.0f;
    conf.rate_pid_config[YAW].integrator.accumulator     = 0.0f;
    conf.rate_pid_config[YAW].integrator.clip            = 0.09f;
    conf.rate_pid_config[YAW].differentiator             = {};
    conf.rate_pid_config[YAW].differentiator.gain        = 0.0f;
    conf.rate_pid_config[YAW].differentiator.previous    = 0.0f;
    conf.rate_pid_config[YAW].differentiator.clip        = 0.0f;
    conf.rate_pid_config[YAW].soft_zone_width            = 0.0;

    // #########################################################################
    // ######  ANGLE CONTROL  ##################################################
    // #########################################################################
    // -----------------------------------------------------------------
    // ------ ROLL ANGLE PID -------------------------------------------
    // -----------------------------------------------------------------
    conf.angle_pid_config[ROLL]                          = {};
    conf.angle_pid_config[ROLL].p_gain                   = 4.0f;
    conf.angle_pid_config[ROLL].clip_min                 = -1.2f;
    conf.angle_pid_config[ROLL].clip_max                 = 1.2f;
    conf.angle_pid_config[ROLL].integrator               = {};
    conf.angle_pid_config[ROLL].integrator.gain          = 0.0f;
    conf.angle_pid_config[ROLL].integrator.clip_pre      = 0.0f;
    conf.angle_pid_config[ROLL].integrator.accumulator   = 0.0f;
    conf.angle_pid_config[ROLL].integrator.clip          = 0.0f;
    conf.angle_pid_config[ROLL].differentiator           = {};
    conf.angle_pid_config[ROLL].differentiator.gain      = 0.0f;
    conf.angle_pid_config[ROLL].differentiator.previous  = 0.0f;
    conf.angle_pid_config[ROLL].differentiator.clip      = 0.0f;
    conf.angle_pid_config[ROLL].soft_zone_width          = 0.0f;
    // -----------------------------------------------------------------
    // ------ PITCH ANGLE PID ------------------------------------------
    // -----------------------------------------------------------------
    conf.angle_pid_config[PITCH]                         = {};
    conf.angle_pid_config[PITCH].p_gain                  = 4.0f;
    conf.angle_pid_config[PITCH].clip_min                = -1.2f;
    conf.angle_pid_config[PITCH].clip_max                = 1.2f;
    conf.angle_pid_config[PITCH].integrator              = {};
    conf.angle_pid_config[PITCH].integrator.gain         = 0.0f;
    conf.angle_pid_config[PITCH].integrator.clip_pre     = 0.0f;
    conf.angle_pid_config[PITCH].integrator.accumulator  = 0.0f;
    conf.angle_pid_config[PITCH].integrator.clip         = 0.0f;
    conf.angle_pid_config[PITCH].differentiator          = {};
    conf.angle_pid_config[PITCH].differentiator.gain     = 0.0f;
    conf.angle_pid_config[PITCH].differentiator.previous = 0.0f;
    conf.angle_pid_config[PITCH].differentiator.clip     = 0.0f;
    conf.angle_pid_config[PITCH].soft_zone_width         = 0.0f;
    // -----------------------------------------------------------------
    // ------ YAW ANGLE PID --------------------------------------------
    // -----------------------------------------------------------------
    conf.angle_pid_config[YAW]                           = {};
    conf.angle_pid_config[YAW].p_gain                    = 3.0f;
    conf.angle_pid_config[YAW].clip_min                  = -1.5f;
    conf.angle_pid_config[YAW].clip_max                  = 1.5f;
    conf.angle_pid_config[YAW].integrator                = {};
    conf.angle_pid_config[YAW].integrator.gain           = 0.0f;
    conf.angle_pid_config[YAW].integrator.clip_pre       = 0.0f;
    conf.angle_pid_config[YAW].integrator.accumulator    = 0.0f;
    conf.angle_pid_config[YAW].integrator.clip           = 0.0f;
    conf.angle_pid_config[YAW].differentiator            = {};
    conf.angle_pid_config[YAW].differentiator.gain       = 0.0f;
    conf.angle_pid_config[YAW].differentiator.previous   = 0.0f;
    conf.angle_pid_config[YAW].differentiator.clip       = 0.0f;
    conf.angle_pid_config[YAW].soft_zone_width           = 0.0f;

    return conf;
};

#endif /* ATTITUDE_CONTROLLER_HPP_ */
