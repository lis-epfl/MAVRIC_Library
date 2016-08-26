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


#ifndef ATTITUDE_CONTROLLER_HPP_
#define ATTITUDE_CONTROLLER_HPP_

#include "control/attitude_error_estimator.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ahrs.hpp"
#include "util/constants.hpp"
#include "control/control_command.h"
#include "control/iattitude_controller.hpp"
//#include "control/irate_controller.hpp"


template<class TRate_controller>
class Attitude_controller : public IAttitude_controller, public TRate_controller
{
public:

    /**
     * \brief Attitude controller configuration
     */
    struct conf_t
    {
        pid_controller_conf_t                pid_config[3];   ///< Attitude PID controller for roll, pitch and yaw
        typename TRate_controller::conf_t    rate_controller_config;    ///< Config for rate control which this controller inherits from
    };

    /**
     * \brief Attitude controller constructor arguments
     */
    struct args_t
    {
        const ahrs_t&                     ahrs;
        typename TRate_controller::args_t rate_controller_args;
    };    

    /**
     * \brief                       Constructor
     *
     * \param   args                Constructor arguments
     * \param   config              Configuration     
     */
    Attitude_controller(args_t args, const conf_t& config = default_config());

    virtual void update();

    bool set_attitude_command(const att_command_t& att_command);

    inline att_command_t attitude_command(){return att_command_;};

    static inline conf_t default_config();

protected:
    /*
     * \brief   calc rate commands based on given attitude command and update underlaying cascade level (TRate)
     * \details Sets the internal attitude_command_ to the provided one, without modifiying cascade_command_
     *          Calls calc_rate_command followed by TRate::update_cascade
     *          This function should be called from higher level controllers if they provide a command
     * \param   attitude_command  attitude command to be set
     */
    void update_cascade(const att_command_t& att_command);

    /*
     * \brief   calc rate commands based on given attitude command
     * \details Sets the internal attitude_command_ to the provided one, without modifiying cascade_command_
     *          This function should be called from higher level controllers if they provide a command
     * \param attitude_command  attitude command
     */
    typename TRate_controller::rate_command_t calc_rate_command(const att_command_t& att_command);

private:
    attitude_error_estimator_t  attitude_error_estimator_;   ///< Attitude error estimator
    pid_controller_t            pid_[3];                     ///< Attitude PID controller for roll, pitch and yaw
    float                       dt_s_;                       ///< The time interval between two updates
    float                       last_update_s_;              ///< The time of the last update in s
    const ahrs_t&               ahrs_;                       ///< Ref to attitude estimation (input)
    att_command_t               att_command_;                  ///< Ref to attitude command (input)
};

template<class TRate_controller>
typename Attitude_controller<TRate_controller>::conf_t Attitude_controller<TRate_controller>::default_config()
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

    conf.rate_controller_config = TRate_controller::default_config();

    return conf;
};

#include "control/attitude_controller.hxx"

#endif /* ATTITUDE_CONTROLLER_HPP_ */
