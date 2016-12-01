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
 * \file position_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Implementation of position controller
 *
 ******************************************************************************/


#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "control/controller.hpp"
#include "control/control_command.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ahrs.hpp"
#include "sensing/ins.hpp"


class Position_controller : public Controller<position_command_t, velocity_command_t>
{
public:

    /**
     * \brief Configuration
     */
    struct conf_t
    {
        float cruise_speed;
        float cruise_dist;
        float max_climb_rate;
		float kp_yaw;
        pid_controller_conf_t pid_config;
    };


    /**
     * \brief   Default configuration
     *
     * \return  config
     */
    static inline conf_t default_config();


    /**
     * \brief   Required arguments
     */
    struct args_t
    {
        const AHRS& ahrs;     ///< Reference to estimated attitude
        const INS& ins;         ///< Reference to estimated speed and position
    };


    /**
     * \brief                 Constructor
     *
     * \param   args          Required arguments
     * \param   config        Configuration
     */
    Position_controller(args_t args, const conf_t& config = default_config());


	/**
     * \brief Main update function
     */
    virtual bool update();


    /**
     * \brief   Sets the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool set_command(const position_command_t& pos);


    /**
     * \brief   Returns the input command
     *
     * \param   command   Input command
     *
     * \return  success
     */
    bool get_command(position_command_t& pos) const;


    /**
     * \brief   Returns the output command
     *
     * \param   command   output command
     *
     * \return  success
     */
    bool get_output(velocity_command_t& vel) const;


    /**
     * \brief   Gives acces to internal pid_controller
     *
     * \return  pid     reference to pid controller
     */
    pid_controller_t& get_pid(void);

private:
    const INS&          ins_;                       /// inertial navigation unit ti use for position estimation
    const AHRS&       ahrs_;                      ///< Attitude estimation

    position_command_t  position_command_;          ///< Position command in local frame; Z coord ONLY VALID if in POS_XYZ
    velocity_command_t  velocity_command_;          ///< Velocity command in local frame;

    pid_controller_t    pid_controller_;            //< position pid controller
    float cruise_speed_;                            ///< Maximal speed when at more than cruise_dist_from the goal
    float cruise_dist_;                             ///< Distance from the goal at which the cruise_speed is reached
    float max_climb_rate_;                          ///< maximal climb rate (velocity is scaled so that output velocity in Z <= max_climb_rate)
    float kp_yaw_;
};


Position_controller::conf_t Position_controller::default_config()
{
    conf_t conf;
    conf.cruise_speed       = 3.0f;
    conf.cruise_dist        = 10.0f;
    conf.max_climb_rate     = 1.0f;
    conf.kp_yaw             = 0.5f;

    conf.pid_config.p_gain                  = 1.0f;
    conf.pid_config.clip_min                = 0.0f;
    conf.pid_config.clip_max                = 3.0f;
    conf.pid_config.integrator.gain         = 0.0f;
    conf.pid_config.integrator.clip_pre     = 0.0f;
    conf.pid_config.integrator.clip         = 0.0f;
    conf.pid_config.differentiator.gain     = 0.0f;
    conf.pid_config.differentiator.clip     = 0.0f;
    conf.pid_config.soft_zone_width         = 0.0f;

    return conf;
};


#endif /* POSITION_CONTROLLER_HPP_ */
