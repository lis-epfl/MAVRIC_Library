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
 * \brief Basic controller for position and yaw
 *
 ******************************************************************************/


#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "control/pid_controller.hpp"
#include "control/stabilisation.hpp"
#include "sensing/ins.hpp"

class Position_controller
{
public:

    struct conf_t
    {
        pid_controller_conf_t cruise_pid_config;
        pid_controller_conf_t hover_pid_config;
        float max_climb_rate;
        float max_rel_yaw;
        float min_cruise_dist;
    };

    Position_controller(control_command_t& vel_command, const INS& ins, const quat_t& qe, const conf_t& config = default_config());

    void update();

    void set_command(local_position_t pos_command_lf, float yaw_command_lf);
    void set_position_command(local_position_t pos_command_lf);
    void set_yaw_command(float yaw_command_lf);

    /**
     * \brief   default configuration for navigation
     *
     * \return default config
     */
    static inline conf_t default_config();

private:

    void set_cruise_mode(bool cruise_mode);

    control_command_t&  vel_command_lf_;
    local_position_t    pos_command_lf_;
    float               yaw_command_lf_;
    bool                cruise_mode_;

    const INS&          ins_;
    const quat_t&       qe_;                          ///< Attitude quaternion structure
    pid_controller_t    pid_controller_;

    /* parameters */
    float max_climb_rate_;
    float max_rel_yaw_;
    float min_cruise_dist_;                          // minimal distance to target above which the vehicle changes to cruise mode
    pid_controller_conf_t cruise_pid_config_;
    pid_controller_conf_t hover_pid_config_;
};

Position_controller::conf_t Position_controller::default_config()
{
    conf_t conf;
    conf.max_climb_rate     = 1.0f;
    conf.max_rel_yaw        = 0.7f;
    conf.min_cruise_dist    = 2;            // TODO: Choose meaning full min_cruise_dist

    /* config of the cruise pid controller */
    conf.cruise_pid_config.p_gain                   = 0.7f;
    conf.cruise_pid_config.clip_min                 = 0.0f;
    conf.cruise_pid_config.clip_max                 = 3.0f;
    conf.cruise_pid_config.integrator.gain          = 0.0f;
    conf.cruise_pid_config.integrator.clip_pre      = 0.0f;
    conf.cruise_pid_config.integrator.clip          = 0.0f;
    conf.cruise_pid_config.differentiator.gain      = 0.14f;
    conf.cruise_pid_config.differentiator.clip      = 0.46f;
    conf.cruise_pid_config.soft_zone_width          = 0.0f;

    /* config of the hover pid controller */
    conf.hover_pid_config.p_gain                  = 0.4f;
    conf.hover_pid_config.clip_min                = 0.0f;
    conf.hover_pid_config.clip_max                = 3.0f;
    conf.hover_pid_config.integrator.gain         = 0.0f;
    conf.hover_pid_config.integrator.clip_pre     = 0.0f;
    conf.hover_pid_config.integrator.clip         = 0.0f;
    conf.hover_pid_config.differentiator.gain     = 0.28f;
    conf.hover_pid_config.differentiator.clip     = 0.46f;
    conf.hover_pid_config.soft_zone_width         = 0.0f;

    return conf;
};

#endif /* POSITION_CONTROLLER_HPP_ */