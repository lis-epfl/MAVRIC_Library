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
 * \file position_controller_direct.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief Basic controller for position and yaw
 *
 ******************************************************************************/


#ifndef POSITION_CONTROLLER_DIRECT_HPP_
#define POSITION_CONTROLLER_DIRECT_HPP_

#include "control/position_controller.hpp"


class Position_controller_direct : public Position_controller
{
public:

    enum class initial_yaw_t
    {
        ZERO = 0,                   ///< set yaw mode to commanded with initial yaw command 0
        INITIAL,                    ///< set yaw mode to commanded with initial yaw command = current yaw
        AUTOMATIC                   ///< set yaw mode to automatic
    };

    struct conf_t
    {
        pid_controller_conf_t cruise_pid_config;
        pid_controller_conf_t hover_pid_config;
        float max_climb_rate;
        float max_rel_yaw;
        float min_cruise_dist;
        initial_yaw_t initial_yaw;
    };

    /**
     * \brief   Constructor
     *
     * \param   vel_command     structure to write the velocity command to
     * \param   ins             inertial navigation unit to use for position estimation
     * \param   qe              attitude quaternion for attitude estimation
     * \param   config          configuration of position controller
     */
    Position_controller_direct(control_command_t& vel_command, const INS& ins, const quat_t& qe, const conf_t& config = default_config());

    /**
     * \brief   Update velocity command
     *
     * \details Calls Position_controller.update_internal() to update internal variables
     */
    void update();

    /**
     * \brief   set yaw and position command (yaw mode is set to commanded)
     *
     * \details see set_position_command and set_yaw_command
     *
     * \param   pos_command_lf     absolute position in local frame
     * \param   yaw_command_lf     absolute yaw in local frame
     */
    void set_command(local_position_t pos_command_lf, float yaw_command_lf);

    /**
     * \brief   set yaw command (changes yaw mode to commanded)
     *
     * \param   yaw_command_lf      absolute yaw in local frame
     */
    void set_yaw_command(float yaw_command_lf);

    /**
     * \brief   set yaw mode to automatic (vehicle points towards goal)
     *
     * \details previous yaw_command is ignored
     */
    void set_yaw_automatic();

    /**
     * \brief   returns whether yaw mode is automatic
     *
     * \details automatic: drone points towards target; commanded: using yaw_command_lf_
     *
     * \return  returns true if yaw_mode is automatic
     */
    inline bool is_yaw_automatic(){return yaw_automatic_;};

    /**
     * \brief   default configuration for navigation
     *
     * \return default config
     */
    static inline conf_t default_config();


    /********************************************
     *         public Get/Set methods           *
     *******************************************/
    pid_controller_conf_t& cruise_pid_config(); // TODO: set return to const when onboard parameter use get/set
    pid_controller_conf_t& hover_pid_config(); // TODO: set return to const when onboard parameter use get/set


private:

    /**
     * \brief   set to cruise mode/hover mode (switches between PID parameters)
     *
     * \param   cruise_mode     if true, switches to cruise_mode, otherwise, switches to hovermode
     */
    void set_cruise_mode(bool cruise_mode);

    float               yaw_command_lf_;            //< input yaw command (ignore if yaw_mode is automatic)
    bool                cruise_mode_;               //< indicating if in cruise mode (far from waypoints) or hover mode (close to waypoints)
    bool                yaw_automatic_;             //< indicating if automatic (vehicle pointing towards goal) or commanded (using yaw_command_lf_)
    pid_controller_t    pid_controller_;            //< position pid controller

    /* parameters */
    pid_controller_conf_t cruise_pid_config_;        //< configuration of cruise pid position controller
    pid_controller_conf_t hover_pid_config_;         //< configuration of hover pid position controller
    float max_climb_rate_;                           //< maximal climb rate (velocity is scaled so that output velocity in Z <= max_climb_rate)
    float max_rel_yaw_;                              //< maximal output yaw command
    float min_cruise_dist_;                          //< minimal distance to target above which the vehicle changes to cruise mode
};

Position_controller_direct::conf_t Position_controller_direct::default_config()
{
    conf_t conf;
    conf.max_climb_rate     = 1.0f;
    conf.max_rel_yaw        = 0.7f;
    conf.min_cruise_dist    = 2.0f;            // TODO: Choose meaning full min_cruise_dist
    conf.initial_yaw        = initial_yaw_t::INITIAL;

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

#endif /* POSITION_CONTROLLER_DIRECT_HPP_ */