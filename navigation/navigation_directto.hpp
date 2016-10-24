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
 * \file navigation_directto.hpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief   Navigation module allowing to navigated to goal position on direct trajectory
 * \details Inherits from Position_controller, which inherits from a Velocity_controller_I
 *          i.e. can take navigation, position and velocity commands
 *
 ******************************************************************************/


#ifndef NAVIGATION_DIRECTTO_HPP_
#define NAVIGATION_DIRECTTO_HPP_

#include "navigation/navigation_controller_i.hpp"
#include "control/position_controller.hpp"

template<class TPosition_controller>
class Navigation_directto : public Navigation_controller_I, public TPosition_controller
{
    typedef typename TPosition_controller::pos_command_t pos_command_t;
public:

    /**
     * \brief Configuration
     */
    struct conf_t
    {
        float min_cruise_dist_sqr;
        pid_controller_conf_t cruise_pid_config;
        pid_controller_conf_t hover_pid_config;
        typename TPosition_controller::conf_t position_controller_config;
    };

    enum class ctrl_mode_t
    {
        POS_XYZ,        ///< 3D position control
        POS_XY_VEL_Z    ///< Horizontal position control + vertical velocity control
    };


    struct args_t
    {
        INS& ins;
        typename TPosition_controller::args_t position_controller_args;
    };

    Navigation_directto(args_t args, const conf_t& config = default_config());


	/**
     * \brief Main update function
     */
    virtual void update();

    /**
     * \brief           sets the navigation command (desired position)
     *
     * \param command   navigation command indicating navigation target location in local frame
     *
     * \return success  whether command was accepted
     */
    bool set_navigation_command(const nav_command_t& command);

    /**
     * \brief           Sets the 3D position command
     * \details         Override of position_controller func; Sets pos controller pid to std_pid_config_
     *                  and calls parent class method
     * \param command   xyz position command indicating target location in local frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_position_command(const pos_command_t& pos_command);

    /**
     * \brief           sets the horizontal position & vertical velocity command
     * \details         Override of position_controller func; Sets pos controller pid to std_pid_config_
     *                  and calls parent class method
     *
     * \param command   xy position z velocity command indicating target location & velocity in local frame
     *
     * \return success  whether command was accepted
     */
    virtual bool set_xyposition_zvel_command(const typename TPosition_controller::xypos_zvel_command_t& command);


    /**
     * \brief       returns the distance to the goal squared
     * \details     updated in calc_position_command()
     *
     * \return success  whether command was accepted
     */
    inline float distance_to_goal_sqr() const {return distance_to_goal_sqr_;};

    /**
     * \brief   Default configuration
     *
     * \return  config
     */
    static inline conf_t default_config();

protected:

    /*
     * \brief   calc velocity command based on given position command and update underlaying cascade level (TVelocity_controller)
     * \details Sets the internal position_command_ to the provided one, without modifiying cascade_command_
     *          Calls calc_velocity_command followed by TVelocity_controller::update_cascade
     *          This function should be called from higher level controllers if they provide a command
     * \param   position_command  position command to be set
     */
    void update_cascade(const nav_command_t& nav_command);

    /*
     * \brief   calc velocity command based on given position command
     * \details Sets the internal position_command_ to the provided one, without modifiying cascade_command_
     *          This function should be called from higher level controllers if they provide a command
     * \param position_command  position command
     */
     pos_command_t calc_position_command(const nav_command_t& nav_command);


    pid_controller_conf_t& cruise_pid_config(); // TODO: set return to const when onboard parameter use get/set
    pid_controller_conf_t& hover_pid_config(); // TODO: set return to const when onboard parameter use get/set


private:

    /**
     * \brief   set to cruise mode/hover mode (switches between PID parameters)
     *
     * \param   cruise_mode     if true, switches to cruise_mode, otherwise, switches to hovermode
     */
    void set_cruise_mode(bool cruise_mode);


    const INS&              ins_;                    ///< inertial navigation unit ti use for position estimation
    nav_command_t    navigation_command_;            ///< Position command in local frame; Z coord ONLY VALID if in POS_XYZ

    /* parameters */
    pid_controller_conf_t cruise_pid_config_;        ///< configuration of cruise pid position controller
    pid_controller_conf_t hover_pid_config_;         ///< configuration of hover pid position controller
    pid_controller_conf_t std_pid_config_;           ///< configuration of standard pid position controller (applied if position command is set)
    float min_cruise_dist_sqr_;                      ///< minimal distance to target above which the vehicle changes to cruise mode

    /* auxiliary variables */
    float distance_to_goal_sqr_;                     ///< Distance to goal squared(updated in calc_position_command)
};

template<class TPosition_controller>
typename Navigation_directto<TPosition_controller>::conf_t Navigation_directto<TPosition_controller>::default_config()
{
    conf_t conf;
    conf.min_cruise_dist_sqr                        = 4.0f;

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

    conf.position_controller_config = TPosition_controller::default_config();

    return conf;
};


#include "navigation/navigation_directto.hxx"

#endif /* NAVIGATION_DIRECTTO_HPP_ */
