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
 * \brief Interface for position controller
 *
 ******************************************************************************/


#ifndef POSITION_CONTROLLER_HPP_
#define POSITION_CONTROLLER_HPP_

#include "control/position_controller_i.hpp"
#include "control/xyposition_zvel_controller_i.hpp"
#include "control/navigation_controller_i.hpp"
#include "control/pid_controller.hpp"
#include "sensing/ins.hpp"

template<class TVelocity_controller>
class Position_controller : public Position_controller_I, public XYposition_Zvel_controller_I, public Navigation_controller_I, public TVelocity_controller
{
public:

    /**
     * \brief Configuration
     */
    struct conf_t
    {
        pid_controller_conf_t cruise_pid_config;
        pid_controller_conf_t hover_pid_config;
        float max_climb_rate;
        float max_rel_yaw;
        float min_cruise_dist;
		float kp_yaw;
        typename TVelocity_controller::conf_t velocity_controller_config;
    };

    enum class ctrl_mode_t
    {
        POS_XYZ,        ///< 3D position control
        POS_XY_VEL_Z    ///< Horizontal position control + vertical velocity control
    };


    struct args_t
    {
        INS& ins;
        ahrs_t& ahrs;
        typename TVelocity_controller::args_t velocity_controller_args;
    };

    Position_controller(args_t args, const conf_t& config = default_config());


	/**
     * \brief Main update function
     */
    virtual void update();

    /**
     * \brief           Sets the 3D position command
     *
     * \param command   xyz position command indicating target location in local frame
     *
     * \return success  whether command was accepted
     */
    bool set_position_command(const pos_command_t& pos_command);

    /**
     * \brief           sets the horizontal position & vertical velocity command
     *
     * \param command   xy position z velocity command indicating target location & velocity in local frame
     *
     * \return success  whether command was accepted
     */
    bool set_xyposition_zvel_command(const xypos_zvel_command_t& command);

    /**
     * \brief           sets the navigation command (desired position)
     *
     * \param command   navigation command indicating navigation target location in local frame
     *
     * \return success  whether command was accepted
     */
    bool set_navigation_command(const nav_command_t& command);

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
    void update_cascade(const pos_command_t& pos_command);

    /*
     * \brief   calc velocity command based on given position command
     * \details Sets the internal position_command_ to the provided one, without modifiying cascade_command_
     *          This function should be called from higher level controllers if they provide a command
     * \param position_command  position command
     */
    typename TVelocity_controller::vel_command_t calc_velocity_command(const pos_command_t& pos_command);


    pid_controller_conf_t& cruise_pid_config(); // TODO: set return to const when onboard parameter use get/set
    pid_controller_conf_t& hover_pid_config(); // TODO: set return to const when onboard parameter use get/set


private:

    /**
     * \brief   set to cruise mode/hover mode (switches between PID parameters)
     *
     * \param   cruise_mode     if true, switches to cruise_mode, otherwise, switches to hovermode
     */
    void set_cruise_mode(bool cruise_mode);


    const INS&          ins_;                       /// inertial navigation unit ti use for position estimation
    const ahrs_t&       ahrs_;                      ///< Attitude estimation
    pos_command_t       position_command_;          ///< Position command in local frame; Z coord ONLY VALID if in POS_XYZ
    float               zvel_command_;              ///< Velocity command in z direction (local frame); ONLY VALID if in POS_XY_VELZ mode
    pid_controller_t    pid_controller_;            //< position pid controller

    bool                cruise_mode_;               //< indicating if in cruise mode (far from waypoints) or hover mode (close to waypoints)
    ctrl_mode_t         ctrl_mode_;

    /* parameters */
    pid_controller_conf_t cruise_pid_config_;        //< configuration of cruise pid position controller
    pid_controller_conf_t hover_pid_config_;         //< configuration of hover pid position controller
    float max_climb_rate_;                           //< maximal climb rate (velocity is scaled so that output velocity in Z <= max_climb_rate)
    float min_cruise_dist_;                          //< minimal distance to target above which the vehicle changes to cruise mode
	float kp_yaw_;
};

template<class TVelocity_controller>
typename Position_controller<TVelocity_controller>::conf_t Position_controller<TVelocity_controller>::default_config()
{
    conf_t conf;
    conf.max_climb_rate     = 1.0f;
    conf.min_cruise_dist    = 2.0f;            // TODO: Choose meaning full min_cruise_dist
    conf.kp_yaw             = 0.2f;

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

    conf.velocity_controller_config = TVelocity_controller::default_config();

    return conf;
};


#include "control/position_controller.hxx"

#endif /* POSITION_CONTROLLER_HPP_ */