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
 * \file navigation.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdbool.h>

#include "communication/mavlink_communication.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/state.hpp"

extern "C"
{
#include "control/stabilisation.h"
#include "util/quaternions.h"
#include "control/pid_controller.h"
}

/**
 * \brief The navigation structure
 */
class Navigation
{

public:
    enum internal_state_t
    {
        NAV_ON_GND,
        NAV_TAKEOFF,
        NAV_MANUAL_CTRL,
        NAV_NAVIGATING,
        NAV_HOLD_POSITION,
        NAV_STOP_ON_POSITION,
        NAV_STOP_THERE,
        NAV_LANDING,
        NAV_LAND_ON_TAG,
    };

    /**
     * \brief   The critical behavior enum
     */
    enum critical_behavior_enum
    {
        CLIMB_TO_SAFE_ALT,                                  ///< First critical behavior
        FLY_TO_HOME_WP,                                     ///< Second critical behavior, comes after Navigation::CLIMB_TO_SAFE_ALT
        HOME_LAND,                                          ///< Third critical behavior, comes after Navigation::FLY_TO_HOME_WP
        CRITICAL_LAND                                       ///< Fourth critical behavior
    };

    /**
     * \brief   The auto-landing enum
     */
    typedef enum
    {
        DESCENT_TO_SMALL_ALTITUDE,                          ///< First auto landing behavior
        DESCENT_TO_GND                                      ///< Second auto landing behavior, comes after DESCENT_TO_SMAL_ALTITUDE
    } auto_landing_behavior_t;

    /**
     * \brief The navigation configuration structure
     */
    struct conf_t
    {
        float dist2vel_gain;                                ///< The gain linking the distance to the goal to the actual speed
        float cruise_speed;                                 ///< The cruise speed in m/s
        float max_climb_rate;                               ///< Max climb rate in m/s

        float soft_zone_size;                               ///< Soft zone of the velocity controller

        float alt_lpf;                                      ///< The low-pass filtered altitude for auto-landing
        float LPF_gain;                                     ///< The value of the low-pass filter gain
        float kp_yaw;                                       ///< The yaw gain in velocity control mode

        pid_controller_t hovering_controller;               ///< hovering controller
        pid_controller_t wpt_nav_controller;                ///< waypoint navigation controller
    };

    /**
     * \brief   Constructor
     *
     * \param   controls_nav            The pointer to the control structure
     * \param   qe                      The pointer to the attitude quaternion structure
     * \param   position_estimation     The pointer to the position estimation structure
     * \param   state                   The pointer to the state structure
     * \param   mavlink_stream          The pointer to the MAVLink_stream structure
     * \param   nav_config              The pointer to the config structure
     */
    Navigation(control_command_t& controls_nav, const quat_t& qe, const Position_estimation& position_estimation, State& state, Mavlink_stream& mavlink_stream, conf_t nav_config = default_config());

    /**
     * \brief   Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
     *
     * \return  Success
     */
    static bool update(Navigation* navigation);

    /**
     * \brief   default configuration for navigation
     *
     * \return default config
     */
    static inline conf_t default_config();

    float dist2vel_gain;                                ///< The gain linking the distance to the goal to the actual speed
    pid_controller_t hovering_controller;               ///< hovering controller
    pid_controller_t wpt_nav_controller;                ///< waypoint navigation controller
    float cruise_speed;                                 ///< The cruise speed in m/s
    float max_climb_rate;                               ///< Max climb rate in m/s
    float soft_zone_size;                               ///< Soft zone of the velocity controller
    local_position_t goal;                              ///< The local position of the navigation function goal (depends on the mode), to be used in another module if needed (e.g. collision avoidance)

    float alt_lpf;                                      ///< The low-pass filtered altitude for auto-landing
    float LPF_gain;                                     ///< The value of the low-pass filter gain

    float dist2wp_sqr;                                  ///< The square of the distance to the waypoint


    internal_state_t internal_state_;                   ///< The internal state of the navigation module
    critical_behavior_enum critical_behavior;           ///< The critical behavior enum
    auto_landing_behavior_t auto_landing_behavior;      ///< The autolanding behavior enum

    float kp_yaw;                                       ///< The yaw gain in velocity control mode

    const quat_t& qe;                                   ///< The pointer to the attitude quaternion structure

private:
    float dt;                                           ///< The time interval between two navigation updates
    uint32_t last_update;                               ///< The time of the last navigation update in ms
    uint32_t loop_count;                                ///< A counter for sending MAVLink messages at a lower rate than the function
    control_command_t& controls_nav;                    ///< The pointer to the navigation control structure
    const Position_estimation& position_estimation;     ///< The pointer to the position estimation structure in central_data
    State& state;                                       ///< The pointer to the state structure in central_data
    const Mavlink_stream& mavlink_stream;               ///< The pointer to the MAVLink stream structure

    /**
     * \brief                   Sets the Robot speed to reach waypoint
     *
     * \param   rel_pos         Relative position of the waypoint
     */
    void set_speed_command(float rel_pos[]);


    /**
     * \brief                       Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
     *
     */
    void run();
};

Navigation::conf_t Navigation::default_config()
{
    conf_t conf                                      = {};

    conf.dist2vel_gain                               = 0.7f;
    conf.cruise_speed                                = 3.0f;
    conf.max_climb_rate                              = 1.0f;
    conf.soft_zone_size                              = 0.0f;
    conf.alt_lpf                                     = 0.0f;
    conf.LPF_gain                                    = 0.9f;
    conf.kp_yaw                                      = 0.2f;
    conf.wpt_nav_controller                          = {};
    conf.wpt_nav_controller.p_gain                   = 0.7f;
    conf.wpt_nav_controller.clip_min                 = 0.0f;
    conf.wpt_nav_controller.clip_max                 = 3.0f;
    conf.wpt_nav_controller.integrator               = {};
    conf.wpt_nav_controller.integrator.gain          = 0.0f;
    conf.wpt_nav_controller.integrator.clip_pre      = 0.0f;
    conf.wpt_nav_controller.integrator.accumulator   = 0.0f;
    conf.wpt_nav_controller.integrator.clip          = 0.0f;
    conf.wpt_nav_controller.differentiator           = {};
    conf.wpt_nav_controller.differentiator.gain      = 0.14f;
    conf.wpt_nav_controller.differentiator.previous  = 0.0f;
    conf.wpt_nav_controller.differentiator.clip      = 0.46f;
    conf.wpt_nav_controller.output                   = 0.0f;
    conf.wpt_nav_controller.error                    = 0.0f;
    conf.wpt_nav_controller.last_update_s            = 0.0f;
    conf.wpt_nav_controller.dt_s                     = 1;
    conf.wpt_nav_controller.soft_zone_width          = 0.0f;
    conf.hovering_controller                         = {};
    conf.hovering_controller.p_gain                  = 0.4f;
    conf.hovering_controller.clip_min                = 0.0f;
    conf.hovering_controller.clip_max                = 3.0f;
    conf.hovering_controller.integrator              = {};
    conf.hovering_controller.integrator.gain         = 0.0f;
    conf.hovering_controller.integrator.clip_pre     = 0.0f;
    conf.hovering_controller.integrator.accumulator  = 0.0f;
    conf.hovering_controller.integrator.clip         = 0.0f;
    conf.hovering_controller.differentiator          = {};
    conf.hovering_controller.differentiator.gain     = 0.28f;
    conf.hovering_controller.differentiator.previous = 0.0f;
    conf.hovering_controller.differentiator.clip     = 0.46f;
    conf.hovering_controller.output                  = 0.0f;
    conf.hovering_controller.error                   = 0.0f;
    conf.hovering_controller.last_update_s           = 0.0f;
    conf.hovering_controller.dt_s                    = 1;
    conf.hovering_controller.soft_zone_width         = 0.0f;

    return conf;
};

#endif // NAVIGATION_H_
