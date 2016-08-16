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
 * \file navigation.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief Waypoint navigation controller
 *
 ******************************************************************************/


#ifndef NAVIGATION_HPP_
#define NAVIGATION_HPP_

#include <cstdbool>

#include "communication/mavlink_communication.hpp"
#include "sensing/position_estimation.hpp"
#include "communication/state.hpp"
#include "mission/mission_handler_registry.hpp"
#include "control/waypoint.hpp"
#include "control/dubin.hpp"
#include "control/stabilisation.hpp"
#include "control/pid_controller.hpp"

extern "C"
{
#include "util/quaternions.h"
}


/**
 * \brief The navigation structure
 */
class Navigation
{

public:
    enum class strategy_t
    {
        DIRECT_TO = 0,
        DUBIN = 1,
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
        float one_over_scaling;                             ///< Line vector field parameter
        float vertical_vel_gain;                            ///< Gain for the vertical velocity calculation

        float soft_zone_size;                               ///< Soft zone of the velocity controller

        float alt_lpf;                                      ///< The low-pass filtered altitude for auto-landing
        float LPF_gain;                                     ///< The value of the low-pass filter gain
        float kp_yaw;                                       ///< The yaw gain in velocity control mode

        float safe_altitude;                                ///< The altitude at which the robot will fly in critical mode
        float critical_landing_altitude;                    ///< The altitude at which the drone will try to land in the critical state
        float minimal_radius;                               ///< The minimal circle radius
        float heading_acceptance;                           ///< The heading acceptance to switch to next waypoint

        pid_controller_t hovering_controller;               ///< hovering controller
        pid_controller_t wpt_nav_controller;                ///< waypoint navigation controller

        float takeoff_altitude;                             ///< Local altitude at which the take-off procedure should stop, for a fixed-wing.

        strategy_t navigation_strategy;                  ///< The type of navigation strategy
    };

    /**
     * \brief   Constructor
     *
     * \param   controls_nav                Reference to the control structure
     * \param   qe                          Reference to the attitude quaternion structure
     * \param   ins                         Reference to the Inertial navigation system
     * \param   state                       Reference to the state structure
     * \param   mavlink_stream              Reference to the MAVLink_stream structure
     * \param   Mission_handler_registry    Reference to the mission handler registry
     * \param   nav_config                  Reference to the config structure
     */
    Navigation(control_command_t& controls_nav, const quat_t& qe, const INS& ins, State& state, Mavlink_stream& mavlink_stream, Mission_handler_registry& mission_handler_registry, conf_t nav_config = default_config());

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

    /**
     * \brief   default configuration for navigation of a wing robot
     *
     * \return default config
     */
    static inline conf_t default_wing_config();

    /**
     * \brief   Sets the goal to be equal to the waypoint inputted
     *
     * Will set the dubin state to be DUBIN_INIT if there is a significant
     * (change in waypoint locaiton) change in the goal.
     *
     * \param   new_goal            The new waypoint goal
     */
    void set_goal(Waypoint new_goal);

    /**
     * \brief   Sets the start_wpt_time_ to the current time
     */
    void set_start_wpt_time();

    /**
     * \brief   Gets the time that the drone started to move towards the waypoint
     *
     * \return  start_wpt_time
     */
    uint32_t start_wpt_time() const;

    /**
     * \brief   Gets a flag stating if the drone is currently waiting at a waypoint
     *
     * \return  Is drone waiting at a waypoint
     */
    bool waiting_at_waypoint() const;

    /**
     * \brief   Sets the flag stating if the drone is currently waiting at a waypoint
     */
    void set_waiting_at_waypoint(bool waiting_at_waypoint);



    dubin_state_t dubin_state;                          ///< The internal Dubin state
    float dist2vel_gain;                                ///< The gain linking the distance to the goal to the actual speed
    pid_controller_t hovering_controller;               ///< hovering controller
    pid_controller_t wpt_nav_controller;                ///< waypoint navigation controller
    float cruise_speed;                                 ///< The cruise speed in m/s
    float max_climb_rate;                               ///< Max climb rate in m/s
    float soft_zone_size;                               ///< Soft zone of the velocity controller
    float one_over_scaling;                             ///< Line vector field parameter
    float vertical_vel_gain;                            ///< Gain for the vertical velocity calculation
    float safe_altitude;                                ///< The altitude at which the robot will fly in critical mode
    float critical_landing_altitude;                    ///< The altitude at which the drone will try to land in the critical state
    float minimal_radius;                               ///< The minimal circle radius
    float heading_acceptance;                           ///< The heading acceptance to switch to next waypoint
    float takeoff_altitude;                             ///< Local altitude at which the take-off procedure should stop, for a fixed-wing

    strategy_t navigation_strategy;                     ///< The type of navigation strategy

    float alt_lpf;                                      ///< The low-pass filtered altitude for auto-landing
    float LPF_gain;                                     ///< The value of the low-pass filter gain

    float dist2wp_sqr;                                  ///< The square of the distance to the waypoint

    critical_behavior_enum critical_behavior;           ///< The critical behavior enum
    auto_landing_behavior_t auto_landing_behavior;      ///< The autolanding behavior enum
    float kp_yaw;                                       ///< The yaw gain in velocity control mode
    const quat_t& qe;                                   ///< The pointer to the attitude quaternion structure

private:
    bool waiting_at_waypoint_;                          ///< Flag stating if the drone is currently at a waypoint waiting to advance
    uint32_t start_wpt_time_;                           ///< The time at which the MAV starts to travel towards its waypoint

    Waypoint goal_;                                     ///< The local position of the navigation function goal
    dubin_t goal_dubin_;                                ///< The dubin structure for the goal

    float dt;                                           ///< The time interval between two navigation updates
    uint32_t last_update;                               ///< The time of the last navigation update in ms
    uint32_t loop_count;                                ///< A counter for sending MAVLink messages at a lower rate than the function
    control_command_t& controls_nav;                    ///< Reference to the navigation control structure
    const INS& ins;                                     ///< Reference to the inertial navigation system
    State& state;                                       ///< Reference to the state
    const Mavlink_stream& mavlink_stream;               ///< Reference to the MAVLink stream
    Mission_handler_registry& mission_handler_registry_;///< The reference to the mission handler registry
    /**
     * \brief                   Sets the Robot speed to reach waypoint
     *
     * \param   rel_pos         Relative position of the waypoint
     */
    void set_speed_command(float rel_pos[]);

    /**
    * \brief                       Computes the Dubin path
    *
    * \param   navigation          Pointer to navigation
    */
    void set_dubin_velocity(dubin_t* dubin);

    /**
     * \brief   Computes the state machine for the Dubin navigation type
     *
     * \param   waypoint_next           The next waypoint structure
     */
    void dubin_state_machine();

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

    conf.one_over_scaling                            = 0.3f;
    conf.safe_altitude                               = -30.0f;
    conf.critical_landing_altitude                   = 5.0f;
    conf.minimal_radius                              = 5.0f;
    conf.heading_acceptance                          = PI/6.0f;
    conf.vertical_vel_gain                           = 1.0f;
    conf.takeoff_altitude                            = -10.0f;
    //conf.navigation_strategy                         = Navigation::strategy_t::DIRECT_TO;
    conf.navigation_strategy                         = Navigation::strategy_t::DUBIN;
    return conf;
};

Navigation::conf_t Navigation::default_wing_config()
{
    conf_t conf = default_config();

    conf.cruise_speed                                = 12.0f;
    conf.max_climb_rate                              = 6.0f;

    conf.safe_altitude                               = -60.0f;
    conf.minimal_radius                              = 45.0f;
    // conf.heading_acceptance                          = PI/6.0f;  //TODO should this be adapted for the wing
    conf.takeoff_altitude                            = -60.0f;
    conf.navigation_strategy                         = Navigation::strategy_t::DUBIN;
    return conf;
};

#endif // NAVIGATION_HPP_
