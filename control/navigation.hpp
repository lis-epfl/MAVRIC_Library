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

#define TAG_SEARCH_TIMEOUT_US 300000000

/**
 * \brief The navigation structure
 */
typedef struct
{
    float dist2vel_gain;                                ///< The gain linking the distance to the goal to the actual speed
    pid_controller_t hovering_controller;               ///< hovering controller
    pid_controller_t wpt_nav_controller;                ///< waypoint navigation controller
    float cruise_speed;                                 ///< The cruise speed in m/s
    float max_climb_rate;                               ///< Max climb rate in m/s
    float soft_zone_size;                               ///< Soft zone of the velocity controller

    local_position_t goal;                              ///< The local position of the navigation function goal (depends on the mode), to be used in another module if needed (e.g. collision avoidance)

    float dt;                                           ///< The time interval between two navigation updates
    uint32_t last_update;                               ///< The time of the last navigation update in ms

    float alt_lpf;                                      ///< The low-pass filtered altitude for auto-landing
    float LPF_gain;                                     ///< The value of the low-pass filter gain
    float kp_yaw;                                       ///< The yaw gain in velocity control mode

    float dist2wp_sqr;                                  ///< The square of the distance to the waypoint

    uint32_t loop_count;                                ///< A counter for sending MAVLink messages at a lower rate than the function

    navigation_internal_state_t internal_state;         ///< The internal state of the navigation module
    critical_behavior_enum critical_behavior;           ///< The critical behavior enum
    auto_landing_behavior_t auto_landing_behavior;      ///< The autolanding behavior enum
    land_on_tag_behavior_t land_on_tag_behavior;        ///< The land on tag behavior enum
    float tag_search_altitude;                          ///< The altitude that the drone should search for the tag at
    uint32_t tag_search_start_time;                     ///< The start time that the offboard camera has been searching for the tag, causes a timeout if too long

    control_command_t* controls_nav;                    ///< The pointer to the navigation control structure
    const quat_t* qe;                                   ///< The pointer to the attitude quaternion structure
    const position_estimation_t* position_estimation;   ///< The pointer to the position estimation structure in central_data
    State* state;                                       ///< The pointer to the state structure in central_data
    const mavlink_stream_t* mavlink_stream;             ///< The pointer to the MAVLink stream structure
} navigation_t;


/**
 * \brief The navigation configuration structure
 */
typedef struct
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
} navigation_config_t;

/**
 * \brief   Initialization
 *
 * \param   navigation              The pointer to the navigation structure
 * \param   nav_config              The pointer to the config structure
 * \param   controls_nav            The pointer to the control structure
 * \param   qe                      The pointer to the attitude quaternion structure
 * \param   waypoint_handler        The pointer to the waypoint handler structure
 * \param   position_estimation     The pointer to the position estimation structure
 * \param   state                   The pointer to the state structure
 * \param   mavlink_communication   The pointer to the MAVLink communication structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool navigation_init(navigation_t* navigation, navigation_config_t nav_config, control_command_t* controls_nav, const quat_t* qe, const position_estimation_t* position_estimation, State* state, mavlink_communication_t* mavlink_communication);

/**
 * \brief   Navigates the robot towards waypoint waypoint_input in 3D velocity command mode
 *
 * \param   navigation      The pointer to the navigation structure in central_data
 *
 * \return  Success
 */
bool navigation_update(navigation_t* navigation);


#endif // NAVIGATION_H_