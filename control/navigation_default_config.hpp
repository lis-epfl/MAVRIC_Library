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
 * \file navigation_default_config.hpp
 *
 * \author MAV'RIC Team
 *
 * \brief  This file configures the PID gains for the navigation speed command
 *
 ******************************************************************************/


#ifndef NAVIGATION_DEFAULT_CONFIG_H_
#define NAVIGATION_DEFAULT_CONFIG_H_

extern "C" {
#include "control/pid_controller.h"
}

static inline navigation_conf_t navigation_default_config()
{
    navigation_conf_t conf                         = {};

    conf.dist2vel_gain                               = 0.7f;
    conf.cruise_speed                                = 3.0f;
    conf.max_climb_rate                              = 1.0f;
    conf.soft_zone_size                              = 0.0f;
    conf.alt_lpf                                     = 0.0f;
    conf.LPF_gain                                    = 0.9f;
    conf.kp_yaw                                      = 0.2f;
    conf.one_over_scaling                            = 0.1f;
    conf.safe_altitude                               = -30.0f;
    conf.minimal_radius                              = 45.0f;
    conf.heading_acceptance                          = PI/6.0f;
    conf.vertical_vel_gain                           = 1.0f;
    conf.takeoff_altitude                            = -10.0f;
    conf.navigation_type                             = DIRECT_TO;
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
    conf.wpt_nav_controller.last_update              = 0.0f;
    conf.wpt_nav_controller.dt                       = 1;
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
    conf.hovering_controller.last_update             = 0.0f;
    conf.hovering_controller.dt                      = 1;
    conf.hovering_controller.soft_zone_width         = 0.0f;

    return conf;
};

#endif /* NAVIGATION_DEFAULT_CONFIG_H_ */
