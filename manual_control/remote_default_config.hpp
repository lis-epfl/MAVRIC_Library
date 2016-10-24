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
 * \file remote_default_config.hpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *
 * \brief Default configuration for the remote module
 *
 ******************************************************************************/


#ifndef REMOTE_DEFAULT_CONFIG_HPP_
#define REMOTE_DEFAULT_CONFIG_HPP_

#include "manual_control/remote.hpp"

static inline remote_conf_t remote_default_config()
{
    remote_conf_t conf                                  = {};

    conf.type                                           = REMOTE_TURNIGY;
    conf.mode_config                                    = {};
    conf.mode_config.safety_channel                     = CHANNEL_GEAR;
    conf.mode_config.safety_mode                        = Mav_mode::ATTITUDE;
    // conf.mode_config.safety_mode.flags               = {};
    // conf.mode_config.safety_mode.flags.MANUAL        = MANUAL_ON;
    conf.mode_config.mode_switch_channel                = CHANNEL_FLAPS;
    conf.mode_config.mode_switch_up                     = Mav_mode::VELOCITY;
    // conf.mode_config.mode_switch_up.flags            = {};
    // conf.mode_config.mode_switch_up.flags.MANUAL     = MANUAL_ON;
    // conf.mode_config.mode_switch_up.flags.STABILISE  = STABILISE_ON;
    conf.mode_config.mode_switch_middle                 = Mav_mode::POSITION_HOLD;
    // conf.mode_config.mode_switch_middle.flags        = {};
    // conf.mode_config.mode_switch_middle.flags.MANUAL = MANUAL_ON;
    // conf.mode_config.mode_switch_middle.flags.GUIDED = GUIDED_ON;
    conf.mode_config.mode_switch_down                   = Mav_mode::GPS_NAV;
    // conf.mode_config.mode_switch_down.flags          = {};
    // conf.mode_config.mode_switch_down.flags.AUTO     = AUTO_ON;
    conf.mode_config.use_custom_switch                  = true; //false,
    conf.mode_config.custom_switch_channel              = CHANNEL_AUX1;
    conf.mode_config.use_test_switch                    = false;
    conf.mode_config.test_switch_channel                = CHANNEL_AUX2;
    conf.mode_config.use_disable_remote_mode_switch     = false;
    conf.mode_config.test_switch_channel                = CHANNEL_AUX2;

    return conf;
};


#endif // REMOTE_DEFAULT_CONFIG_HPP_
