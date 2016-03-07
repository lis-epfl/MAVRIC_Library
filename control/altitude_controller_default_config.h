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
 * \file altitude_controller_default_config.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Default configuration for the module altitude_controller
 *
 ******************************************************************************/


#ifndef ALTITUDE_CONTROLLER_DEFAULT_CONFIG_H_
#define ALTITUDE_CONTROLLER_DEFAULT_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "control/altitude_controller.h"

static inline altitude_controller_conf_t altitude_controller_default_config()
{
    altitude_controller_conf_t conf = {};

    conf.hover_point                        = -0.28f;
    conf.pid_config                         = {};
    conf.pid_config.p_gain                  = 0.2f;
    conf.pid_config.clip_min                = -1.0f;
    conf.pid_config.clip_max                = 1.0f;
    conf.pid_config.integrator              = {};
    conf.pid_config.integrator.pregain      = 0.5f;
    conf.pid_config.integrator.postgain     = 1.0f;
    conf.pid_config.integrator.accumulator  = 0.0f;
    conf.pid_config.integrator.clip         = 0.5f;
    conf.pid_config.differentiator          = {};
    conf.pid_config.differentiator.gain     = 0.4f;
    conf.pid_config.differentiator.previous = 0.0f;
    conf.pid_config.differentiator.clip     = 0.65f;
    conf.pid_config.soft_zone_width         = 0.0f;

    return conf;
};

#ifdef __cplusplus
}
#endif

#endif // ALTITUDE_CONTROLLER_DEFAULT_CONFIG_H_
