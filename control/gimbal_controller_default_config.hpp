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
 * \author Alexandre Cherpillod
 *
 * \brief Default configuration for the module gimbal_controller
 *
 ******************************************************************************/


#ifndef GIMBAL_CONTROLLER_DEFAULT_CONFIG_H_
#define GIMBAL_CONTROLLER_DEFAULT_CONFIG_H_

#include "control/gimbal_controller.hpp"

static inline gimbal_controller_conf_t gimbal_controller_default_config()
{
    gimbal_controller_conf_t conf = {};

    //init desired attitude command
    conf.attitude_command_desired_config.rpy[0]   = 0.0f;
    conf.attitude_command_desired_config.rpy[1]   = 0.0f;
    conf.attitude_command_desired_config.rpy[2]   = 0.0f;

    //init MIN range of allowed attitude command
	conf.attitude_command_range_config[0].rpy[0] = -45.0f;
	conf.attitude_command_range_config[0].rpy[1] = -45.0f;
	conf.attitude_command_range_config[0].rpy[2] = -45.0f;

	//init MAX range of allowed attitude command
	conf.attitude_command_range_config[1].rpy[0] = 45.0f;
	conf.attitude_command_range_config[1].rpy[1] = 45.0f;
	conf.attitude_command_range_config[1].rpy[2] = 45.0f;

	//init output commands
	conf.attitude_output_config.rpy[0]   = 0.0f;
	conf.attitude_output_config.rpy[1]   = 0.0f;
	conf.attitude_output_config.rpy[2]   = 0.0f;

    return conf;
};


#endif // GIMBAL_CONTROLLER_DEFAULT_CONFIG_H_
