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
 * \file gimbal_controller.hpp
 *
 * \author MAV'RIC Team
 * \author Alexandre Cherpillod
 *
 * \brief   Gimbal control management
 *
 ******************************************************************************/


#ifndef GIMBAL_CONTROLLER_HPP_
#define GIMBAL_CONTROLLER_HPP_

#include "control/pid_controller.hpp"
#include "drivers/servo.hpp"

extern "C"
{
#include "control/control_command.h"
}

/**
 * \brief Gimbal controller configuration
 */
typedef struct
{
    attitude_command_t   		attitude_command_desired_config;	///< Initial desired attitude command
    attitude_command_t   		attitude_command_range_config[2];  	///< Allowed [min;max] range of the gimbal
    attitude_command_t   		attitude_output_config;  			///< Initial output commands
} gimbal_controller_conf_t;

/**
 * \brief Default configuration structure
 */
static inline gimbal_controller_conf_t gimbal_controller_default_config();


class Gimbal_controller
{
public:
    /**
     * \brief   Constructor
     *
     * \param   servo_pitch         Servo for pitch
     * \param   servo_yaw           Servo for yaw
     * \param   config              Configuration structure
     */
    Gimbal_controller(Servo& servo_pitch, Servo& servo_yaw, const gimbal_controller_conf_t config = gimbal_controller_default_config());


    /**
     * \brief   Main update function - sends gimbal command to two PWM outputs (for pitch and yaw)
     *
     * \return  success
     */
    bool update(void);


    attitude_command_t			attitude_command_desired_;	///< Attitude command (input from head-tracker)

private:
    /**
     * \brief   Sends the output to the servos
     */
    void gimbal_controller_mix_to_servos(void);


    enum RANGE_GIMBAL
    {
        MIN_RANGE_GIMBAL = 0,              ///< Range of the minimum allowed gimbal angles
        MAX_RANGE_GIMBAL = 1               ///< Range of the maximum allowed gimabl angles
    };

    attitude_command_t			attitude_command_range_[2];	///< Range [min; max] of the attitude commands
    attitude_command_t			attitude_output_;			///< Output to PWM (output)
    Servo&						servo_pitch_;				///< Gimbal pitch servo
    Servo&						servo_yaw_;					///< Gimbal yaw servo
};


static inline gimbal_controller_conf_t gimbal_controller_default_config()
{
    gimbal_controller_conf_t conf = {};

    float min_max_angle = 45.0f;

    //init desired attitude command
    conf.attitude_command_desired_config.rpy[0] = 0.0f;
    conf.attitude_command_desired_config.rpy[1] = 0.0f;
    conf.attitude_command_desired_config.rpy[2] = 0.0f;

    //init output commands
    conf.attitude_output_config.rpy[0] = 0.0f;
    conf.attitude_output_config.rpy[1] = 0.0f;
    conf.attitude_output_config.rpy[2] = 0.0f;

    //init MIN range of allowed attitude command
    conf.attitude_command_range_config[0].rpy[0] = -min_max_angle;
    conf.attitude_command_range_config[0].rpy[1] = -min_max_angle;
    conf.attitude_command_range_config[0].rpy[2] = -min_max_angle;

    //init MAX range of allowed attitude command
    conf.attitude_command_range_config[1].rpy[0] = min_max_angle;
    conf.attitude_command_range_config[1].rpy[1] = min_max_angle;
    conf.attitude_command_range_config[1].rpy[2] = min_max_angle;

    return conf;
};

#endif /* GIMBAL_CONTROLLER_HPP_ */
