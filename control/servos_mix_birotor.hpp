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
 * \file servos_mix_quadcopter_diag.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in diagonal configuration
 *
 ******************************************************************************/

#ifndef SERVOS_MIX_BIROTOR_HPP_
#define SERVOS_MIX_BIROTOR_HPP_

#include "drivers/servo.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "control/control_command.h"
#include "drivers/unsupported/daler_dc_motor_ctrl.h"
}

/**
 * \brief Configuration structure
 */
typedef struct
{
    rot_dir_t   motor_left_dir;     ///< Left motor rotation direction
    rot_dir_t   motor_right_dir;    ///< Right motor rotation direction
    rot_dir_t   servo_left_dir;     ///< Left servo rotation direction
    rot_dir_t   servo_right_dir;    ///< Right servo rotation direction
    float       min_thrust;         ///< Minimum thrust command
    float       max_thrust;         ///< Maximum thrust command
    float       min_servo;          ///< Minimum servo command
    float       max_servo;          ///< Maximum servo command

} servo_mix_birotor_conf_t;


/**
 * \brief   servos mix structure
 */
typedef struct
{
    rot_dir_t   motor_left_dir;                 ///< Left motor rotation direction
    rot_dir_t   motor_right_dir;                ///< Right motor rotation direction
    rot_dir_t   servo_left_dir;                 ///< Left servo rotation direction
    rot_dir_t   servo_right_dir;                ///< Right servo rotation direction
    float       min_thrust;                     ///< Minimum thrust command
    float       max_thrust;                     ///< Maximum thrust command
    float       min_servo;                      ///< Minimum servo command
    float       max_servo;                      ///< Maximum servo command
    const torque_command_t* torque_command;     ///< Pointer to torque command (input)
    const thrust_command_t* thrust_command;     ///< Pointer to thrust command (input)
    Servo* motor_left;                          ///< Pointer to servos (output)
    Servo* motor_right;                         ///< Pointer to servos (output)
    Servo* servo_left;                          ///< Pointer to servos (output)
    Servo* servo_right;                         ///< Pointer to servos (output)
    daler_dc_motor_ctrl_t*  dc_motors;          ///< Pointer to DC motor controller (output)
} servo_mix_birotor_t;


/**
 * \brief [brief description]
 * @details [long description]
 *
 * \param servo_mix [description]
 * \param config [description]
 * \param torque_command [description]
 * \param servo_pwm [description]
 *
 * \return success
 */
bool servo_mix_birotor_init(servo_mix_birotor_t* mix, const servo_mix_birotor_conf_t* config, const torque_command_t* torque_command, const thrust_command_t* thrust_command, servos_t* servos, daler_dc_motor_ctrl_t*  dc_motor);


/**
 * \brief [brief description]
 * @details [long description]
 *
 * \param servo_mix [description]
 *
 * \return success
 */
bool servos_mix_birotor_update(servo_mix_birotor_t* mix);

#endif // SERVOS_MIX_BIROTOR_HPP_
