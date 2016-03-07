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
 * \file servos_mix_quadcopter_diag.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 * \author Nicolas Dousse
 *
 * \brief Links between torque commands and servos PWM command for quadcopters
 * in diagonal configuration
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_QUADCOPTER_DIAG_HPP_
#define SERVOS_MIX_QUADCOPTER_DIAG_HPP_

#include "drivers/servo.hpp"

extern "C"
{
#include "control/control_command.h"
#include "util/constants.h"
}

/**
 * \brief The servo mix structure for a quad in cross shape
 */
typedef struct
{
    rot_dir_t   motor_front_right_dir;      ///< Front motor turning direction
    rot_dir_t   motor_front_left_dir;       ///< Left  motor turning direction
    rot_dir_t   motor_rear_right_dir;       ///< Right motor turning direction
    rot_dir_t   motor_rear_left_dir;        ///< Rear  motor turning direction
    float       min_thrust;                 ///< Minimum thrust
    float       max_thrust;                 ///< Maximum thrust
} servos_mix_quadcopter_diag_conf_t;


/**
 * \brief   servos mix structure
 */
typedef struct
{
    rot_dir_t   motor_front_right_dir;          ///< Front motor turning direction
    rot_dir_t   motor_front_left_dir;           ///< Left  motor turning direction
    rot_dir_t   motor_rear_right_dir;           ///< Right motor turning direction
    rot_dir_t   motor_rear_left_dir;            ///< Rear  motor turning direction
    float       min_thrust;                     ///< Minimum thrust
    float       max_thrust;                     ///< Maximum thrust
    const torque_command_t* torque_command;     ///< Pointer to the torque command structure
    const thrust_command_t* thrust_command;     ///< Pointer to the thrust command structure
    Servo*  motor_front_right;              ///< Pointer to servos for front right motor
    Servo*  motor_front_left;               ///< Pointer to servos for front right motor
    Servo*  motor_rear_right;               ///< Pointer to servos for front right motor
    Servo*  motor_rear_left;                ///< Pointer to servos for front right motor
} servos_mix_quadcotper_diag_t;


/**
 * \brief                   Initialize the servo mix
 *
 * \param mix               The pointer to the servo mix structure of the quad in cross shape
 * \param config            The pointer to the configuration of servo mix structure
 * \param torque_command    The pointer to the torque command structure
 * \param thrust_command    The pointer to the thrust command structure
 * \param motor_rear_left   The pointer to the servos structure
 * \param motor_front_left  The pointer to the servos structure
 * \param motor_front_right The pointer to the servos structure
 * \param motor_rear_right  The pointer to the servos structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool servos_mix_quadcotper_diag_init(servos_mix_quadcotper_diag_t* mix,
                                     const servos_mix_quadcopter_diag_conf_t config,
                                     const torque_command_t* torque_command,
                                     const thrust_command_t* thrust_command,
                                     Servo* motor_rear_left,
                                     Servo* motor_front_left,
                                     Servo* motor_front_right,
                                     Servo* motor_rear_right);


/**
 * \brief                   Update from servos mix
 *
 * \param mix               The pointer to the servos mix structure
 */
void servos_mix_quadcopter_diag_update(servos_mix_quadcotper_diag_t* mix);


#endif
