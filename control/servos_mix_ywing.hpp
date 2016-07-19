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
 * \file servos_mix_ywing.h
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Links between torque commands and servos PWM command for Ywing
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_YWING_HPP_
#define SERVOS_MIX_YWING_HPP_

#include "drivers/servo.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "control/control_command.h"
}

/**
 * \brief The servo mix structure for a Ywing
 */
typedef struct
{
    flap_dir_t  flap_top_dir;       ///< Left  flap turning direction
    flap_dir_t  flap_right_dir;     ///< Right flap turning direction
    flap_dir_t  flap_left_dir;      ///< Rear  flap turning direction
    float       min_thrust;         ///< Minimum thrust
    float       max_thrust;         ///< Maximum thrust
    float       min_deflection;     ///< Minimum deflection for flaps
    float       max_deflection;     ///< Maximum deflection for flaps
} servo_mix_ywing_conf_t;


/**
 * \brief   servos mix structure
 */
typedef struct
{
    flap_dir_t  flap_top_dir;       ///< Left  flap turning direction
    flap_dir_t  flap_right_dir;     ///< Right flap turning direction
    flap_dir_t  flap_left_dir;      ///< Rear  flap turning direction
    float       min_thrust;         ///< Minimum thrust
    float       max_thrust;         ///< Maximum thrust
    float       min_deflection;     ///< Minimum deflection for flaps
    float       max_deflection;     ///< Maximum deflection for flaps
    const torque_command_t* torque_command; ///< Pointer to the torque command structure
    const thrust_command_t* thrust_command; ///< Pointer to the thrust command structure
    Servo*                  motor;          ///< Pointer to the servos structure for main motor
    Servo*                  flap_top;       ///< Pointer to the servos structure for top flap
    Servo*                  flap_right;     ///< Pointer to the servos structure for right flap
    Servo*                  flap_left;      ///< Pointer to the servos structure for left flap
} servo_mix_ywing_t;


/**
 * \brief   Initialize the servo mix
 *
 * \param   mix             Pointer to the servo mix structure
 * \param   config          Pointer to the configuration
 * \param   torque_command  Pointer to the torque command structure
 * \param   thrust_command  Pointer to the thrust command structure
 * \param   servo_motor     Pointer to the servos structure
 * \param   servo_flap_top  Pointer to the servos structure
 * \param   servo_flap_right Pointer to the servos structure
 * \param   servo_flap_left Pointer to the servos structure
 *
 * \return  True if the init succeed, false otherwise
 */
bool servo_mix_ywing_init(servo_mix_ywing_t* mix,
                          const servo_mix_ywing_conf_t* config,
                          const torque_command_t* torque_command,
                          const thrust_command_t* thrust_command,
                          Servo* servo_motor,
                          Servo* servo_flap_top,
                          Servo* servo_flap_right,
                          Servo* servo_motor_left);


/**
 * \brief           Update des servos mix
 *
 * \param mix       Pointer to the servos mix structure
 */
void servos_mix_ywing_update(servo_mix_ywing_t* mix);


#endif
