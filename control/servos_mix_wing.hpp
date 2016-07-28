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
 * \file servos_mix_wing.hpp
 * 
 * \author MAV'RIC Team
 * \author Simon Pyroth
 *   
 * \brief Links between regulation output and PWM commands for a wing aircraft
 *
 ******************************************************************************/


#ifndef SERVOS_MIX_WING_H_
#define SERVOS_MIX_WING_H_

#include "drivers/servo.hpp"

extern "C"
{
#include "control/control_command.h"
#include "control/stabilisation.h"
#include "util/constants.h"
}


/**
 * \brief The servo mix structure for a wing
 */
typedef struct
{
    uint8_t             servo_right;        ///< Right aileron servo index
    uint8_t             servo_left;         ///< Left aileron servo index
    uint8_t             motor;              ///< Propulsion motor index
    
    flap_dir_t          servo_right_dir;    ///< Right aileron servo direction
    flap_dir_t          servo_left_dir;     ///< Left aileron servo direction
    
    float               min_amplitude;      ///< Minimum value which can be put on servo
    float               max_amplitude;      ///< Maximum value which can be put on servo
    float               min_thrust;         ///< Minimum value which can be put on the motor
    float               max_thrust;         ///< Maximum value which can be put on the motor
    
    float               trim_roll;          ///< Trim value for roll
    float               trim_pitch;         ///< Trim value for pitch
} servos_mix_wing_conf_t;

/**
 * \brief   Servos mix structure
 */
typedef struct 
{   
    servos_mix_wing_conf_t   config;        ///< Configuration of the mix
    const torque_command_t* torque_command; ///< The pointer to the torque command
    const thrust_command_t* thrust_command; ///< The pointer to the thrust command
    Servo* servo_left;                      ///< The pointer to the left servo
    Servo* servo_right;                     ///< The pointer to the right servo
    Servo* motor;                           ///< The pointer to the motor
} servos_mix_wing_t;


/**
 * \brief                  Initialize the servo mix
 * 
 * \param mix              Pointer to the servo mix structure of the wing
 * \param config           Pointer to the configuration of servo mix structure
 * \param torque_command   Pointer to the torque command
 * \param thrust_command   Pointer to the torque command
 * \param servo_left       Pointer to the left servo
 * \param servo_right      Pointer to the right servo
 * \param motor            Pointer to the motor
 *
 * \return  True if the init succeed, false otherwise
 */
bool servos_mix_wing_init(  servos_mix_wing_t* mix, 
                            const servos_mix_wing_conf_t config,
                            const torque_command_t* torque_command,
                            const thrust_command_t* thrust_command,
                            Servo* servo_left,
                            Servo* servo_right,
                            Servo* motor);


/**
 * \brief                  Update the servos mix
 * 
 * \param mix              Pointer to the servos mix structure
 */
void servos_mix_wing_update(servos_mix_wing_t* mix);


#endif