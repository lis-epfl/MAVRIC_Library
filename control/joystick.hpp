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
 * \file joystick.h
 *
 * \author MAV'RIC Team
 *
 * \brief This file is to decode the set manual command message from MAVLink
 *
 ******************************************************************************/


#ifndef JOYSTICK_H_
#define JOYSTICK_H_

#include "communication/state.hpp"

extern "C"
{
#include "control/stabilisation.h"
#include "control/control_command.h"
}


#define MAX_JOYSTICK_RANGE 0.8  ///< Scale down the joystick channel amplitude, as done in remote

/**
 * \brief button enumeration
 */
typedef enum
{
    BUTTON_UNPRESSED = 0,
    BUTTON_PRESSED = 1,
} button_pressed_t;


/**
 * \brief   The union structure for the bit mask of the joystick buttons
 */
typedef union
{
    uint16_t button_mask;
    // unamed bitfield structure, use to access directly the flags
    struct
    {
        button_pressed_t        button_16   : 1;
        button_pressed_t        button_15   : 1;
        button_pressed_t        button_14   : 1;
        button_pressed_t        button_13   : 1;
        button_pressed_t        button_12   : 1;
        button_pressed_t        button_11   : 1;
        button_pressed_t        button_10   : 1;
        button_pressed_t        button_9    : 1;
        button_pressed_t        button_8    : 1;
        button_pressed_t        button_7    : 1;
        button_pressed_t        button_6    : 1;
        button_pressed_t        button_5    : 1;
        button_pressed_t        button_4    : 1;
        button_pressed_t        button_3    : 1;
        button_pressed_t        button_2    : 1;
        button_pressed_t        button_1    : 1;
    };
    // identical bitfield, but named (useful for initialisation)
    struct
    {
        button_pressed_t        button_16   : 1;
        button_pressed_t        button_15   : 1;
        button_pressed_t        button_14   : 1;
        button_pressed_t        button_13   : 1;
        button_pressed_t        button_12   : 1;
        button_pressed_t        button_11   : 1;
        button_pressed_t        button_10   : 1;
        button_pressed_t        button_9    : 1;
        button_pressed_t        button_8    : 1;
        button_pressed_t        button_7    : 1;
        button_pressed_t        button_6    : 1;
        button_pressed_t        button_5    : 1;
        button_pressed_t        button_4    : 1;
        button_pressed_t        button_3    : 1;
        button_pressed_t        button_2    : 1;
        button_pressed_t        button_1    : 1;
    } button;
} joystick_button_t;


/**
 * \brief  Joystick Channels
 */
typedef struct
{
    float x;    // Longitudinal (pitch)
    float y;    // Lateral      (roll)
    float z;    // Vertical     (thrust)
    float r;    // Rotation     (yaw)
} joystick_channels_t;


/**
 * \brief   The structure for the joystick
 */
typedef struct
{
    joystick_button_t buttons;          ///< The bit mask of the button pressed
    joystick_channels_t channels;       ///< Channels of the joystick
    Mav_mode mav_mode_desired;          ///< The mav mode indicated by the remote
    arm_action_t arm_action;
} joystick_t;


/**
 * \brief   Initialisation of the joystick module
 *
 * \param   joystick        The pointer to the joystick structure
 *
 * \return  True if succeeded
 */
bool joystick_init(joystick_t* joystick);


/**
 * \brief   Returns the throttle value from the joystick
 *
 * \param   joystick        The pointer to the remote structure
 *
 * \return  The value of the throttle
 */
float joystick_get_throttle(const joystick_t* joystick);


/**
 * \brief   Returns the roll value from the joystick
 *
 * \param   joystick        The pointer to the remote structure
 *
 * \return  The value of the roll
 */
float joystick_get_roll(const joystick_t* joystick);


/**
 * \brief   Returns the pitch value from the joystick
 *
 * \param   joystick        The pointer to the remote structure
 *
 * \return  The value of the pitch
 */
float joystick_get_pitch(const joystick_t* joystick);


/**
 * \brief   Returns the yaw value from the joystick
 *
 * \param   joystick        The pointer to the remote structure
 *
 * \return  The value of the yaw
 */
float joystick_get_yaw(const joystick_t* joystick);


/**
 * \brief   Returns the current desired mode value from the joystick
 *
 * \param   joystick        The pointer to the remote structure
 *
 * \return  The value of the current desired mode
 */
Mav_mode joystick_get_mode(joystick_t* joystick, const Mav_mode current_mode);


/**
 * \brief   Parse joystick to velocity vector command
 *
 * \param   joystick        The pointer to the joystick structure
 * \param   controls        The pointer to the control structure
 */
void joystick_get_velocity_vector(const joystick_t* joystick, control_command_t* controls);


/**
 * \brief   Parse joystick to attitude command
 *
 * \param   joystick        The pointer to the joystick structure
 * \param   controls        The pointer to the control structure
 */
void joystick_get_control_command(const joystick_t* joystick, control_command_t* controls);


/**
 * \brief               Do operations when buttons are pressed
 *
 * \param   joystick    The pointer to the joystick structure
 * \param   buttons     The bit mask of the buttons
 */
void joystick_button_update(joystick_t* joystick, uint16_t buttons);


/**
 * \brief   Compute torque command from the joystick
 *
 * \param   joystick        Joystick structure (input)
 * \param   command         Torque command (output)
 */
void joystick_get_torque_command(const joystick_t* joystick, torque_command_t* command, float scale);


/**
 * \brief   Compute rate command from the joystick
 *
 * \param   joystick        Joystick structure (input)
 * \param   command         Rate command (output)
 */
void joystick_get_rate_command(const joystick_t* joystick, rate_command_t* command, float scale);


/**
 * \brief   Compute thrust command from the joystick
 *
 * \param   joystick        Joystick structure (input)
 * \param   command         Thrust command (output)
 */
void joystick_get_thrust_command(const joystick_t* joystick, thrust_command_t* command);


/**
 * \brief   Compute attitude command from the joystick (absolute roll and pitch, integrated yaw)
 *
 * \param   joystick        Joystick structure (input)
 * \param   ki_yaw          Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 */
void joystick_get_attitude_command(const joystick_t* joystick, const float ki_yaw, attitude_command_t* command, float scale);


/**
 * \brief   Compute velocity command from the joystick
 *
 * \param   joystick        Joystick structure (input)
 * \param   command         Velocity command (output)
 */
void joystick_get_velocity_command(const joystick_t* joystick, velocity_command_t* command, float scale);


/**
 * \brief   Compute attitude command from the joystick (absolute angles)
 *
 * \param   joystick        Joystick structure (input)
 * \param   command         Attitude command (output)
 */
void joystick_get_attitude_command_absolute_yaw(const joystick_t* joystick, attitude_command_t* command, float scale);

/**
 * \brief   Compute attitude command from the joystick (absolute roll and pitch, integrated yaw)
 *
 * \param   joystick        Joystick structure (input)
 * \param   ki_yaw          Integration factor for yaw (0.02 is ok) (input)
 * \param   command         Attitude command (output)
 * \param   scale           Scale (maximum output / max remote input)
 * \param   reference_pitch Transition factor (0: forward flight, PI/2:hover)
 */
void joystick_get_attitude_command_vtol(const joystick_t* joystick, const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch);


#endif // JOYSTICK_H_
