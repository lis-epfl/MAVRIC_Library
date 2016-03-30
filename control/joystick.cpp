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
 * \file joystick.c
 *
 * \author MAV'RIC Team
 *
 * \brief This file is to decode the set manual command message from MAVLink
 *
 ******************************************************************************/


#include "control/joystick.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/constants.h"
#include "util/coord_conventions.h"
#include "util/quick_trig.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief               Arming/Disarming the motors when button 1 is pressed
 *
 * \param   joystick    The pointer to the joystick structure
 * \param   button_1    The button 1 value (true if pressed)
 */
static void joystick_button_1(joystick_t* joystick, bool button_1);


/**
 * \brief               Do operations when a button is pressed
 *
 * \param   joystick    The pointer to the joystick structure
 * \param   button      The value of the button pressed (true if pressed)
 * \param   mode_flag   The flag mode to be set
 */
static void joystick_button(joystick_t* joystick, bool button, mav_flag_mask_t mode_flag);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void joystick_button_1(joystick_t* joystick, bool button_1)
{
    if (button_1)
    {
        if ((joystick->buttons.button_mask & 0x0001) != 0x0001)
        {
            if (mav_modes_is_armed(joystick->mav_mode_desired))
            {
                print_util_dbg_print("Disarming from joystick\r\n");
                joystick->mav_mode_desired &= ~MAV_MODE_FLAG_SAFETY_ARMED;
                joystick->arm_action = ARM_ACTION_DISARMING;
            }
            else
            {
                print_util_dbg_print("Arming from joystick\r\n");
                if ((joystick->mav_mode_desired & 0b01011100) == MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
                {
                    joystick->mav_mode_desired |= MAV_MODE_FLAG_SAFETY_ARMED;
                    joystick->arm_action = ARM_ACTION_ARMING;
                }
            }
            joystick->buttons.button_mask |= 0x0001;
        }
    }
    else
    {
        if (!button_1)
        {
            if ((joystick->buttons.button_mask & 0x0001) == 0x0001)
            {
                joystick->buttons.button_mask &= ~0x0001;
            }
        }
    }
}


static void joystick_button(joystick_t* joystick, bool button, mav_flag_mask_t mode_flag)
{
    if (button)
    {
        joystick->mav_mode_desired &= 0b10100011;
        joystick->mav_mode_desired += mode_flag;
    }
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool joystick_init(joystick_t* joystick)
{
    bool init_success = true;

    //joystick channels init
    joystick->channels.x = 0.0f;
    joystick->channels.y = 0.0f;
    joystick->channels.z = -1.0f;
    joystick->channels.r = 0.0f;

    //joystick buttons init
    joystick->buttons.button_mask = 0;

    joystick->mav_mode_desired = MAV_MODE_SAFE;
    joystick->arm_action = ARM_ACTION_NONE;

    return init_success;
}


float joystick_get_throttle(const joystick_t* joystick)
{
    return joystick->channels.z;
}


float joystick_get_roll(const joystick_t* joystick)
{
    return joystick->channels.y;
}



float joystick_get_pitch(const joystick_t* joystick)
{
    return joystick->channels.x;
}


float joystick_get_yaw(const joystick_t* joystick)
{
    return joystick->channels.r;
}

mav_mode_t joystick_get_mode(joystick_t* joystick, const mav_mode_t current_mode)
{
    mav_mode_t new_mode = current_mode;
    new_mode = (current_mode & 0b10100000) + (joystick->mav_mode_desired & 0b01011111);

    if (joystick->arm_action == ARM_ACTION_ARMING)
    {
        new_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
        joystick->arm_action = ARM_ACTION_NONE;
        print_util_dbg_print("Arming in new fct\r\n");
    }
    else if (joystick->arm_action == ARM_ACTION_DISARMING)
    {
        new_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
        joystick->arm_action = ARM_ACTION_NONE;
        print_util_dbg_print("Disarming in new fct\r\n");
    }

    return new_mode;
}

void joystick_get_velocity_vector(const joystick_t* joystick, control_command_t* controls)
{
    controls->tvel[X] = -10.0f  * joystick->channels.x  * MAX_JOYSTICK_RANGE;
    controls->tvel[Y] =  10.0f  * joystick->channels.y  * MAX_JOYSTICK_RANGE;
    controls->tvel[Z] = -1.5f   * joystick->channels.z;

    controls->rpy[YAW] = joystick->channels.r * MAX_JOYSTICK_RANGE;
}

void joystick_get_velocity_vector_version2(const joystick_t* joystick, control_command_t* controls)
{
	//all joystick inputs are [-1;1]
    controls->tvel[X] 	= joystick->channels.x;
    controls->tvel[Y] 	= 0.0f;
    controls->tvel[Z] 	= joystick->channels.z;
    controls->rpy[ROLL] = joystick->channels.y;
    //controls->rpy[YAW] = joystick->channels.r;
}


void joystick_get_control_command(const joystick_t* joystick, control_command_t* controls)
{
    controls->rpy[ROLL]     = joystick->channels.y * MAX_JOYSTICK_RANGE;
    controls->rpy[PITCH]    = joystick->channels.x * MAX_JOYSTICK_RANGE;
    controls->rpy[YAW]      = joystick->channels.r * MAX_JOYSTICK_RANGE;
    controls->thrust        = joystick->channels.z;
}


void joystick_button_mask(joystick_t* joystick, uint16_t buttons)
{
    joystick_button_t button_local;
    button_local.button_mask = buttons;

    //print_util_dbg_print_num(button_local.button_mask, 10);

    bool but;
    but = ((button_local.button_mask & 0x0001) == 0x0001);
    joystick_button_1(joystick, but);

    but = ((button_local.button_mask & 0x0002) == 0x0002);
    joystick_button(joystick, but, (mav_flag_mask_t)(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED));  // MAV_MODE_POSITION_HOLD

    but = ((button_local.button_mask & 0x0010) == 0x0010);
    joystick_button(joystick, but, (mav_flag_mask_t)(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED + MAV_MODE_FLAG_STABILIZE_ENABLED));                                // MAV_MODE_VELOCITY_CONTROL

    but = ((button_local.button_mask & 0x0020) == 0x0020);
    joystick_button(joystick, but, (mav_flag_mask_t)(MAV_MODE_FLAG_STABILIZE_ENABLED + MAV_MODE_FLAG_GUIDED_ENABLED + MAV_MODE_FLAG_AUTO_ENABLED));             // MAV_MODE_GPS_NAVIGATION

    but = ((button_local.button_mask & 0x0004) == 0x0004);
    joystick_button(joystick, but, (mav_flag_mask_t)(MAV_MODE_FLAG_MANUAL_INPUT_ENABLED));                                                                  // MAV_MODE_ATTITUDE_CONTROL

    joystick->buttons.button_mask = buttons;
}


void joystick_get_torque_command(const joystick_t* joystick, torque_command_t* command, float scale)
{
    command->xyz[ROLL]  = scale * joystick_get_roll(joystick);
    command->xyz[PITCH] = scale * joystick_get_pitch(joystick);
    command->xyz[YAW]   = scale * joystick_get_yaw(joystick);
}


void joystick_get_rate_command(const joystick_t* joystick, rate_command_t* command, float scale)
{
    command->xyz[ROLL]  = scale * joystick_get_roll(joystick);
    command->xyz[PITCH] = scale * joystick_get_pitch(joystick);
    command->xyz[YAW]   = scale * joystick_get_yaw(joystick);
}


void joystick_get_thrust_command(const joystick_t* joystick, thrust_command_t* command)
{
    command->thrust = joystick_get_throttle(joystick);
}


void joystick_get_attitude_command(const joystick_t* joystick, const float ki_yaw, attitude_command_t* command, float scale)
{
    command->rpy[ROLL]  = scale * joystick_get_roll(joystick);
    command->rpy[PITCH] = scale * joystick_get_pitch(joystick);
    command->rpy[YAW]   += ki_yaw * scale * joystick_get_yaw(joystick);

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void joystick_get_velocity_command(const joystick_t* joystick, velocity_command_t* command, float scale)
{
    command->xyz[X] = -10.0f * scale * joystick_get_pitch(joystick);
    command->xyz[Y] =  10.0f * scale * joystick_get_roll(joystick);
    command->xyz[Z] = -1.5f  * scale * joystick_get_throttle(joystick);
}


void joystick_get_attitude_command_absolute_yaw(const joystick_t* joystick, attitude_command_t* command, float scale)
{
    command->rpy[ROLL]  = scale * joystick_get_roll(joystick);
    command->rpy[PITCH] = scale * joystick_get_pitch(joystick);
    command->rpy[YAW]   = scale * joystick_get_yaw(joystick);

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void joystick_get_attitude_command_vtol(const joystick_t* joystick, const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch)
{
    // Get Roll Pitch and Yaw from joystick
    command->rpy[ROLL]  = scale * joystick_get_roll(joystick);
    command->rpy[PITCH] = scale * joystick_get_pitch(joystick) + reference_pitch;
    command->rpy[YAW]   += ki_yaw * scale * joystick_get_yaw(joystick);

    // Apply yaw and pitch first
    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = 0.0f;
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);


    // Apply roll according to transition factor
    quat_t q_roll = { quick_trig_cos(0.5f * command->rpy[ROLL]),
        {
            quick_trig_cos(reference_pitch)* quick_trig_sin(0.5f * command->rpy[ROLL]),
            0.0f,
            quick_trig_sin(reference_pitch)* quick_trig_sin(0.5f * command->rpy[ROLL])
        }
    };

    // q := q . q_rh . q_rf
    command->quat = quaternions_multiply(command->quat, q_roll);
}
