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

    joystick->mav_mode_desired = Mav_mode(0);
    joystick->arm_action = ARM_ACTION_NONE;

    //Alex --
    joystick->commTrigger = 0;
    joystick->updateFenceCenter = false;
    joystick->isFenceEnabled = false;
    //--

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

Mav_mode joystick_get_mode(joystick_t* joystick, const Mav_mode current_mode)
{
    // new mode equals desired_mode, execept for HIL which is taken from current_mode
    Mav_mode new_mode = joystick->mav_mode_desired;
    new_mode.set_hil_flag(current_mode.is_hil());

    // set armed flag depending on arm_action
    if (joystick->arm_action == ARM_ACTION_ARMING)
    {
        new_mode.set_armed_flag(true);
        joystick->arm_action = ARM_ACTION_NONE;
    }
    else if (joystick->arm_action == ARM_ACTION_DISARMING)
    {
        new_mode.set_armed_flag(false);
        joystick->arm_action = ARM_ACTION_NONE;
    }
    else
    {
        new_mode.set_armed_flag(current_mode.is_armed());
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
    //controls->rpy[YAW] 	= joystick->channels.y;
    controls->tvel[Y] 	= joystick->channels.y;
    controls->tvel[Z] 	= joystick->channels.z;
    //controls->rpy[YAW] = joystick->channels.r;
}
void joystick_get_rate_command_wing(joystick_t* joystick, control_command_t* controls)
{
    /*  We want to obtain same results as with full manual control.
        So, we want the output of the regulator to go from -1 to +1 on each axis
        (if scaling is applied on manual mode by the joystick, it will also be applied on the rate, so the remote scaling doesn't matter)
        Assuming the regulators are only P, if the current rate is 0, we have at the output of the regulator: u = Kp*r = Kp * scaler * joystickInput
        ==> we want u = remoteInput to have the same behavior
        ==> scaler = 1/Kp
    */

    controls->rpy[ROLL] = 15.4f * joystick_get_roll(joystick);
    controls->rpy[PITCH] = 18.2f * joystick_get_pitch(joystick);
    controls->rpy[YAW] = joystick_get_yaw(joystick);
    controls->thrust = joystick_get_throttle(joystick);
}

void joystick_get_angle_command_wing(joystick_t* joystick, control_command_t* controls)
{
    controls->rpy[ROLL] = asinf(joystick_get_roll(joystick));
    controls->rpy[PITCH] = asinf(joystick_get_pitch(joystick));
    controls->rpy[YAW] = asinf(joystick_get_yaw(joystick));
    controls->thrust = joystick_get_throttle(joystick);
}

void joystick_get_velocity_wing(const joystick_t* joystick, const float ki_yaw, control_command_t* controls)
{
    controls->tvel[X] = 10.0f * (1.0f + joystick->channels.z * MAX_JOYSTICK_RANGE);
    controls->tvel[Y] = 0.0f;
    controls->tvel[Z] = -6.0f * joystick->channels.x * MAX_JOYSTICK_RANGE;
    controls->rpy[YAW] += ki_yaw * 0.2f * joystick->channels.y * MAX_JOYSTICK_RANGE; // Turn rate
}void joystick_get_control_command(const joystick_t* joystick, control_command_t* controls)
{
    controls->rpy[ROLL]     = joystick->channels.y * MAX_JOYSTICK_RANGE;
    controls->rpy[PITCH]    = joystick->channels.x * MAX_JOYSTICK_RANGE;
    controls->rpy[YAW]      = joystick->channels.r * MAX_JOYSTICK_RANGE;
    controls->thrust        = joystick->channels.z;
}

void joystick_button_update(joystick_t* joystick, uint16_t buttons)
{
    // check if ARMING button pressed and not pressed at last update
    if((buttons & 0x0001) == 0x0001 && (joystick->buttons.button_mask & 0x0001) != 0x0001)
    {
        if (joystick->mav_mode_desired.is_armed())
        {
            joystick->mav_mode_desired.set_armed_flag(false);
            joystick->arm_action = ARM_ACTION_DISARMING;
        }
        else
        {
            joystick->mav_mode_desired.set_armed_flag(true);
            joystick->arm_action = ARM_ACTION_ARMING;
        }
    }

    // set ctrl mode according to buttons (precedence for ctrl mode if several buttons are pressed: 
    // ATTIUDE -> VELOCITY -> POSITION_HOLD -> GPS_NAV
    if((buttons & 0x0004) == 0x0004)        // check if ATTITUDE button pressed
    {
        joystick->mav_mode_desired.set_ctrl_mode(Mav_mode::ATTITUDE);
    }
    else if((buttons & 0x0010) == 0x0010)   // check if VELOCITY button pressed
    {
        joystick->mav_mode_desired.set_ctrl_mode(Mav_mode::VELOCITY);
    }
    else if((buttons & 0x0002) == 0x0002)   // check if POSITION_HOLD button pressed
    {
        joystick->mav_mode_desired.set_ctrl_mode(Mav_mode::POSITION_HOLD);
    }
    else if((buttons & 0x0020) == 0x0020)   // check if GPS_NAV button pressed
    {
        joystick->mav_mode_desired.set_ctrl_mode(Mav_mode::GPS_NAV);
    }

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
    command->mode = VELOCITY_COMMAND_MODE_SEMI_LOCAL;
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
