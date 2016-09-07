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
 * \file joystick.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief This file is to decode the set manual command message from MAVLink
 *
 ******************************************************************************/


#include "control/joystick.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"

extern "C"
{
#include "util/print_util.hpp"
#include "util/quick_trig.hpp"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Joystick::Joystick(conf_t config)
{
    //joystick channels init
    channels_.x = 0.0f;
    channels_.y = 0.0f;
    channels_.z = -1.0f;
    channels_.r = 0.0f;

    //joystick buttons init
    buttons_.button_mask = 0;

    mav_mode_desired_ = Mav_mode(0);
    arm_action_ = ARM_ACTION_NONE;

    /* apply config */
    throttle_mode_ = config.throttle_mode;
    scale_attitude_ = config.scale_attitude;
    scale_velocity_ = config.scale_velocity;
}


float Joystick::throttle() const
{
    return channels_.z;
}


float Joystick::roll() const
{
    return channels_.y;
}


float Joystick::pitch() const
{
    return channels_.x;
}


float Joystick::yaw() const
{
    return channels_.r;
}

Mav_mode Joystick::get_mode(const Mav_mode current_mode)
{
    // new mode equals desired_mode, execept for HIL which is taken from current_mode
    Mav_mode new_mode = mav_mode_desired_;
    new_mode.set_hil_flag(current_mode.is_hil());

    // set armed flag depending on arm_action
    if (arm_action_ == ARM_ACTION_ARMING)
    {
        new_mode.set_armed_flag(true);
        arm_action_ = ARM_ACTION_NONE;
    }
    else if (arm_action_ == ARM_ACTION_DISARMING)
    {
        new_mode.set_armed_flag(false);
        arm_action_ = ARM_ACTION_NONE;
    }
    else
    {
        new_mode.set_armed_flag(current_mode.is_armed());
    }
    return new_mode;
}

void Joystick::get_velocity_vector(control_command_t* controls) const
{
    controls->tvel[X] =   channels_.x  * scale_velocity_.x;
    controls->tvel[Y] =   channels_.y  * scale_velocity_.x;

    controls->rpy[YAW] =  channels_.r * scale_velocity_.r;

    if(throttle_mode_ == throttle_mode_t::ZERO_CENTER)
    {
        controls->tvel[Z] = channels_.z * scale_velocity_.z;
    }else
    {
        controls->tvel[Z] = (2 * channels_.z - 1) * scale_velocity_.z;
    }
}

void Joystick::get_rate_command_wing(control_command_t* controls) const
{
    /*  We want to obtain same results as with full manual control.
        So, we want the output of the regulator to go from -1 to +1 on each axis
        (if scaling is applied on manual mode by the joystick, it will also be applied on the rate, so the remote scaling doesn't matter)
        Assuming the regulators are only P, if the current rate is 0, we have at the output of the regulator: u = Kp*r = Kp * scaler * joystickInput
        ==> we want u = remoteInput to have the same behavior
        ==> scaler = 1/Kp
    */

    controls->rpy[ROLL] = 15.4f * roll();
    controls->rpy[PITCH] = 18.2f * pitch();
    controls->rpy[YAW] = yaw();
    controls->thrust = throttle();
}


void Joystick::get_control_command(control_command_t* controls) const
{
    controls->rpy[ROLL]     = channels_.y * scale_attitude_.y;
    controls->rpy[PITCH]    = channels_.x * scale_attitude_.x;
    controls->rpy[YAW]      = channels_.r * scale_attitude_.r;
    if(throttle_mode_ == throttle_mode_t::ZERO_CENTER)
    {
        controls->thrust        = channels_.z * scale_attitude_.z;
    }else
    {
        controls->thrust        = (2*channels_.z - 1) * scale_attitude_.z;
    }
}


void Joystick::button_update(uint16_t buttons)
{
    // check if ARMING button pressed and not pressed at last update
    if((buttons & 0x0001) == 0x0001 && (buttons_.button_mask & 0x0001) != 0x0001)
    {
        if (mav_mode_desired_.is_armed())
        {
            mav_mode_desired_.set_armed_flag(false);
            arm_action_ = ARM_ACTION_DISARMING;
        }
        else
        {
            mav_mode_desired_.set_armed_flag(true);
            arm_action_ = ARM_ACTION_ARMING;
        }
    }

    // set ctrl mode according to buttons (precedence for ctrl mode if several buttons are pressed:
    // ATTIUDE -> VELOCITY -> POSITION_HOLD -> GPS_NAV
    if((buttons & 0x0004) == 0x0004)        // check if ATTITUDE button pressed
    {
        mav_mode_desired_.set_ctrl_mode(Mav_mode::ATTITUDE);
    }
    else if((buttons & 0x0010) == 0x0010)   // check if VELOCITY button pressed
    {
        mav_mode_desired_.set_ctrl_mode(Mav_mode::VELOCITY);
    }
    else if((buttons & 0x0002) == 0x0002)   // check if POSITION_HOLD button pressed
    {
        mav_mode_desired_.set_ctrl_mode(Mav_mode::POSITION_HOLD);
    }
    else if((buttons & 0x0020) == 0x0020)   // check if GPS_NAV button pressed
    {
        mav_mode_desired_.set_ctrl_mode(Mav_mode::GPS_NAV);
    }

    buttons_.button_mask = buttons;
}


void Joystick::get_torque_command(torque_command_t* command, float scale) const
{
    command->xyz[ROLL]  = scale * roll();
    command->xyz[PITCH] = scale * pitch();
    command->xyz[YAW]   = scale * yaw();
}


void Joystick::get_rate_command(rate_command_t* command, float scale) const
{
    command->xyz[ROLL]  = scale * roll();
    command->xyz[PITCH] = scale * pitch();
    command->xyz[YAW]   = scale * yaw();
}


void Joystick::get_thrust_command(thrust_command_t* command) const
{
    command->thrust = throttle();
}


void Joystick::get_attitude_command(const float ki_yaw, attitude_command_t* command, float scale) const
{
    command->rpy[ROLL]  = scale * roll();
    command->rpy[PITCH] = scale * pitch();
    command->rpy[YAW]   += ki_yaw * scale * yaw();

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void Joystick::get_velocity_command(velocity_command_t* command, float scale) const
{
    command->xyz[X] = -10.0f * scale * pitch();
    command->xyz[Y] =  10.0f * scale * roll();
    command->xyz[Z] = -1.5f  * scale * throttle();
    command->mode = VELOCITY_COMMAND_MODE_SEMI_LOCAL;
}


void Joystick::get_angle_command_wing(control_command_t* controls) const
{
    controls->rpy[ROLL] = asinf(roll());   // arcsin()
    controls->rpy[PITCH] = asinf(pitch()); // arcsin()
    controls->rpy[YAW] = asinf(yaw());     // arcsin()
    controls->thrust = throttle();
}


void Joystick::get_velocity_wing(const float ki_yaw, control_command_t* controls) const
{
    controls->tvel[X] = 10.0f * (1.0f + channels_.z * MAX_JOYSTICK_RANGE);
    controls->tvel[Y] = 0.0f;
    controls->tvel[Z] = -6.0f * channels_.x * MAX_JOYSTICK_RANGE;
    controls->rpy[YAW] += ki_yaw * 0.2f * channels_.y * MAX_JOYSTICK_RANGE; // Turn rate
}


void Joystick::get_attitude_command_absolute_yaw(attitude_command_t* command, float scale) const
{
    command->rpy[ROLL]  = scale * roll();
    command->rpy[PITCH] = scale * pitch();
    command->rpy[YAW]   = scale * yaw();

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void Joystick::get_attitude_command_vtol(const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch) const
{
    // Get Roll Pitch and Yaw from joystick
    command->rpy[ROLL]  = scale * roll();
    command->rpy[PITCH] = scale * pitch() + reference_pitch;
    command->rpy[YAW]   += ki_yaw * scale * yaw();

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
