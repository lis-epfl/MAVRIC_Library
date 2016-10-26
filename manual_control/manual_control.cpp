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
 * \file manual_control.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of taking the correct input for the control
 * (i.e. the remote or the joystick)
 *
 ******************************************************************************/


#include "manual_control/manual_control.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Manual_control::Manual_control(Satellite* sat, conf_t config, remote_conf_t remote_config) :
    joystick(config.joystick_config),
    mode_source_(config.mode_source),
    control_source_(config.control_source)
{
    remote_init(&remote, sat, remote_config);
}

float Manual_control::throttle() const
{
    float throttle = 0.0f;

    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            throttle = remote_get_throttle(&remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            throttle = joystick.throttle();
            break;
        case CONTROL_SOURCE_NONE:
            throttle = -1.0f;
            break;
    }
    return throttle;
}


float Manual_control::roll() const
{
    float roll = 0.0f;

    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            roll = remote_get_roll(&remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            roll = joystick.roll();
            break;
        case CONTROL_SOURCE_NONE:
            roll = 0.0f;
            break;
    }
    return roll;
}


float Manual_control::pitch() const
{
    float pitch = 0.0f;

    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            pitch = remote_get_pitch(&remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            pitch = joystick.pitch();
            break;
        case CONTROL_SOURCE_NONE:
            pitch = 0.0f;
            break;
    }
    return pitch;
}

float Manual_control::yaw() const
{
    float yaw = 0.0f;

    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            yaw = remote_get_yaw(&remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            yaw = joystick.yaw();
            break;
        case CONTROL_SOURCE_NONE:
            yaw = -1.0f;
            break;
    }
    return yaw;
}

void Manual_control::get_torque_command(torque_command_t* command,
                                        float scale_roll,
                                        float scale_pitch,
                                        float scale_yaw) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        command->xyz[ROLL]  = scale_roll  * roll();
        command->xyz[PITCH] = scale_pitch * pitch();
        command->xyz[YAW]   = scale_yaw   * yaw();
    }
}


void Manual_control::get_rate_command(rate_command_t* command,
                                      float scale_roll,
                                      float scale_pitch,
                                      float scale_yaw) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        command->xyz[ROLL]  = scale_roll  * roll();
        command->xyz[PITCH] = scale_pitch * pitch();
        command->xyz[YAW]   = scale_yaw   * yaw();
    }
}


void Manual_control::get_thrust_command_copter(thrust_command_t* command, float scale) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        command->xyz[X] = 0.0f;
        command->xyz[Y] = 0.0f;
        command->xyz[Z] = - scale * throttle();
    }
}


void Manual_control::get_thrust_command_wing(thrust_command_t* command, float scale) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        command->xyz[X] = scale * throttle();
        command->xyz[Y] = 0.0f;
        command->xyz[Z] = 0.0f;
    }
}


void Manual_control::get_attitude_command_absolute_yaw( attitude_command_t* command,
                                                        float scale_roll,
                                                        float scale_pitch,
                                                        float scale_yaw) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        *command = coord_conventions_quaternion_from_rpy(scale_roll  * roll(),
                                                         scale_pitch * pitch(),
                                                         scale_yaw   * yaw());
    }
}


void Manual_control::get_attitude_command(  attitude_command_t* command,
                                            quat_t current_attitude,
                                            float scale_roll,
                                            float scale_pitch,
                                            float scale_yaw) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        float rpy[3];
        rpy[ROLL]  = scale_roll * roll();
        rpy[PITCH] = scale_pitch * pitch();
        rpy[YAW]   = scale_yaw * yaw() + coord_conventions_get_yaw(current_attitude);

        *command = coord_conventions_quaternion_from_rpy(rpy);
    }
}


void Manual_control::get_attitude_command_vtol( attitude_command_t* command,
                                                quat_t current_attitude,
                                                float reference_pitch,
                                                float scale_roll,
                                                float scale_pitch,
                                                float scale_yaw) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        // Apply yaw and pitch first
        quat_t q_yawpitch = coord_conventions_quaternion_from_rpy(0.0f,
                                                                  scale_pitch * pitch() + reference_pitch,
                                                                  scale_yaw * yaw() + coord_conventions_get_yaw(current_attitude));

        // Apply roll according to transition factor
        quat_t q_roll = coord_conventions_quaternion_from_rpy(scale_roll * roll(),
                                                              0.0f,
                                                              0.0f);

        // q := q . q_rh . q_rf
        *command = quaternions_multiply(q_yawpitch, q_roll);
    }
}


void Manual_control::get_velocity_command(  velocity_command_t* command,
                                            float scale_x,
                                            float scale_y,
                                            float scale_z) const
{
    if (control_source_ != CONTROL_SOURCE_NONE)
    {
        command->xyz[X] = scale_x * pitch();
        command->xyz[Y] = scale_y * roll();
        command->xyz[Z] = scale_z * throttle();
    }
}


Mav_mode Manual_control::get_mode_from_source(Mav_mode mode_current)
{
    Mav_mode new_mode = mode_current;

    switch (mode_source_)
    {
        case MODE_SOURCE_GND_STATION:
            break;
        case MODE_SOURCE_REMOTE:
            if (remote_check(&remote) != SIGNAL_LOST)
            {
                // Update mode from remote
                remote_mode_update(&remote);
                new_mode = remote_mode_get(&remote, mode_current);
                //joystick.mav_mode_desired = mode_current;
            }
            break;
        case MODE_SOURCE_JOYSTICK:
            new_mode = joystick.get_mode(mode_current);
            break;
    }

    return new_mode;
}


void Manual_control::set_mode_of_source(Mav_mode mode)
{
    // override internal mav_mode of joystick
    joystick.mav_mode_desired_ = mode;
}

signal_quality_t Manual_control::get_signal_strength()
{
    signal_quality_t rc_check;

    // Get remote signal strength
    if (control_source_ == CONTROL_SOURCE_REMOTE)
    {
        rc_check = remote_check(&remote);
    }
    else
    {
        rc_check = SIGNAL_GOOD;
    }

    return rc_check;
}
