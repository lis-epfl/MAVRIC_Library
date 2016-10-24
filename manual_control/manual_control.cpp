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


void Manual_control::get_control_command(control_command_t* controls)
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_control_command(&remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_control_command(controls);
            break;
        case CONTROL_SOURCE_NONE:
            controls->rpy[ROLL] = 0.0f;
            controls->rpy[PITCH] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            controls->thrust = -1.0f;
            break;
    }
}


void Manual_control::get_velocity_vector(control_command_t* controls)
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_velocity_vector(&remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_velocity_vector(controls);
            break;
        case CONTROL_SOURCE_NONE:
            controls->tvel[X] = 0.0f;
            controls->tvel[Y] = 0.0f;
            controls->tvel[Z] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            break;
    }
}

void Manual_control::get_rate_command_wing(control_command_t* controls)
{
    switch(control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_rate_command_wing(&remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_rate_command_wing(controls);
        case CONTROL_SOURCE_NONE:
            controls->rpy[ROLL] = 0.0f;
            controls->rpy[PITCH] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            controls->thrust = -1.0f;
            break;
    }
}

void Manual_control::get_angle_command_wing(control_command_t* controls)
{
    switch(control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_angle_command_wing(&remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_angle_command_wing(controls);
        case CONTROL_SOURCE_NONE:
            controls->rpy[ROLL] = 0.0f;
            controls->rpy[PITCH] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            controls->thrust = -1.0f;
            break;
    }
}

void Manual_control::get_velocity_vector_wing(const float ki_yaw, control_command_t* controls)
{
    switch(control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_velocity_wing(&remote, ki_yaw, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_velocity_wing(ki_yaw, controls);
        case CONTROL_SOURCE_NONE:
            controls->tvel[X] = 0.0f;
            controls->tvel[Y] = 0.0f;
            controls->tvel[Z] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            break;
    }
}

float Manual_control::get_thrust() const
{
    float thrust = 0.0f;

    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            thrust = remote_get_throttle(&remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            thrust = joystick.throttle();
            break;
        case CONTROL_SOURCE_NONE:
            thrust = -1.0f;
            break;
    }
    return thrust;
}


void Manual_control::get_torque_command(torque_command_t* command, float scale) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_torque_command(&remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_torque_command(command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void Manual_control::get_rate_command(rate_command_t* command, float scale) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_rate_command(&remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_rate_command(command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void Manual_control::get_thrust_command(thrust_command_t* command) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_thrust_command(&remote, command);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_thrust_command(command);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void Manual_control::get_attitude_command_absolute_yaw(attitude_command_t* command, float scale) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_attitude_command_absolute_yaw(&remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_attitude_command_absolute_yaw(command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


Attitude_controller_I::att_command_t Manual_control::get_attitude_command(quat_t current_attitude) const
{
    Attitude_controller_I::att_command_t command;
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            command = remote_get_attitude_command(&remote, current_attitude);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            command = joystick.get_attitude_command(current_attitude);
            break;
        case CONTROL_SOURCE_NONE:
            command.att.s = 0.0f;
            command.att.v[0] = 0.0f;
            command.att.v[1] = 0.0f;
            command.att.v[2] = 0.0f;
            command.thrust = 0;
            break;
    }
    return command;
}


void Manual_control::get_attitude_command_vtol(const float k_yaw, attitude_command_t* command, float scale, float reference_pitch) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_attitude_command_vtol(&remote, k_yaw, command, scale, reference_pitch);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_attitude_command_vtol(k_yaw, command, scale, reference_pitch);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void Manual_control::get_velocity_command(velocity_command_t* command, float scale) const
{
    switch (control_source_)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_velocity_command(&remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick.get_velocity_command(command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
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
