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
 * \file manual_control.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of taking the correct input for the control
 * (i.e. the remote or the joystick)
 *
 ******************************************************************************/


#include "control/manual_control.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/constants.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

manual_control_t::manual_control_t(Satellite* sat, manual_control_conf_t config, remote_conf_t remote_config)
{
    mode_source     = config.mode_source;
    control_source  = config.control_source;

    remote_init(&remote, sat, remote_config);
    joystick_init(&joystick);
}


void manual_control_get_control_command(manual_control_t* manual_control, control_command_t* controls)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_control_command(&manual_control->remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_control_command(&manual_control->joystick, controls);
            break;
        case CONTROL_SOURCE_NONE:
            controls->rpy[ROLL] = 0.0f;
            controls->rpy[PITCH] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            controls->thrust = -1.0f;
            break;
    }
}


void manual_control_get_velocity_vector(manual_control_t* manual_control, control_command_t* controls)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_velocity_vector(&manual_control->remote, controls);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_velocity_vector(&manual_control->joystick, controls);
            break;
        case CONTROL_SOURCE_NONE:
            controls->tvel[X] = 0.0f;
            controls->tvel[Y] = 0.0f;
            controls->tvel[Z] = 0.0f;
            controls->rpy[YAW] = 0.0f;
            break;
    }
}


float manual_control_get_thrust(const manual_control_t* manual_control)
{
    float thrust = 0.0f;

    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            thrust = remote_get_throttle(&manual_control->remote);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            thrust = joystick_get_throttle(&manual_control->joystick);
            break;
        case CONTROL_SOURCE_NONE:
            thrust = -1.0f;
            break;
    }
    return thrust;
}


void manual_control_get_torque_command(const manual_control_t* manual_control, torque_command_t* command, float scale)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_torque_command(&manual_control->remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_torque_command(&manual_control->joystick, command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_rate_command(const manual_control_t* manual_control, rate_command_t* command, float scale)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_rate_command(&manual_control->remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_rate_command(&manual_control->joystick, command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_thrust_command(const manual_control_t* manual_control, thrust_command_t* command)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_thrust_command(&manual_control->remote, command);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_thrust_command(&manual_control->joystick, command);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_attitude_command_absolute_yaw(const manual_control_t* manual_control, attitude_command_t* command, float scale)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_attitude_command_absolute_yaw(&manual_control->remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_attitude_command_absolute_yaw(&manual_control->joystick, command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_attitude_command(const manual_control_t* manual_control, const float k_yaw, attitude_command_t* command, float scale)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_attitude_command(&manual_control->remote, k_yaw, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_attitude_command(&manual_control->joystick, k_yaw, command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_attitude_command_vtol(const manual_control_t* manual_control, const float k_yaw, attitude_command_t* command, float scale, float reference_pitch)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_attitude_command_vtol(&manual_control->remote, k_yaw, command, scale, reference_pitch);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_attitude_command_vtol(&manual_control->joystick, k_yaw, command, scale, reference_pitch);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


void manual_control_get_velocity_command(const manual_control_t* manual_control, velocity_command_t* command, float scale)
{
    switch (manual_control->control_source)
    {
        case CONTROL_SOURCE_REMOTE:
            remote_get_velocity_command(&manual_control->remote, command, scale);
            break;
        case CONTROL_SOURCE_JOYSTICK:
            joystick_get_velocity_command(&manual_control->joystick, command, scale);
            break;
        case CONTROL_SOURCE_NONE:
            break;
    }
}


mav_mode_t manual_control_get_mode_from_source(manual_control_t* manual_control, mav_mode_t mode_current)
{
    mav_mode_t new_mode = mode_current;

    switch (manual_control->mode_source)
    {
        case MODE_SOURCE_GND_STATION:
            new_mode = mode_current;
            manual_control->joystick.mav_mode_desired = mode_current;
            break;
        case MODE_SOURCE_REMOTE:
            if (remote_check(&manual_control->remote) != SIGNAL_LOST)
            {
                // Update mode from remote
                remote_mode_update(&manual_control->remote);
                new_mode = remote_mode_get(&manual_control->remote, mode_current);
                manual_control->joystick.mav_mode_desired = mode_current;
            }
            break;
        case MODE_SOURCE_JOYSTICK:
            new_mode = joystick_get_mode(&manual_control->joystick, mode_current);
            break;
    }

    return new_mode;
}


signal_quality_t manual_control_get_signal_strength(manual_control_t* manual_control)
{
    signal_quality_t rc_check;

    // Get remote signal strength
    if (manual_control->control_source == CONTROL_SOURCE_REMOTE)
    {
        rc_check = remote_check(&manual_control->remote);
    }
    else
    {
        rc_check = SIGNAL_GOOD;
    }

    return rc_check;
}
