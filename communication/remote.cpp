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
 * \file remote.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file is the driver for the remote control
 *
 ******************************************************************************/


#include "communication/remote.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "util/constants.h"
#include "util/coord_conventions.h"
#include "util/quick_trig.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief   Returns the value of the ARMED flag
 *
 * \param   remote          The pointer to the remote structure
 *
 * \return  The value of the ARMED flag
 */
static bool get_armed_flag(remote_t* remote);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static bool get_armed_flag(remote_t* remote)
{
    remote_mode_t* remote_mode = &remote->mode;
    bool armed;
    armed = remote_mode->current_desired_mode.is_armed();

    // Get armed flag
    if (remote_get_throttle(remote) < -0.95f &&
            remote_get_yaw(remote) < -0.9f &&
            remote_get_pitch(remote) > 0.9f &&
            remote_get_roll(remote) > 0.9f)
    {
        // Left stick bottom left corner, right stick bottom right corner => arm
        armed = true;
        remote_mode->arm_action = ARM_ACTION_ARMING;
    }
    else if (remote_get_throttle(remote) < -0.95f &&
             remote_get_yaw(remote) > 0.9f &&
             remote_get_pitch(remote) > 0.9f &&
             remote_get_roll(remote) < -0.9f)
    {
        // Left stick bottom right corner, right stick bottom left corner => disarm
        armed = false;
        remote_mode->arm_action = ARM_ACTION_DISARMING;
    }
    else
    {
        // Keep current flag
    }

    return armed;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


bool remote_init(remote_t* remote, Satellite* sat, const remote_conf_t config)
{
    bool init_success = true;

    // Init dependencies
    remote->sat = sat;

    // Init mode from remote
    remote_mode_init(&remote->mode, config.mode_config);

    // Init parameters according to remote type
    remote->type = config.type;
    switch (remote->type)
    {
        case REMOTE_TURNIGY:
            remote->scale = 1.0f / 880.0f;
            remote->deadzone = 30;

            remote->channel_inv[CHANNEL_THROTTLE] = NORMAL;
            remote->channel_inv[CHANNEL_ROLL]     = NORMAL;
            remote->channel_inv[CHANNEL_PITCH]    = INVERTED;
            remote->channel_inv[CHANNEL_YAW]      = NORMAL;
            remote->channel_inv[CHANNEL_GEAR]     = INVERTED;
            remote->channel_inv[CHANNEL_FLAPS]    = INVERTED;
            remote->channel_inv[CHANNEL_AUX1]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX2]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX3]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX4]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX5]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX6]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX7]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX8]     = NORMAL;

            init_success &= true;
            break;

        case REMOTE_SPEKTRUM:
            remote->scale = 1.0f / 700.0f;
            remote->deadzone = 30;

            remote->channel_inv[CHANNEL_THROTTLE] = NORMAL;
            remote->channel_inv[CHANNEL_ROLL]     = INVERTED;
            remote->channel_inv[CHANNEL_PITCH]    = NORMAL;
            remote->channel_inv[CHANNEL_YAW]      = NORMAL;
            remote->channel_inv[CHANNEL_GEAR]     = NORMAL;
            remote->channel_inv[CHANNEL_FLAPS]    = NORMAL;
            remote->channel_inv[CHANNEL_AUX1]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX2]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX3]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX4]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX5]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX6]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX7]     = NORMAL;
            remote->channel_inv[CHANNEL_AUX8]     = NORMAL;

            init_success &= true;
            break;
        default:
            init_success &= false;
            break;
    }

    // Init
    remote->signal_quality = SIGNAL_GOOD;
    for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; i++)
    {
        remote->channels[i] = 0.0f;
        remote->trims[i] = 0.0f;
    }
    remote->last_satellite_update = time_keeper_get_us();

    return init_success;
}


void remote_update(remote_t* remote)
{
    uint32_t now = time_keeper_get_us() ;
    float raw;

    if (remote->sat->last_update() > remote->last_satellite_update)
    {
        // Keep last update time
        remote->last_satellite_update = remote->sat->last_update();

        // Check signal quality
        if (remote->sat->dt() < 100000)
        {
            // ok
            remote->signal_quality = SIGNAL_GOOD;
        }
        else if (remote->sat->dt() < 1500000)
        {
            // warning
            remote->signal_quality = SIGNAL_BAD;
        }

        // Retrieve and scale channels
        for (uint8_t i = 0; i < REMOTE_CHANNEL_COUNT; ++i)
        {
            raw = remote->sat->channel(i);
            if (raw < remote->deadzone && raw > -remote->deadzone)
            {
                remote->channels[i] = 0.0f;
            }
            else
            {
                remote->channels[i] = raw * remote->scale * remote->channel_inv[i] - remote->trims[i];
            }
        }
    } //end of if ( remote->sat->get_new_data_available() == true )
    else
    {
        // Check for signal loss
        if ((now - remote->sat->last_update()) > 1500000)
        {
            // CRITICAL: Set all channels to failsafe
            remote->channels[CHANNEL_THROTTLE] = -1.0f;
            for (uint8_t i = 1; i < REMOTE_CHANNEL_COUNT; i++)
            {
                remote->channels[i] = 0.0f;
            }

            remote->signal_quality = SIGNAL_LOST;
        }
    }
}


signal_quality_t remote_check(remote_t* remote)
{
    remote_update(remote);

    return remote->signal_quality;
}


void remote_calibrate(remote_t* remote, remote_channel_t channel)
{
    remote->trims[channel] = remote->channels[channel] - remote->trims[channel];
}


float remote_get_channel(const remote_t* remote, remote_channel_t ch)
{
    return remote->channels[ch];
}


float remote_get_throttle(const remote_t* remote)
{
    return remote->channels[CHANNEL_THROTTLE];
}


float remote_get_roll(const remote_t* remote)
{
    return remote->channels[CHANNEL_ROLL];
}


float remote_get_pitch(const remote_t* remote)
{
    return remote->channels[CHANNEL_PITCH];
}


float remote_get_yaw(const remote_t* remote)
{
    return remote->channels[CHANNEL_YAW];
}


void remote_mode_init(remote_mode_t* remote_mode, const remote_mode_conf_t config)
{
    // Init parameters
    remote_mode->safety_channel         = config.safety_channel;
    remote_mode->safety_mode            = config.safety_mode;
    remote_mode->mode_switch_channel    = config.mode_switch_channel;
    remote_mode->mode_switch_up         = config.mode_switch_up;
    remote_mode->mode_switch_middle     = config.mode_switch_middle;
    remote_mode->mode_switch_down       = config.mode_switch_down;
    remote_mode->use_custom_switch      = config.use_custom_switch;
    remote_mode->custom_switch_channel  = config.custom_switch_channel;
    remote_mode->use_test_switch        = config.use_test_switch;
    remote_mode->test_switch_channel    = config.test_switch_channel;
    remote_mode->use_disable_remote_mode_switch = config.use_disable_remote_mode_switch;
    remote_mode->disable_remote_mode_channel        = config.disable_remote_mode_channel;

    // Init state to safety state, disarmed
    remote_mode->current_desired_mode       = remote_mode->safety_mode;
    remote_mode->arm_action                 = ARM_ACTION_NONE;
    remote_mode->current_desired_mode.set_armed_flag(false);
}


void remote_mode_update(remote_t* remote)
{
    remote_mode_t* remote_mode = &remote->mode;
    bool do_update = false;

    remote_update(remote);

    // Check whether modes should be updated
    if (remote_mode->use_disable_remote_mode_switch == true)
    {
        if (remote->channels[remote_mode->disable_remote_mode_channel] >= 0.5f)
        {
            do_update = true;
        }
        else
        {
            do_update = false;
        }
    }
    else
    {
        do_update = true;
    }

    // Do update if required
    if (do_update == true)
    {
        // Fallback to safety
        Mav_mode new_desired_mode = remote_mode->safety_mode;

        // Get armed flag from stick combinaison
        bool flag_armed = get_armed_flag(remote);

        if (remote->channels[remote_mode->safety_channel] > 0)
        {
            // Safety switch UP => Safety mode ON
            new_desired_mode = remote_mode->safety_mode;

            // Allow arm and disarm in safety mode
            new_desired_mode.set_armed_flag(flag_armed);

        }
        else
        {
            // Normal mode

            // Get base mode
            if (remote->channels[remote_mode->mode_switch_channel] >= 0.5f)
            {
                // Mode switch UP
                new_desired_mode = remote_mode->mode_switch_up;
            }
            else if (remote->channels[remote_mode->mode_switch_channel] < 0.5f &&
                     remote->channels[remote_mode->mode_switch_channel] > -0.5f)
            {
                // Mode switch MIDDLE
                new_desired_mode = remote_mode->mode_switch_middle;
            }
            else if (remote->channels[remote_mode->mode_switch_channel] <= -0.5f)
            {
                // Mode switch DOWN
                new_desired_mode = remote_mode->mode_switch_down;
            }

            // Apply custom flag
            if (remote_mode->use_custom_switch == true)
            {
                new_desired_mode.set_custom_flag(remote->channels[remote_mode->custom_switch_channel] > 0.0f);
            }

            // Apply test flag
            if (remote_mode->use_test_switch == true)
            {
                new_desired_mode.set_test_flag(remote->channels[remote_mode->test_switch_channel] > 0.0f);
            }

            // Allow only disarm in normal mode
            if (!flag_armed)
            {
                new_desired_mode.set_armed_flag(false);
            }
            else
            {
                // Keep current armed flag
                new_desired_mode.set_armed_flag(remote_mode->current_desired_mode.is_armed());
            }
        }

        // Store desired mode
        remote_mode->current_desired_mode = new_desired_mode;
    } //end of if( do_update == true )
}


Mav_mode remote_mode_get(remote_t* remote, Mav_mode current_mode)
{
    Mav_mode new_mode = remote->mode.current_desired_mode;
    // copy armed and hil flag from current mode
    new_mode.set_armed_flag(current_mode.is_armed());
    new_mode.set_hil_flag(current_mode.is_hil());

    if (remote->mode.arm_action == ARM_ACTION_ARMING)
    {
        new_mode.set_armed_flag(true);

        remote->mode.arm_action = ARM_ACTION_NONE;
    }
    else if (remote->mode.arm_action == ARM_ACTION_DISARMING)
    {
        new_mode.set_armed_flag(false);
        remote->mode.arm_action = ARM_ACTION_NONE;
    }

    return new_mode;
}

void remote_get_control_command(remote_t* remote, control_command_t* controls)
{
    remote_update(remote);

    controls->rpy[ROLL]     = remote_get_roll(remote);
    controls->rpy[PITCH]    = remote_get_pitch(remote);
    controls->rpy[YAW]      = remote_get_yaw(remote);
    controls->thrust        = remote_get_throttle(remote);
}

void remote_get_velocity_vector(remote_t* remote, control_command_t* controls)
{
    remote_update(remote);

    controls->tvel[X]   = - 10.0f * remote_get_pitch(remote);
    controls->tvel[Y]   = 10.0f * remote_get_roll(remote);
    controls->tvel[Z]   = - 1.5f * remote_get_throttle(remote);
    controls->rpy[YAW]  = remote_get_yaw(remote);

    /*print_util_dbg_print("vel cmd\r\n");
    print_util_dbg_print("rVx ");
    print_util_dbg_putfloat(controls->tvel[X],3);
    print_util_dbg_print("  rVy ");
	print_util_dbg_putfloat(controls->tvel[Y],3);
	print_util_dbg_print("  rVz ");
	print_util_dbg_putfloat(controls->tvel[Z],3);
	print_util_dbg_print("  rYaw ");
	print_util_dbg_putfloat(controls->rpy[YAW],3);*/
}

void remote_get_rate_command_wing(remote_t* remote, control_command_t* controls)
{
    remote_update(remote);

    /*  We want to obtain same results as with full manual control.
        So, we want the output of the regulator to go from -1 to +1 on each axis
        (if scaling is applied on manual mode by the remote, it will also be applied on the rate, so the remote scaling doesn't matter)
        Assuming the regulators are only P, if the current rate is 0, we have at the output of the regulator: u = Kp*r = Kp * scaler * remoteInput
        ==> we want u = remoteInput to have the same behavior
        ==> scaler = 1/Kp
    */

    controls->rpy[ROLL] = 15.4f * remote_get_roll(remote);
    controls->rpy[PITCH] = 18.2f * remote_get_pitch(remote);
    controls->rpy[YAW] = remote_get_yaw(remote);
    controls->thrust = remote_get_throttle(remote);
}

void remote_get_angle_command_wing(remote_t* remote, control_command_t* controls)
{
    remote_update(remote);

    controls->rpy[ROLL] = asinf(remote_get_roll(remote));
    controls->rpy[PITCH] = asinf(remote_get_pitch(remote));
    controls->rpy[YAW] = asinf(remote_get_yaw(remote));
    controls->thrust = remote_get_throttle(remote);
}

void remote_get_velocity_wing(remote_t* remote, const float ki_yaw, control_command_t* controls)
{
    remote_update(remote);

    controls->tvel[X] = 10.0f * (1.0f + remote_get_throttle(remote));
    controls->tvel[Y] = 0.0f;
    controls->tvel[Z] = -6.0f * remote_get_pitch(remote);
    controls->rpy[YAW] += ki_yaw * 0.2f * remote_get_roll(remote); // Turn rate
}

void remote_get_torque_command(const remote_t* remote, torque_command_t* command, float scale)
{
    command->xyz[ROLL]  = scale * remote_get_roll(remote);
    command->xyz[PITCH] = scale * remote_get_pitch(remote);
    command->xyz[YAW]   = scale * remote_get_yaw(remote);
}


void remote_get_rate_command(const remote_t* remote, rate_command_t* command, float scale)
{
    command->xyz[ROLL]  = scale * remote_get_roll(remote);
    command->xyz[PITCH] = scale * remote_get_pitch(remote);
    command->xyz[YAW]   = scale * remote_get_yaw(remote);
}


void remote_get_thrust_command(const remote_t* remote, thrust_command_t* command)
{
    command->thrust = remote_get_throttle(remote);
}


void remote_get_attitude_command_absolute_yaw(const remote_t* remote, attitude_command_t* command, float scale)
{
    command->rpy[ROLL]  = scale * remote_get_roll(remote);
    command->rpy[PITCH] = scale * remote_get_pitch(remote);
    command->rpy[YAW]   = scale * remote_get_yaw(remote);

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void remote_get_attitude_command(const remote_t* remote, const float ki_yaw, attitude_command_t* command, float scale)
{
    command->rpy[ROLL]  = scale * remote_get_roll(remote);
    command->rpy[PITCH] = scale * remote_get_pitch(remote);
    command->rpy[YAW]   += ki_yaw * scale * remote_get_yaw(remote);

    aero_attitude_t attitude;
    attitude.rpy[ROLL]  = command->rpy[ROLL];
    attitude.rpy[PITCH] = command->rpy[PITCH];
    attitude.rpy[YAW]   = command->rpy[YAW];
    command->quat = coord_conventions_quaternion_from_aero(attitude);
}


void remote_get_attitude_command_vtol(const remote_t* remote, const float ki_yaw, attitude_command_t* command, float scale, float reference_pitch)
{
    // Get Roll Pitch and Yaw from remote
    command->rpy[ROLL]  = scale * remote_get_roll(remote);
    command->rpy[PITCH] = scale * remote_get_pitch(remote) + reference_pitch;
    command->rpy[YAW]   += ki_yaw * scale * remote_get_yaw(remote);

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


void remote_get_velocity_command(const remote_t* remote, velocity_command_t* command, float scale)
{
    command->xyz[X] = - 10.0f * scale * remote_get_pitch(remote);
    command->xyz[Y] =   10.0f * scale * remote_get_roll(remote);
    command->xyz[Z] = - 1.5f  * scale * remote_get_throttle(remote);
	command->mode = VELOCITY_COMMAND_MODE_SEMI_LOCAL;
}
