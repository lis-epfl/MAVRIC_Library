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
 * \file remote.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This file is the driver for the remote control
 *
 ******************************************************************************/


#include "manual_control/remote.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"
#include "util/quick_trig.hpp"


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
    remote->last_satellite_update_us = time_keeper_get_us();

    return init_success;
}


bool remote_update(remote_t* remote)
{
    time_us_t now_us = time_keeper_get_us() ;
    float raw;

    if (remote->sat->last_update_us() > remote->last_satellite_update_us)
    {
        // Keep last update time
        remote->last_satellite_update_us = remote->sat->last_update_us();

        // Check signal quality
        if (remote->sat->dt_us() < 100000)
        {
            // ok
            remote->signal_quality = SIGNAL_GOOD;
        }
        else if (remote->sat->dt_us() < 1500000)
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
        if ((now_us - remote->sat->last_update_us()) > 1500000)
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

    return true;
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
