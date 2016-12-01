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


#include "manual_control/joystick.hpp"
#include "util/coord_conventions.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"
#include "util/quick_trig.hpp"


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
}


float Joystick::throttle() const
{
    if(throttle_mode_ == throttle_mode_t::ZERO_CENTER)
    {
        return channels_.z;
    }else
    {
        return 2 * channels_.z - 1;
    }
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
    // ATTIUDE -> VELOCITY -> POSITION_HOLD -> AUTO
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
    else if((buttons & 0x0020) == 0x0020)   // check if AUTO button pressed
    {
        mav_mode_desired_.set_ctrl_mode(Mav_mode::AUTO);
    }

    buttons_.button_mask = buttons;
}
