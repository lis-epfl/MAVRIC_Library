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
 * \file    gpio_chibios.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   GPIO wrapper for ChibiOS/HAL
 *
 ******************************************************************************/

#include "hal/chibios/gpio_chibios.hpp"

Gpio_chibios::Gpio_chibios(conf_t config):
    port_(config.port),
    pin_(config.pin),
    level_(false)
{}


bool Gpio_chibios::init(void)
{
    return set_low();
}


bool Gpio_chibios::configure(gpio_dir_t dir, gpio_pull_updown_t pull)
{
    // Unsupported
    (void)dir;
    (void)pull;
    return false;
}


bool Gpio_chibios::set_high(void)
{
    palSetPad(port_, pin_);
    level_ = true;
    return true;
}


bool Gpio_chibios::set_low(void)
{
    palClearPad(port_, pin_);
    level_ = false;
    return true;
}


bool Gpio_chibios::toggle(void)
{
    if (level_ == true)
    {
        set_low();
    }
    else
    {
        set_high();
    }
    return true;
}


bool Gpio_chibios::write(bool level)
{
    if (level == true)
    {
        level_ = true;
        set_high();
    }
    else
    {
        level_ = false;
        set_low();
    }

    return true;
}

bool Gpio_chibios::read(void)
{
    if (palReadPad(port_, pin_) == PAL_HIGH)
    {
        level_ = true;
    }
    else
    {
        level_ = false;
    }

    return level_;
}
