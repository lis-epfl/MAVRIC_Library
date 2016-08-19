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
 * \file pwm_chibios.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Wrapper class for PWM using ChibiOS/HAL
 *
 ******************************************************************************/

#include "hal/chibios/pwm_chibios.hpp"


Pwm_chibios::Pwm_chibios(conf_t config):
    driver_(config.driver),
    config_(config.config),
    channel_(config.channel),
    port_(config.port),
    pin_(config.pin),
    alternate_function_(config.alternate_function)
{}


bool Pwm_chibios::init(void)
{
    pwmStart(driver_, &config_);
    pwmEnablePeriodicNotification(driver_);
    palSetPadMode(port_, pin_, alternate_function_);
    return true;
}


bool Pwm_chibios::set_pulse_width_us(uint16_t pulse_us)
{
    pwmEnableChannel(driver_, channel_, pulse_us);
    pwmEnableChannelNotification(driver_, channel_);
    return true;
}


bool Pwm_chibios::set_period_us(uint16_t period_us)
{
    pwmChangePeriodI(driver_, period_us);
    pwmEnablePeriodicNotification(driver_);
    
    return true;
}
