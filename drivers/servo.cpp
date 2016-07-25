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
 * \file servo.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for servomotors using PWM
 *
 ******************************************************************************/


#include "drivers/servo.hpp"
#include "hal/common/time_keeper.hpp"



Servo::Servo(Pwm& pwm, const servo_conf_t config):
    pwm_(pwm),
    config_(config),
    value_(config.failsafe)
{
    pwm_.set_period_us(1000000.0f / config_.repeat_freq);
    failsafe(true);
}


float Servo::read(void) const
{
    return value_;
}


bool Servo::write(float value, bool to_hardware)
{
    bool success = false;

    float trimmed_value = value + config_.trim;

    if (trimmed_value < config_.min)
    {
        value_ = config_.min;
    }
    else if (trimmed_value > config_.max)
    {
        value_ = config_.max;
    }
    else
    {
        value_ = trimmed_value;
        success      = true;
    }

    if (to_hardware == true)
    {
        success &= write_to_hardware();
    }

    return success;
}


bool Servo::failsafe(bool to_hardware)
{
    bool success = true;

    value_ = config_.failsafe;

    if (to_hardware == true)
    {
        success &= write_to_hardware();
    }

    return success;
}


bool Servo::write_to_hardware(void)
{
    bool success = true;

    // Compute pulse length
    uint16_t pulse_us = (config_.pulse_magnitude_us * value_) + config_.pulse_center_us;

    // Write to pwm line
    success &= pwm_.set_pulse_width_us(pulse_us);

    return success;
}


void Servo::calibrate_esc(void)
{
    write(config_.max, true);
    time_keeper_delay_ms(2000);
    failsafe(true);
}


void Servo::set_servo_max(void)
{
    write(config_.max, true);
}


void Servo::set_servo_min(void)
{
    write(config_.min, true);
}


float Servo::servo_max(void)
{
    return config_.max;
}


float Servo::servo_min(void)
{
    return config_.min;
}
