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
 * \file barometer_bmp085.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief   Driver for the BMP085 barometer
 *
 ******************************************************************************/


#include "drivers/barometer_ms5611.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/maths.h"
#include "util/print_util.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Barometer_MS5611::Barometer_MS5611(I2c& i2c, conf_t config):
    i2c_(i2c),
    config_(config),
    state_(state_t::IDLE),
    last_state_update_us_(0.0f),
    last_update_us_(0.0f),
    dt_s_(0.1f),
    last_altitudes_{0.0f, 0.0f, 0.0f}
{
    pressure_           = 0.0f;
    temperature_        = 0.0f;
    altitude_gf_        = 0.0f;
    altitude_bias_gf_   = 0.0f;
    speed_lf_           = 0.0f;
    temperature_        = 24.0f;    // Nice day
}


bool Barometer_MS5611::init(void)
{
    bool res = true;
    uint8_t command;

    // Reset sensor
    command = COMMAND_RESET;
    // i2c_.probe((uint8_t)config_.address);

    // i2c_.write(&command, 1, (uint8_t)config_.address);
    // Wait a bit
    time_keeper_delay_ms(20);

    // Test if the sensor is here
    // res = i2c_.probe((uint8_t)config_.address);

    if (res)
    {
        // Read calibration data
        uint8_t buffer[12];
        command = COMMAND_GET_CALIBRATION;

        // i2c_.write(&command, 1, (uint8_t)config_.address);
        // i2c_.read(buffer, 12, (uint8_t)config_.address);

        calib_.SENS_T1  = (buffer[0] << 8)  + buffer[1];
        calib_.OFF_T1   = (buffer[2] << 8)  + buffer[3];
        calib_.TCS      = (buffer[4] << 8)  + buffer[5];
        calib_.TCO      = (buffer[6] << 8)  + buffer[7];
        calib_.T_REF    = (buffer[8] << 8)  + buffer[9];
        calib_.TEMPSENS = (buffer[10] << 8) + buffer[11];

        state_ = state_t::IDLE;
    }
    else
    {
        // Try again later
        state_ = state_t::INIT;
    }

    return res;
}


bool Barometer_MS5611::update(void)
{
    bool res = true;

    switch (state_)
    {
        case state_t::INIT:
            res &= init();
        break;

        case state_t::IDLE:
            res &= start_temperature_sampling();
            state_ = state_t::GET_TEMPERATURE;
        break;

        case state_t::GET_TEMPERATURE:
            res &= read_temperature();
            res &= start_pressure_sampling();
            state_ = state_t::GET_PRESSURE;
        break;

        case state_t::GET_PRESSURE:
            res &= read_pressure();
            res &= start_temperature_sampling();
            state_ = state_t::GET_TEMPERATURE;
        break;
    }

    speed_lf_ = (float)state_;  // TODO remove

    last_state_update_us_ = time_keeper_get_us();

    return res;
}


uint64_t Barometer_MS5611::last_update_us(void) const
{
    return last_update_us_;
}


float Barometer_MS5611::pressure(void)  const
{
    return pressure_;
}


float Barometer_MS5611::altitude_gf(void) const
{
    return altitude_gf_;
}


float Barometer_MS5611::vertical_speed_lf(void) const
{
    return speed_lf_;
}


float Barometer_MS5611::temperature(void) const
{
    return temperature_;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Barometer_MS5611::start_temperature_sampling(void)
{
    bool res = true;

    uint8_t command = COMMAND_START_TEMPERATURE_CONV + (uint8_t)config_.oversampling_ratio_temperature;
    // res &= i2c_.write(&command, 1, (uint8_t)config_.address);

    return res;
}

bool Barometer_MS5611::read_temperature(void)
{
    bool res = true;

    // Get sampled data
    uint8_t data[3];
    uint8_t command = COMMAND_GET_DATA;
    // res &= i2c_.write(&command, 1, (uint8_t)config_.address);
    // res &= i2c_.read(data, 3, (uint8_t)config_.address);

    // Convert raw data to temperature and apply calibration
    int32_t raw_temperature      = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 0);
	int64_t delta_temp           = raw_temperature - (calib_.T_REF << 8);
    int64_t temperature_unscaled = 2000 + ((delta_temp * calib_.TEMPSENS) >> 23);
    float temperature            = ((float)temperature_unscaled) / 100.0f;

    // second order temperature compensation
	if (temperature < 2000)
    {
		temperature_unscaled -= (delta_temp * delta_temp) >> 31;
    }

    // TODO
    temperature_ = raw_temperature; // temperature_unscaled;

    return res;
}

bool Barometer_MS5611::start_pressure_sampling(void)
{
    bool res = true;

    uint8_t command = COMMAND_START_PRESSURE_CONV + (uint8_t)config_.oversampling_ratio_pressure;
    // res &= i2c_.write(&command, 1, (uint8_t)config_.address);

    time_sampling_start_ms_ = time_keeper_get_ms();

    return res;
}

bool Barometer_MS5611::read_pressure(void)
{
    bool res = true;

    // Get sampled data
    uint8_t data[3];
    uint8_t command = COMMAND_GET_DATA;
    // res &= i2c_.write(&command, 1, (uint8_t)config_.address);
    // res &= i2c_.read(data, 3, (uint8_t)config_.address);

    // TODO
    int32_t raw_pressure      = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 0);
    altitude_gf_ = raw_pressure;

    return res;
}
