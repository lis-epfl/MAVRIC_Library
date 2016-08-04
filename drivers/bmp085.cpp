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
 * \file bmp085.cpp
 *
 * \author MAV'RIC Team
 * \author Felix Schill
 * \author Julien Lecoeur
 *
 * \brief   Driver for the BMP085 barometer
 *
 ******************************************************************************/


#include "drivers/bmp085.hpp"

extern "C"
{
#include "util/maths.h"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
}

enum
{
    BMP085_ULTRALOWPOWER,
    BMP085_STANDARD,
    BMP085_HIGHRES,
    BMP085_ULTRAHIGHRES
};

const float BARO_ALT_LPF                = 0.95f;        ///< low pass filter factor for altitude measured by the barometer
const float VARIO_LPF                   = 0.95f;        ///< low pass filter factor for the Vario altitude speed

const uint8_t BMP085_SLAVE_ADDRESS      = 0x77;         ///< Address of the barometer sensor on the i2c bus

const uint8_t BMP085_CAL_AC1            = 0xAA;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC2            = 0xAC;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC3            = 0xAE;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC4            = 0xB0;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC5            = 0xB2;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_AC6            = 0xB4;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_B1             = 0xB6;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_B2             = 0xB8;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MB             = 0xBA;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MC             = 0xBC;         ///< R Calibration data (16 bits)
const uint8_t BMP085_CAL_MD             = 0xBE;         ///< R Calibration data (16 bits)

const uint8_t BMP085_CONTROL            = 0xF4;         ///< Control register of the barometer sensor
const uint8_t BMP085_TEMPDATA           = 0xF6;         ///< Temperature register of the barometer sensor
const uint8_t BMP085_PRESSUREDATA       = 0xF6;         ///< Pressure Data register of the barometer sensor
const uint8_t BMP085_READTEMPCMD        = 0x2E;         ///< Read temperature Command register of the barometer sensor
const uint8_t BMP085_READPRESSURECMD    = 0x34;         ///< Read Pressure Command register of the barometer sensor

#define BMP085_OVERSAMPLING_MODE BMP085_HIGHRES         ///< Set oversampling mode of the barometer sensor to high resolution mode


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Bmp085::Bmp085(I2c& i2c):
    i2c_(i2c),
    ac1_(408),
    ac2_(-72),
    ac3_(-14383),
    ac4_(32741),
    ac5_(32757),
    ac6_(23153),
    mb_(1),     // TODO check value in datasheet
    mc_(-8711),
    md_(2868),
    b1_(6190),
    b2_(4),
    raw_pressure_{ 0, 0, 0 },
    raw_temperature_{ 0, 0 },
    last_altitudes_{0.0f, 0.0f, 0.0f},
    last_state_update_us_(0.0f),
    dt_s_(0.1f),
    state_(BMP085_IDLE)
{
    pressure_           = 0.0f;
    temperature_        = 0.0f;
    altitude_gf_        = 0.0f;
    altitude_gf_raw_    = 0.0f;
    altitude_filtered   = 0.0f;
    altitude_raw_       = 0.0f;
    altitude_bias_gf_   = 0.0f;
    speed_lf_           = 0.0f;
    speed_lf_raw_       = 0.0f;
    last_update_us_     = 0.0f;
    temperature_        = 24.0f;    // Nice day
}


bool Bmp085::init(void)
{
    bool res = true;

    // Test if the sensor if here
    res &= i2c_.probe(BMP085_SLAVE_ADDRESS);

    // Reset Bmp085 state
    state_ = BMP085_IDLE;

    return res;
}


bool Bmp085::update(void)
{
    bool res            = true;

    float altitude_raw_old;
    float altitude_filtered_old;
    float altitude_median;
    float new_speed_lf_raw;
    float new_speed_lf;

    int32_t UT, UP, B3, B5, B6, X1, X2, X3, p;
    uint32_t B4, B7;

    uint8_t start_address;

    uint8_t start_command_temp [] =
    {
        BMP085_CONTROL,
        BMP085_READTEMPCMD
    };

    uint8_t start_command_pressure [] =
    {
        BMP085_CONTROL,
        BMP085_READPRESSURECMD + (BMP085_OVERSAMPLING_MODE << 6)
    };

    switch (state_)
    {
        case BMP085_IDLE:
            res &= i2c_.write((uint8_t*) &start_command_temp, 2, BMP085_SLAVE_ADDRESS);
            state_ = BMP085_GET_TEMP;
            break;

        case BMP085_GET_TEMP:
            start_address = BMP085_TEMPDATA;
            res &= i2c_.write((uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS);
            res &= i2c_.read((uint8_t*) & (raw_temperature_), 2, BMP085_SLAVE_ADDRESS);

            res &= i2c_.write((uint8_t*) &start_command_pressure, 2, BMP085_SLAVE_ADDRESS);
            state_ = BMP085_GET_PRESSURE;
            break;

        case BMP085_GET_PRESSURE:
            start_address = BMP085_PRESSUREDATA;
            res &= i2c_.write((uint8_t*) &start_address, 1, BMP085_SLAVE_ADDRESS);
            res &= i2c_.read((uint8_t*) & (raw_pressure_), 3, BMP085_SLAVE_ADDRESS);

            UP = ((uint32_t)raw_pressure_[0] << 16 | (uint32_t)raw_pressure_[1] << 8 | (uint32_t)raw_pressure_[2]) >> (8 - BMP085_OVERSAMPLING_MODE);

            UT = raw_temperature_[0] << 8 | raw_temperature_[1];

            ///< step 1
            X1 = (UT - (int32_t)ac6_) * ((int32_t)ac5_) / pow(2, 15);
            X2 = ((int32_t)mc_ * pow(2, 11)) / (X1 + (int32_t)md_);
            B5 = X1 + X2;
            temperature_ = (B5 + 8) / pow(2, 4);
            temperature_ /= 10;

            ///< do pressure calcs
            B6 = B5 - 4000;
            X1 = ((int32_t)b2_ * ((B6 * B6) >> 12)) >> 11;
            X2 = ((int32_t)ac2_ * B6) >> 11;
            X3 = X1 + X2;
            B3 = ((((int32_t)ac1_ * 4 + X3) << BMP085_OVERSAMPLING_MODE) + 2) / 4;


            X1 = ((int32_t)ac3_ * B6) >> 13;
            X2 = ((int32_t)b1_ * ((B6 * B6) >> 12)) >> 16;
            X3 = ((X1 + X2) + 2) >> 2;
            B4 = ((uint32_t)ac4_ * (uint32_t)(X3 + 32768)) >> 15;
            B7 = ((uint32_t)UP - B3) * (uint32_t)(50000UL >> BMP085_OVERSAMPLING_MODE);


            if (B7 < 0x80000000)
            {
                p = (B7 * 2) / B4;
            }
            else
            {
                p = (B7 / B4) * 2;
            }

            X1 = (p >> 8) * (p >> 8);
            X1 = (X1 * 3038) >> 16;
            X2 = (- 7357 * p) >> 16;

            p = p + ((X1 + X2 + (int32_t)3791) >> 4);

            // Store current pressure
            pressure_ = p;

            // Keep old raw altitude value
            altitude_raw_old = altitude_raw_;

            // Compute new altitude from pressure
            altitude_raw_ = altitude_from_pressure(pressure_, 0);

            // Update circular buffer with last 3 altitudes
            for (int32_t i = 0; i < 2; i++)
            {
                last_altitudes_[i] = last_altitudes_[i + 1];
            }
            last_altitudes_[2] = altitude_raw_;

            // Apply median filter on last 3 altitudes
            altitude_median = maths_median_filter_3x(last_altitudes_[0], last_altitudes_[1], last_altitudes_[2]);

            // Keep old, filtered altitude
            altitude_filtered_old = altitude_filtered;

            // Low pass filter the altitude, only if this is not a spike
            if (maths_f_abs(altitude_median - altitude_filtered_old) < 15.0f)
            {
                altitude_filtered = (BARO_ALT_LPF * altitude_filtered_old) + (1.0f - BARO_ALT_LPF) * altitude_median;
            }
            else
            {
                altitude_filtered = altitude_median;
            }
            
            // remove bias
            altitude_gf_raw_ = altitude_raw_ - altitude_bias_gf_;
            altitude_gf_ = altitude_filtered - altitude_bias_gf_;

            // Time interval since last update
            dt_s_ = (time_keeper_get_us() - last_update_us_) / 1000000.0f;

            // Compute new vertical speed from two last filtered altitudes
            new_speed_lf_raw = - (altitude_raw_ - altitude_raw_old) / dt_s_;
            new_speed_lf = - (altitude_filtered - altitude_filtered_old) / dt_s_;

            // Remove spikes
            if (maths_f_abs(new_speed_lf) > 20)
            {
                new_speed_lf = 0.0f;
            }

            // Low pass filter vertical speed
            speed_lf_raw_ = new_speed_lf_raw;
            speed_lf_ = (VARIO_LPF) * speed_lf_ + (1.0f - VARIO_LPF) * (new_speed_lf);

            // Update timing
            last_update_us_ = time_keeper_get_us();
            state_          = BMP085_IDLE;
            break;
    }

    last_state_update_us_ = time_keeper_get_us();

    return res;
}
