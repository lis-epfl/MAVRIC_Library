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
 * \file sonar_i2cxl.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for the sonar module using i2C communication protocol
 *
 ******************************************************************************/


#include "drivers/sonar_i2cxl.hpp"

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
#include "util/constants.h"
}

const float   SONAR_I2CXL_LPF_VARIO 				= 0.4f;		///< Low pass filter for velocity estimation

const uint8_t SONAR_I2CXL_RANGE_COMMAND				= 0x51;		///< Address of the Range Command Register
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_1	= 0xAA;		///< Address of the Change Command address Register 1
const uint8_t SONAR_I2CXL_CHANGE_ADDRESS_COMMAND_2	= 0xA5;		///< Address of the Change Command address Register 2


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Sonar_i2cxl::Sonar_i2cxl(I2c& i2c, sonar_i2cxl_conf_t config):
    i2c_(i2c),
    config_(config),
    distance_(0.2f),
    velocity_(0.0f),
    healthy_(false),
    last_update_us_(time_keeper_get_us())
{}


bool Sonar_i2cxl::init(void)
{
    // Try to probe the device
    return i2c_.probe(config_.i2c_address);
}


bool Sonar_i2cxl::update(void)
{
    bool res = true;

    // Get measure
    res &= get_last_measure();
    res &= send_range_command();

    // Update timing
    last_update_us_ = time_keeper_get_us();

    return res;
}


const float& Sonar_i2cxl::last_update_us(void) const
{
    return last_update_us_;
}


const std::array<float, 3>& Sonar_i2cxl::orientation_bf(void) const
{
    return config_.orientation_bf;
}


const float& Sonar_i2cxl::distance(void) const
{
    return distance_;
}


const float& Sonar_i2cxl::velocity(void) const
{
    return velocity_;
}


const bool& Sonar_i2cxl::healthy(void) const
{
    return healthy_;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Sonar_i2cxl::send_range_command(void)
{
    bool res;

    uint8_t buff = SONAR_I2CXL_RANGE_COMMAND;
    res = i2c_.write(&buff, 1, config_.i2c_address);

    return res;
}


bool Sonar_i2cxl::get_last_measure(void)
{
    bool res;
    uint8_t buf[2];
    uint16_t distance_cm 	= 0;
    float distance_m 		= 0.0f;
    float new_velocity 		= 0.0f;
    float dt_s 				= 0.0f;
    float time_us 			= time_keeper_get_us();

    res = i2c_.read(buf, 2, config_.i2c_address);

    distance_cm = (buf[0] << 8) + buf[1];
    distance_m  = ((float)distance_cm) / 100.0f;

    if (distance_m > config_.min_distance && distance_m < config_.max_distance)
    {
        dt_s = (time_us - last_update_us_) / 1000000.0f;

        // Update velocity
        if (healthy_ && dt_s != 0.0f)
        {
            new_velocity = (distance_m - distance_) / dt_s;

            //discard sonar new_velocity estimation if it seems too big
            if (maths_f_abs(new_velocity) > 20.0f)
            {
                new_velocity = 0.0f;
            }

            velocity_ = (1.0f - SONAR_I2CXL_LPF_VARIO) * velocity_
                        + SONAR_I2CXL_LPF_VARIO      * new_velocity;
        }
        else
        {
            velocity_ = 0.0f;
        }

        distance_  		= distance_m;
        last_update_us_ = time_us;
        healthy_ 		= true;
    }
    else
    {
        velocity_	= 0.0f;
        healthy_ 	= false;
    }

    return res;
}


//------------------------------------------------------------------------------
// GLUE FUNCTION (TEMPORARY)
//------------------------------------------------------------------------------
bool sonar_i2cxl_update(Sonar_i2cxl* sonar)
{
    return sonar->update();
}