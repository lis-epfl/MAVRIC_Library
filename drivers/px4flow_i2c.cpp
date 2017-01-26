/*******************************************************************************
 * Copyright (c) 2009-2015, MAV'RIC Development Team
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
 * \file px4flow_i2c.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Driver for PX4Flow optical flow smart camera using I2C
 *
 ******************************************************************************/

#include "drivers/px4flow_i2c.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/maths.h"

PX4Flow_i2c::PX4Flow_i2c(I2c& i2c, conf_t config):
    PX4Flow(),
    i2c_(i2c),
    config_(config)
{}


bool PX4Flow_i2c::update(void)
{
    bool res = true;

    // Send command
    uint8_t buff = GET_FRAME_COMMAND;
    res &= i2c_.write(&buff, 1, config_.i2c_address);

    uint8_t rec[22];
    res &= i2c_.read(rec, 22, config_.i2c_address);

    if (res == true)
    {
        // Read optic flow and convert from decirad/s to rad/s
        float flow_x_raw       = 0.1f * (float)((int16_t)(rec[3] << 8 | rec[2]));
        float flow_y_raw       = 0.1f * (float)((int16_t)(rec[5] << 8 | rec[4]));

        // Read velocity and convert from mm/s to m/s
        float velocity_x_raw = 0.001f * (float)((int16_t)(rec[7] << 8 | rec[6]));
        float velocity_y_raw = 0.001f * (float)((int16_t)(rec[9] << 8 | rec[8]));
        flow_quality_ = (uint8_t)((int16_t)(rec[11] << 8 | rec[10]));

        // Apply rotation according to how the camera is mounted
        rotate_raw_values(config_.orientation, flow_x_raw, flow_y_raw, velocity_x_raw, velocity_y_raw);

        float new_distance = 0.001f * (float)((int16_t)(rec[21] << 8 | rec[20]));
        if (new_distance < 4.5f)
        {
            ground_distance_buffer_.put_lossy(new_distance);
        }

        // Get new filtered ground distance from 3 last measures
        float gd[3] = {0.0f, 0.0f, 0.0f};
        ground_distance_buffer_.get_element(0, gd[0]);
        ground_distance_buffer_.get_element(1, gd[1]);
        ground_distance_buffer_.get_element(2, gd[2]);
        float new_distance_filtered = 0.2f * ground_distance_ + 0.8f * maths_median_filter_3x(gd[0], gd[1], gd[2]);

        // Keep values
        velocity_z_      = - (new_distance_filtered - ground_distance_) / (time_keeper_get_s() - last_update_s_);
        ground_distance_ = new_distance_filtered;

        // Update healthiness
        if ((flow_quality_ > 30) && (ground_distance_ < 4.5f))
        {
            is_healthy_ = true;
        }
        else
        {
            is_healthy_ = false;
        }
    }
    else
    {
        is_healthy_ = false;
    }

    last_update_s_ = time_keeper_get_s();

    return res;
}
