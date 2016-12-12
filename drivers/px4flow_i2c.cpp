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

Px4flow_i2c::Px4flow_i2c(I2c& i2c, conf_t config):
  flow_x_(0.0f),
  flow_y_(0.0f),
  flow_quality_(0),
  velocity_x_(0.0f),
  velocity_y_(0.0f),
  velocity_z_(0.0f),
  ground_distance_(0.0f),
  last_update_s_(0.0f),
  is_healthy_(false),
  i2c_(i2c),
  config_(config)
{
    // fill buffer
    while(ground_distance_buffer_.put(0.0f))
    {;}
}

bool Px4flow_i2c::healthy(void) const
{
    return is_healthy_;
}

#include "util/print_util.hpp"
bool Px4flow_i2c::update(void)
{
    bool res = true;

    // Send command
    uint8_t buff = GET_INTEGRAL_FRAME_COMMAND;
    // uint8_t buff = GET_FRAME_COMMAND;
    res &= i2c_.write(&buff, 1, config_.i2c_address);

    if (res == true)
    {
        uint8_t rec[25];
        res &= i2c_.read(rec, 25, config_.i2c_address);

        if (res == true)
        {
            // Read optic flow and convert from decirad/s to rad/s
            // float flow_x_raw       = 0.0001f * (float)((int16_t)(rec[3] << 8 | rec[2]));
            // float flow_y_raw       = 0.0001f * (float)((int16_t)(rec[5] << 8 | rec[4]));
            float flow_x_raw       = 1.234f; // * (float)((int16_t)(rec[3] << 8 | rec[2]));
            float flow_y_raw       = (float)((int16_t)(rec[5] << 8 | rec[4]));

            float gyro_x = 0.0001f * (float)((int16_t)rec[7]  << 8 | rec[6]);
            float gyro_y = 0.0001f * (float)((int16_t)rec[9]  << 8 | rec[8]);
            float dt = 1e-6f * (float)((uint32_t)rec[15] << 24 | (uint32_t)rec[14] << 16 | (uint32_t)rec[13] << 8 | rec[12]);
            float new_distance = 0.001f * (float)((int16_t)(rec[21] << 8 | rec[20]));

            print_util_dbg_print("\r\n");
            print_util_dbg_putfloat(flow_x_raw, 3);
            print_util_dbg_print_num((int16_t)(rec[5] << 8 | rec[4]), 10);

            flow_quality_ = rec[24];
            if ((flow_quality_ > 50) && (new_distance < 4.5f))
            {
                is_healthy_ = true;
                ground_distance_buffer_.put_lossy(new_distance);
            }
            else
            {
                is_healthy_ = false;
            }

            // Get new filtered ground distance from 3 last measures
            float gd[3] = {0.0f, 0.0f, 0.0f};
            ground_distance_buffer_.get_element(0, gd[0]);
            ground_distance_buffer_.get_element(1, gd[1]);
            ground_distance_buffer_.get_element(2, gd[2]);
            float new_distance_filtered = 0.2f * ground_distance_ + 0.8f * maths_median_filter_3x(gd[0], gd[1], gd[2]);

            // Kepp values
            velocity_z_      = - (new_distance_filtered - ground_distance_) / (time_keeper_get_s() - last_update_s_);
            ground_distance_ = new_distance_filtered;

            // Compensate flow using gyroscope
            float flow_x_comp = flow_x_raw - gyro_x;
            float flow_y_comp = flow_y_raw - gyro_y;

            // Get angular velocity from integrated flow
            // if (dt != 0.0f)
            // {
            //     flow_x_comp /= dt;
            //     flow_y_comp /= dt;
            // }

            // Compute velocity using flow and measured distance
            float velocity_x_comp = flow_x_comp * ground_distance_;
            float velocity_y_comp = flow_y_comp * ground_distance_;

            // Apply rotation according to how the camera is mounted
            rotate_raw_values(flow_x_comp, flow_y_comp, velocity_x_comp, velocity_y_comp);
        }
    }
    else
    {
        is_healthy_ = false;
    }

    last_update_s_ = time_keeper_get_s();

    return res;
}

float Px4flow_i2c::flow_x(void) const
{
    return flow_x_;
}

float Px4flow_i2c::flow_y(void) const
{
    return flow_y_;
}

uint8_t Px4flow_i2c::flow_quality(void) const
{
    return flow_quality_;
}

float Px4flow_i2c::velocity_x(void) const
{
    return velocity_x_;
}

float Px4flow_i2c::velocity_y(void) const
{
    return velocity_y_;
}

float Px4flow_i2c::velocity_z(void) const
{
    return velocity_z_;
}

float Px4flow_i2c::ground_distance(void) const
{
    return ground_distance_;
}

float Px4flow_i2c::last_update_s(void) const
{
    return last_update_s_;
}

void Px4flow_i2c::rotate_raw_values(float flow_x_raw, float flow_y_raw, float velocity_x_raw, float velocity_y_raw)
{
    switch (config_.orientation)
    {
        case ORIENT_0_DEG:
            flow_x_     = flow_x_raw;
            flow_y_     = flow_y_raw;
            velocity_x_ = velocity_x_raw;
            velocity_y_ = velocity_y_raw;
        break;

        case ORIENT_90_DEG:
            flow_x_     = - flow_y_raw;
            flow_y_     = flow_x_raw;
            velocity_x_ = - velocity_y_raw;
            velocity_y_ = velocity_x_raw;
        break;

        case ORIENT_180_DEG:
            flow_x_     = - flow_x_raw;
            flow_y_     = - flow_y_raw;
            velocity_x_ = - velocity_x_raw;
            velocity_y_ = - velocity_y_raw;
        break;

        case ORIENT_270_DEG:
            flow_x_     = flow_y_raw;
            flow_y_     = - flow_x_raw;
            velocity_x_ = velocity_y_raw;
            velocity_y_ = - velocity_x_raw;
        break;
    }
}
