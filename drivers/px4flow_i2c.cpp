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

Px4flow_i2c::Px4flow_i2c(I2c& i2c, conf_t config):
  flow_x_(0.0f),
  flow_y_(0.0f),
  flow_quality_(0),
  velocity_x_(0.0f),
  velocity_y_(0.0f),
  ground_distance_(0.0f),
  last_update_s_(0.0f),
  i2c_(i2c),
  config_(config)
{}

bool Px4flow_i2c::update(void)
{
    bool res = true;

    // Send command
    uint8_t buff = GET_FRAME_COMMAND;
    res = i2c_.write(&buff, 1, config_.i2c_address);

    uint8_t rec[22];
    res = i2c_.read(rec, 22, config_.i2c_address);

    if (res == true)
    {
        flow_x_       = 0.1f * (float)((int16_t)(rec[3] << 8 | rec[2]));
        flow_y_       = 0.1f * (float)((int16_t)(rec[5] << 8 | rec[4]));
        velocity_x_ = 0.001f * (float)((int16_t)(rec[7] << 8 | rec[6]));
        velocity_y_ = 0.001f * (float)((int16_t)(rec[9] << 8 | rec[8]));
        flow_quality_ = (uint8_t)((int16_t)(rec[11] << 8 | rec[10]));
        ground_distance_ = 0.001f * (float)((int16_t)(rec[21] << 8 | rec[20]));

        last_update_s_ = time_keeper_get_s();
    }

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

float Px4flow_i2c::ground_distance(void) const
{
    return ground_distance_;
}

float Px4flow_i2c::last_update_s(void) const
{
    return last_update_s_;
}
