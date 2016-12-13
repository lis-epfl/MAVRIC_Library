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
 * \file px4flow.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Interface for Optic Flow sensors
 *
 ******************************************************************************/

#include "drivers/px4flow.hpp"

PX4Flow::PX4Flow(void):
    flow_x_(0.0f),
    flow_y_(0.0f),
    flow_quality_(0.0f),
    velocity_x_(0.0f),
    velocity_y_(0.0f),
    velocity_z_(0.0f),
    ground_distance_(0.0f),
    last_update_s_(0.0f),
    is_healthy_(false)
{
    // fill buffer
    while(ground_distance_buffer_.put(0.0f))
    {;}
}

bool PX4Flow::healthy(void) const
{
    return is_healthy_;
}

float PX4Flow::flow_x(void) const
{
    return flow_x_;
}

float PX4Flow::flow_y(void) const
{
    return flow_y_;
}

uint8_t PX4Flow::flow_quality(void) const
{
    return flow_quality_;
}

float PX4Flow::velocity_x(void) const
{
    return velocity_x_;
}

float PX4Flow::velocity_y(void) const
{
    return velocity_y_;
}

float PX4Flow::velocity_z(void) const
{
    return velocity_z_;
}

float PX4Flow::ground_distance(void) const
{
    return ground_distance_;
}

float PX4Flow::last_update_s(void) const
{
    return last_update_s_;
}

void PX4Flow::rotate_raw_values(orientation_t orientation, float flow_x_raw, float flow_y_raw, float velocity_x_raw, float velocity_y_raw)
{
    switch (orientation)
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
