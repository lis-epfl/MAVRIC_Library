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
 * \file px4flow_serial.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#include "drivers/px4flow_serial.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "hal/common/mavric_endian.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

PX4Flow_serial::PX4Flow_serial(Serial& uart, conf_t config):
    PX4Flow(),
    uart_(uart),
    mavlink_stream_(uart_, Mavlink_stream::default_config()),
    config_(config)
{}


bool PX4Flow_serial::update(void)
{
    Mavlink_stream::msg_received_t rec;

    // Receive incoming bytes
    while (mavlink_stream_.receive(&rec))
    {
        // Get pointer to new message
        mavlink_message_t* msg = &rec.msg;

        switch (msg->msgid)
        {
            case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
            {
                // Decode message
                mavlink_optical_flow_rad_t of_msg;
                mavlink_msg_optical_flow_rad_decode(msg, &of_msg);


                // Get optic flow
                flow_quality_  = of_msg.quality;
                flow_x_ = (of_msg.integrated_y - of_msg.integrated_ygyro);
                flow_y_ = - (of_msg.integrated_x - of_msg.integrated_xgyro);
                if (of_msg.integration_time_us != 0)
                {
                    flow_x_ /= (1e-6f * (float)(of_msg.integration_time_us));
                    flow_y_ /= (1e-6f * (float)(of_msg.integration_time_us));
                }

                // Get new distance
                float new_distance = of_msg.distance;
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
                time_s_t dt_s = (time_keeper_get_us() - last_update_us_) / 1e6f;
                velocity_z_      = - (new_distance_filtered - ground_distance_) / dt_s;
                ground_distance_ = new_distance_filtered;

                // Compute XY velocity
                velocity_x_ = flow_x_ * ground_distance_;
                velocity_y_ = flow_y_ * ground_distance_;

                // Apply rotation according to how the camera is mounted
                rotate_raw_values(config_.orientation, flow_x_, flow_y_, velocity_x_, velocity_y_);

                // Update healthiness
                last_update_us_ = time_keeper_get_us();
                if (ground_distance_ < 4.5f)
                {
                    is_healthy_ = true;
                }
                else
                {
                    is_healthy_ = false;
                }
            }
            break;


            default:
            break;
        }
    }

    return true;
}
