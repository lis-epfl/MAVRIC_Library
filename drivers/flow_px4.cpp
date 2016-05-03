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
 * \file flow.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief Driver for optic flow sensors
 *
 ******************************************************************************/

#include "drivers/flow_px4.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "hal/common/mavric_endian.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Flow_px4::Flow_px4(Serial& uart):
  uart_(uart),
  mavlink_stream_(uart_, Mavlink_stream::default_config())
{
    // Init members
    last_update_us  = time_keeper_get_us();
    handshake_state = FLOW_NO_HANDSHAKE;
    of_count        = 0;
}


bool Flow_px4::update(void)
{
    Mavlink_stream::msg_received_t rec;

    // Receive incoming bytes
    while (mavlink_stream_.receive(&rec))
    {
        // Get pointer to new message
        mavlink_message_t* msg = &rec.msg;

        // declare messages
        mavlink_optical_flow_t          optical_flow_msg;
        mavlink_data_transmission_handshake_t   handshake_msg;
        mavlink_encapsulated_data_t     data_msg;

        switch (msg->msgid)
        {
            case MAVLINK_MSG_ID_OPTICAL_FLOW:
                // Decode message
                mavlink_msg_optical_flow_decode(msg, &optical_flow_msg);
                break;

            case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
                // Decode message
                mavlink_msg_data_transmission_handshake_decode(msg, &handshake_msg);

                // Get type of handshake
                switch (handshake_msg.jpg_quality)
                {
                    case 0:
                        handshake_state = FLOW_HANDSHAKE_DATA;
                        break;

                    case 255:
                        handshake_state = FLOW_HANDSHAKE_METADATA;
                        break;

                    default:
                        handshake_state = FLOW_HANDSHAKE_DATA;
                        break;
                }

                // Get number of of vectors
                of_count        = handshake_msg.width;
                if (of_count > 125)
                {
                    of_count = 125;
                }

                // Get total payload size
                packet_count_       = handshake_msg.packets;
                byte_count_       = handshake_msg.size;
                if (byte_count_ > 500)
                {
                    byte_count_ = 500;
                }
                break;

            case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
                // Decode message
                mavlink_msg_encapsulated_data_decode(msg, &data_msg);

                // Handle according to hanshake state
                switch (handshake_state)
                {
                    case FLOW_HANDSHAKE_METADATA:
                        if (data_msg.seqnr < packet_count_ - 1)
                        {
                            // not last packet
                            for (uint32_t i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                            {
                                of_loc_tmp_.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                            }
                        }
                        else if (data_msg.seqnr < packet_count_)
                        {
                            // last packet
                            for (uint32_t i = 0; i < byte_count_; ++i)
                            {
                                of_loc_tmp_.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                            }

                            // swap bytes
                            for (uint32_t i = 0; i < of_count; ++i)
                            {
                                of_loc.x[i] =  endian_rev16s(of_loc_tmp_.x[i]);
                                of_loc.y[i] = endian_rev16s(of_loc_tmp_.y[i]);
                            }

                        }
                        break;

                    default:
                        if (data_msg.seqnr < (packet_count_ - 1))
                        {
                            // not last packet
                            for (uint32_t i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                            {
                                of_tmp_.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                            }
                        }
                        else if (data_msg.seqnr == (packet_count_ - 1))
                        {
                            // last packet
                            for (uint32_t i = 0; i < byte_count_ % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                            {
                                of_tmp_.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                            }

                            // swap bytes and filter for high frequency noise
                            for (int i = 0; i < of_count; ++i)
                            {
                                of.x[i] = filter_constant * 0.001f * (float)(endian_rev16s(of_tmp_.x[i])) +
                                          (1.0f - filter_constant) * of.x[i];
                                of.y[i] = filter_constant * 0.001f * (float)(endian_rev16s(of_tmp_.y[i])) +
                                          (1.0f - filter_constant) * of.y[i];
                            }

                            // Update time
                            last_update_us  = time_keeper_get_us();
                        }
                        break;
                }

            default:
            break;
        }
    }

    return true;
}
