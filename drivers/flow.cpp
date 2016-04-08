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

#include "drivers/flow.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/print_util.h"
#include "hal/common/mavric_endian.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool flow_init(flow_t* flow, Serial* uart_)
{
    bool success = true;

    // Init members
    flow->last_update_us  = time_keeper_get_us();
    flow->handshake_state = FLOW_NO_HANDSHAKE;
    flow->of_count        = 0;

    // Init uart
    flow->uart = uart_;

    // Init MAVLink stream
    mavlink_stream_conf_t mavlink_stream_conf = {};
    mavlink_stream_conf.sysid   = 1;
    mavlink_stream_conf.compid  = 50;
    mavlink_stream_conf.debug   = true;
    success &= mavlink_stream_init(&(flow->mavlink_stream),
                                   &mavlink_stream_conf,
                                   uart_);

    return success;
}


bool flow_update(flow_t* flow)
{
    // Receive incoming bytes
    while (mavlink_stream_receive(&flow->mavlink_stream))
    {
        // Check if a new message is here
        if (flow->mavlink_stream.msg_available == true)
        {
            // Get pointer to new message
            mavlink_message_t* msg = &flow->mavlink_stream.rec.msg;

            // Indicate that the message was handled
            flow->mavlink_stream.msg_available = false;

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
                            flow->handshake_state = FLOW_HANDSHAKE_DATA;
                            break;

                        case 255:
                            flow->handshake_state = FLOW_HANDSHAKE_METADATA;
                            break;

                        default:
                            flow->handshake_state = FLOW_HANDSHAKE_DATA;
                            break;
                    }

                    // Get number of of vectors
                    flow->of_count        = handshake_msg.width;
                    if (flow->of_count > 125)
                    {
                        flow->of_count = 125;
                    }

                    // Get total payload size
                    flow->n_packets       = handshake_msg.packets;
                    flow->size_data       = handshake_msg.size;
                    if (flow->size_data > 500)
                    {
                        flow->size_data = 500;
                    }
                    break;

                case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
                    // Decode message
                    mavlink_msg_encapsulated_data_decode(msg, &data_msg);

                    // Handle according to hanshake state
                    switch (flow->handshake_state)
                    {
                        case FLOW_HANDSHAKE_METADATA:
                            if (data_msg.seqnr < flow->n_packets - 1)
                            {
                                // not last packet
                                for (uint32_t i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                                {
                                    flow->of_loc_tmp.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                                }
                            }
                            else if (data_msg.seqnr < flow->n_packets)
                            {
                                // last packet
                                for (uint32_t i = 0; i < flow->size_data; ++i)
                                {
                                    flow->of_loc_tmp.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                                }

                                // swap bytes
                                for (uint32_t i = 0; i < flow->of_count; ++i)
                                {
                                    flow->of_loc.x[i] =  endian_rev16s(flow->of_loc_tmp.x[i]);
                                    flow->of_loc.y[i] = endian_rev16s(flow->of_loc_tmp.y[i]);
                                }

                            }
                            break;

                        default:
                            if (data_msg.seqnr < (flow->n_packets - 1))
                            {
                                // not last packet
                                for (uint32_t i = 0; i < MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                                {
                                    flow->of_tmp.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                                }
                            }
                            else if (data_msg.seqnr == (flow->n_packets - 1))
                            {
                                // last packet
                                for (uint32_t i = 0; i < flow->size_data % MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN; ++i)
                                {
                                    flow->of_tmp.data[i + data_msg.seqnr * MAVLINK_MSG_ENCAPSULATED_DATA_FIELD_DATA_LEN] = data_msg.data[i];
                                }

                                // swap bytes and filter for high frequency noise
                                for (int i = 0; i < flow->of_count; ++i)
                                {
                                    flow->of.x[i] = filter_constant * endian_rev16s(flow->of_tmp.x[i])+ (1-filter_constant)*flow->of.x[i];
                                    flow->of.y[i] = filter_constant * endian_rev16s(flow->of_tmp.y[i])+ (1-filter_constant)*flow->of.y[i];
                                }

                                // Update time
                                flow->last_update_us  = time_keeper_get_us();
                            }
                            break;
                    }
                    break;

                default:
                    break;
            }
        }
    }

    return true;
}
