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
 * \file mavlink_stream.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief A wrapper for MAVLink to use the stream interface
 *
 ******************************************************************************/


#include "communication/mavlink_stream.hpp"
#include "communication/onboard_parameters.hpp"
#include "communication/mavlink_message_handler.hpp"
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_stream::Mavlink_stream(Serial& serial, const conf_t& config) : serial_(serial)
{
    // Init static variable storing number of mavlink stream instances
    static uint8_t nb_mavlink_stream_instances = 0;
    if (nb_mavlink_stream_instances < MAVLINK_COMM_NUM_BUFFERS)
    {
        mavlink_channel_ =  nb_mavlink_stream_instances;
        nb_mavlink_stream_instances     += 1;
    }
    else
    {
        // ERROR !
        if (config.debug == true)
        {
            print_util_dbg_print("[MAVLINK STREAM] Error: Too many instances !\r\n");
            print_util_dbg_print("[MAVLINK STREAM] Try to increase MAVLINK_COMM_NUM_BUFFERS\r\n");
        }
    }

    sysid_          = config.sysid;
    compid_         = config.compid;
    debug_          = config.debug;
}

Mavlink_stream::conf_t Mavlink_stream::default_config(void)
{
    Mavlink_stream::conf_t conf = {};
    conf. sysid = 1;
    conf.compid = 50;
    conf.debug  = false;
    return conf;
}


bool Mavlink_stream::send(mavlink_message_t* msg) const
{
    bool success = true;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    // Send byte per byte
    if (serial_.writeable() >= len)
    {
        success &= serial_.write(buf, len);
    }
    else
    {
        success = false;
    }

    return success;
}


bool Mavlink_stream::receive(Mavlink_stream::msg_received_t* rec)
{
    uint8_t byte;

    // Try to decode bytes until a message is complete, or there is nothing left to read
    while (serial_.readable() > 0)
    {
        // read one byte
        serial_.read(&byte);

        // Use the byte to decode current message
        if (mavlink_parse_char(mavlink_channel_, byte, &rec->msg, &rec->status))
        {
            return true;
        }
    }
    return false;
}

void Mavlink_stream::flush()
{
    serial_.flush();
}
