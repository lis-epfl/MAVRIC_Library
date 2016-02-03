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
 * \file mavlink_stream.c
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

extern "C"
{
#include "util/print_util.h"
#include "hal/common/time_keeper.hpp"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool mavlink_stream_init(mavlink_stream_t* mavlink_stream,
                         const mavlink_stream_conf_t* config,
                         Serial* serial)
{
    bool success = false;

    // Init static variable storing number of mavlink stream instances
    static uint8_t nb_mavlink_stream_instances = 0;
    if (nb_mavlink_stream_instances < MAVLINK_COMM_NUM_BUFFERS)
    {
        mavlink_stream->mavlink_channel =  nb_mavlink_stream_instances;
        nb_mavlink_stream_instances 	+= 1;

        mavlink_stream->serial 			  = serial;
        mavlink_stream->sysid             = config->sysid;
        mavlink_stream->compid            = config->compid;
        mavlink_stream->msg_available     = false;
        mavlink_stream->debug          = config->debug;

        success = true;
    }
    else
    {
        // ERROR !
        if (config->debug == true)
        {
            print_util_dbg_print("[MAVLINK STREAM] Error: Too many instances !\r\n");
            print_util_dbg_print("[MAVLINK STREAM] Try to increase MAVLINK_COMM_NUM_BUFFERS\r\n");
        }

        success = false;
    }

    return success;
}


bool mavlink_stream_send(const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg)
{
    bool success = true;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    // Send byte per byte
    if (mavlink_stream->serial->writeable() >= len)
    {
        success &= mavlink_stream->serial->write(buf, len);
    }
    else
    {
        success = false;
    }

    return success;
}


bool mavlink_stream_receive(mavlink_stream_t* mavlink_stream)
{
    uint8_t byte;
    mavlink_received_t* rec = &mavlink_stream->rec;

    // Try to decode bytes until a message is complete, or there is nothing left to read
    while ((mavlink_stream->msg_available == false) && (mavlink_stream->serial->readable() > 0))
    {
        // read one byte
        mavlink_stream->serial->read(&byte);

        // Use the byte to decode current message
        if (mavlink_parse_char(mavlink_stream->mavlink_channel, byte, &rec->msg, &rec->status))
        {
            // If message was sucessfully decoded, exit while loop
            mavlink_stream->msg_available = true;
        }
    }

    return mavlink_stream->msg_available;
}


void mavlink_stream_flush(mavlink_stream_t* mavlink_stream)
{
    mavlink_stream->serial->flush();
}