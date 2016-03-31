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
 * \file mavlink_stream.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief A wrapper for MAVLink to use the stream interface
 *
 ******************************************************************************/


#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#include "hal/common/serial.hpp"

extern "C"
{
#include <stdint.h>
#include <stdbool.h>
#include "hal/common/mavric_endian.h"
}

#ifdef __MAVRIC_ENDIAN_BIG__
#define NATIVE_BIG_ENDIAN
#endif

extern "C"
{
#include "libs/mavlink/include/mavric/mavlink.h"
}

#define MAVLINK_BASE_STATION_ID 255


/**
 * \brief   Main structure for the MAVLink stream module
 */
class Mavlink_stream
{
public:

    /**
     * \brief   Mavlink structures for the receive message and its status
     */
    typedef struct
    {
        mavlink_message_t msg;          ///< Mavlink message
        mavlink_status_t status;        ///< Status on the message
    } msg_received_t;

    /**
     * \brief   Configuration structure for the module MAVLink stream
     */
    struct conf_t
    {
        uint32_t sysid;                 ///< System ID
        uint32_t compid;                ///< System Component ID
        bool     debug;                 ///< Debug flag
    };


    Mavlink_stream(Serial& serial, const conf_t& config);


    /**
     * \brief   Send Mavlink stream
     *
     * \param   msg                 msg to stream
     *
     * \return success
     */
    bool send(mavlink_message_t* msg) const;

    /**
     * \brief   Mavlink parsing of message; the message is not available in this module afterwards
     *
     * \param   rec             Address where to store the received message
     *
     * \return  Success         True if a message was successfully decoded, false else
     */
    bool receive(msg_received_t* rec);

    /**
     * \brief   Flushing MAVLink stream
     */
    void flush();

    uint32_t sysid;             ///< System ID
    uint32_t compid;            ///< System Component ID
private:
    Serial& serial;
    uint8_t mavlink_channel;    ///< Channel number used internally by mavlink to retrieve incomplete incoming message
    bool debug;                 ///< Debug flag
};

#endif /* MAVLINK_STREAM_H */