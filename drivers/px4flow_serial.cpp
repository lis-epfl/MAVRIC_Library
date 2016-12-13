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

PX4Flow_serial::PX4Flow_serial(Serial& uart):
    PX4Flow(),
    uart_(uart),
    mavlink_stream_(uart_, Mavlink_stream::default_config())
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
                mavlink_optical_flow_rad_t optical_flow_msg;
                // Decode message
                mavlink_msg_optical_flow_rad_decode(msg, &optical_flow_msg);
            }
            break;


            default:
            break;
        }
    }

    return true;
}
