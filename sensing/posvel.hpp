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
 * \file posvel.hpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Position and velocity estimate
 *
 ******************************************************************************/


#ifndef POSVEL_HPP_
#define POSVEL_HPP_

#include "communication/mavlink_stream.hpp"
#include "hal/common/time_keeper.hpp"

struct posvel_t
{
    float pos[3];
    float vel[3];
};

// void posvel_telemetry_send(const posvel_t* posvel, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg);
static inline void posvel_telemetry_send(const posvel_t* posvel, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    float cov[45];
    mavlink_msg_local_position_ned_cov_pack(mavlink_stream->sysid(),
                                            mavlink_stream->compid(),
                                            msg,
                                            time_keeper_get_ms(),
                                            time_keeper_get_ms(),
                                            0,
                                            posvel->pos[0],
                                            posvel->pos[1],
                                            posvel->pos[2],
                                            posvel->vel[0],
                                            posvel->vel[1],
                                            posvel->vel[2],
                                            0.0f,
                                            0.0f,
                                            0.0f,
                                            cov);
}

#endif /* POSVEL_HPP_ */
