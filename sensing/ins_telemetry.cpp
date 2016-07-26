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
 * \file ins_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief   Telemetry for Inertial Navigation System
 *
 ******************************************************************************/

#include "sensing/ins_telemetry.hpp"

void ins_telemetry_send(const INS_kf* ins, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    float cov[45];
    mavlink_msg_local_position_ned_cov_pack(mavlink_stream->sysid(),
                                            mavlink_stream->compid(),
                                            msg,
                                            time_keeper_get_ms(),
                                            time_keeper_get_ms(),
                                            0,
                                            ins->position_lf()[0],
                                            ins->position_lf()[1],
                                            ins->position_lf()[2],
                                            ins->velocity_lf()[0],
                                            ins->velocity_lf()[1],
                                            ins->velocity_lf()[2],
                                            0.0f,
                                            0.0f,
                                            ins->absolute_altitude(),
                                            cov);
}
