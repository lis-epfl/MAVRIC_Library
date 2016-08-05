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
    // Fill the covaiance array
    float cov[45];
    // First elements are the diagonal
    for(int i = 0; i < 11; i++)
    {
        cov[i] = ins->P()(i,i);
    }
    // Last elements are the noisy GPS postions and speed
    cov[20] = ins->x()[3];
    cov[30] = ins->x()[10];
    cov[31] = ins->barometer_.altitude_gf_raw() - INS::origin().altitude;
    cov[32] = -(ins->barometer_.altitude_gf_raw() - INS::origin().altitude + ins->x_[10]);
    cov[39] = ins->gps_local[0];
    cov[40] = ins->gps_local[1];
    cov[41] = ins->gps_local[2];
    cov[42] = ins->gps_velocity[0];
    cov[43] = ins->gps_velocity[1];
    cov[44] = ins->gps_velocity[2];

    // Send message
    mavlink_msg_local_position_ned_cov_pack(mavlink_stream->sysid(),
                                            mavlink_stream->compid(),
                                            msg,
                                            time_keeper_get_ms(),
                                            time_keeper_get_ms(),
                                            0,
                                            ins->x()[0],
                                            ins->x()[1],
                                            ins->x()[2],
                                            ins->x()[4],
                                            ins->x()[5],
                                            ins->x()[6],
                                            ins->x()[7],
                                            ins->x()[8],
                                            ins->x()[9],
                                            cov);

    // mavlink_msg_hil_sensor_pack(mavlink_stream->sysid(),
    //                             mavlink_stream->compid(),
    //                             msg,
    //                             time_keeper_get_us(),
    //                             ins->x()[0],    // xacc <-> x
    //                             ins->x()[1],    // yacc <-> y
    //                             ins->x()[2],    // zacc <-> z
    //                             ins->x()[4],    // xgyro <-> vx
    //                             ins->x()[5],    // ygyro <-> vy
    //                             ins->x()[6],    // zgyro <-> vz
    //                             ins->x()[7],    // xmag <-> bias_accx
    //                             ins->x()[8],    // ymag <-> bias_accy
    //                             ins->x()[9],    // zmag <-> bias_accz
    //                             ins->x()[3],    // abs_pressure <-> z_gnd
    //                             ins->x()[10],   // diff_pressure <-> bias_baro
    //                             0.0f,           // pressure_alt <-> 0.0f
    //                             0.0f,           // temperature <-> 0.0f
    //                             0);             // fields_updated <-> 0
}
