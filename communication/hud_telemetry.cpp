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
 * \file hud_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Gregoire Heitz
 *
 * \brief This file sends the MAVLink HUD message
 *
 ******************************************************************************/


#include "communication/hud_telemetry.hpp"
#include "communication/mavlink_communication.hpp"
#include "util/coord_conventions.hpp"
#include "util/print_util.hpp"


bool hud_telemetry_init(hud_telemetry_t* hud, const INS* ins, const command_t* controls, const AHRS* ahrs)
{
    bool init_success = true;

    hud->ahrs       = ahrs;
    hud->controls   = controls;
    hud->ins        = ins;

    return init_success;
}

void hud_telemetry_send_message(const hud_telemetry_t* hud, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    std::array<float,3> vel = hud->ins->velocity_lf();
    float groundspeed       = sqrt(vel[0] * vel[0] + vel[1] * vel[1]);
    float airspeed          = groundspeed;

    aero_attitude_t aero_attitude;
    aero_attitude = coord_conventions_quat_to_aero(hud->ahrs->attitude());

    int16_t heading;
    if (aero_attitude.rpy[2] < 0)
    {
        heading = (int16_t)(360.0f + 180.0f * aero_attitude.rpy[2] / PI); //you want to normalize between 0 and 360°
    }
    else
    {
        heading = (int16_t)(180.0f * aero_attitude.rpy[2] / PI);
    }

    float thrust = vectors_norm(hud->controls->thrust.xyz.data());

    mavlink_msg_vfr_hud_pack(mavlink_stream->sysid(),
                             mavlink_stream->sysid(),
                             msg,
                             airspeed,
                             groundspeed,
                             heading,
                             (int32_t)(thrust * 50),
                             hud->ins->absolute_altitude(),
                             -vel[2]);
}
