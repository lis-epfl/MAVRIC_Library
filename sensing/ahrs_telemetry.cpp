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
 * \file ahrs_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the ahrs module
 *
 ******************************************************************************/


#include "sensing/ahrs_telemetry.hpp"
#include "util/coord_conventions.hpp"
#include "hal/common/time_keeper.hpp"


void ahrs_telemetry_send_attitude(const AHRS* ahrs, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    aero_attitude_t aero_attitude;
    aero_attitude = coord_conventions_quat_to_aero(ahrs->attitude());
    std::array<float, 3> angular_speed = ahrs->angular_speed();

    mavlink_msg_attitude_pack(mavlink_stream->sysid(),
                              mavlink_stream->compid(),
                              msg,
                              time_keeper_get_ms(),
                              aero_attitude.rpy[0],
                              aero_attitude.rpy[1],
                              aero_attitude.rpy[2],
                              angular_speed[0],
                              angular_speed[1],
                              angular_speed[2]);
}

void ahrs_telemetry_send_attitude_quaternion(const AHRS* ahrs, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    quat_t attitude                    = ahrs->attitude();
    std::array<float, 3> angular_speed = ahrs->angular_speed();

    mavlink_msg_attitude_quaternion_pack(mavlink_stream->sysid(),
                                         mavlink_stream->compid(),
                                         msg,
                                         time_keeper_get_ms(),
                                         attitude.s,
                                         attitude.v[0],
                                         attitude.v[1],
                                         attitude.v[2],
                                         angular_speed[0],
                                         angular_speed[1],
                                         angular_speed[2]);
}
