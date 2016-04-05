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
 * \file sonar_telemetry.c
 *
* \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for sonars
 *
 ******************************************************************************/


#include "drivers/sonar_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
#include "util/maths.h"
}

void sonar_telemetry_send(const Sonar* sonar, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_distance_sensor_pack(mavlink_stream->sysid,
                                     mavlink_stream->compid,
                                     msg,
                                     time_keeper_get_ms(),
                                     maths_f_max(-sonar->velocity() * 100.0f, 0.0f),        // min distance => we send negative velocity instead
                                     maths_f_max(sonar->velocity() * 100.0f, 0.0f),     // max distance => we send positive velocity instead
                                     sonar->distance() * 100,       // distance
                                     MAV_DISTANCE_SENSOR_ULTRASOUND,    // type
                                     0,                                 // id 0
                                     0,                                 // orientation 0
                                     sonar->healthy());             // covariance
}