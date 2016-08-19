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
 * \file dynamic_model_telemetry.cpp
 *
 * \author MAV'RIC Team
 * \author Basil Huber
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the dynamic model
 *
 ******************************************************************************/


#include "simulation/dynamic_model_telemetry.hpp"
#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"
//#include "util/coord_conventions.hpp"


void dynamic_model_telemetry_send_state_quaternion(const Dynamic_model* model, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
      const quat_t attitude_quat = model->attitude();
      const float attitude[4] = {attitude_quat.s, attitude_quat.v[X], attitude_quat.v[Y], attitude_quat.v[Z]};
      const std::array<float, 3>& rate = model->angular_velocity_bf();
      const global_position_t& position = model->position_gf();
      const std::array<float, 3>& velocity = model->velocity_lf();
      const uint16_t speed = vectors_norm(velocity.data());
      const std::array<float, 3>& acc = model->acceleration_bf();

      mavlink_msg_hil_state_quaternion_pack(mavlink_stream->sysid(),
                                          mavlink_stream->compid(),
                                          msg,
                                          time_keeper_get_ms(),                     // Timestamp (microseconds since UNIX epoch or microseconds since system boot)
                                          attitude,                                 // attitude_quaternion Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)
                                          rate[ROLL],                               // Body frame roll / phi angular speed (rad/s)
                                          rate[PITCH],                              // Body frame pitch / theta angular speed (rad/s)
                                          rate[YAW],                                // Body frame yaw / psi angular speed (rad/s)
                                          (int32_t)(position.latitude  * 1e7),      // Latitude, expressed as * 1E7
                                          (int32_t)(position.longitude * 1e7),      // Longitude, expressed as * 1E7,      //
                                          (int32_t)(position.altitude  * 1e3),      // Altitude in meters, expressed as * 1000 (millimeters)
                                          (int16_t)(velocity[X] * 1e2),             // Ground X Speed (Latitude), expressed as m/s * 100
                                          (int16_t)(velocity[Y] * 1e2),             // Ground Y Speed (Longitude), expressed as m/s * 100
                                          (int16_t)(velocity[Z] * 1e2),             // Ground Z Speed (Altitude), expressed as m/s * 100
                                          (uint16_t)(speed * 1e2),                  // Indicated airspeed, expressed as m/s * 100 [CURRENTLY NORM OF VELOCITY]
                                          (uint16_t)(speed * 1e2),                  // True airspeed, expressed as m/s * 100     [CURRENTLY NORM OF VELOCITY]
                                          (int16_t)(acc[X] * 1e3 * GRAVITY),        // X acceleration (mg) [BODY FRAME]
                                          (int16_t)(acc[Y] * 1e3 * GRAVITY),        // Y acceleration (mg) [BODY FRAME]
                                          (int16_t)(acc[Z] * 1e3 * GRAVITY));       // Z acceleration (mg) [BODY FRAME]
}