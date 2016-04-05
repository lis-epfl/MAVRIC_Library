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
 * \file servos_telemetry.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *
 * \brief This module takes care of sending periodic telemetric messages for
 * the servos
 *
 ******************************************************************************/


#include "drivers/servos_telemetry.hpp"

extern "C"
{
#include "hal/common/time_keeper.hpp"
}

void servos_telemetry_init(servos_telemetry_t* servos_telemetry, Servo* servo_0, Servo* servo_1, Servo* servo_2, Servo* servo_3)
{
    servos_telemetry->servos[0] = servo_0;
    servos_telemetry->servos[1] = servo_1;
    servos_telemetry->servos[2] = servo_2;
    servos_telemetry->servos[3] = servo_3;
}

void servos_telemetry_mavlink_send(servos_telemetry_t* servos_telemetry, Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    mavlink_msg_servo_output_raw_pack(mavlink_stream->sysid,
                                      mavlink_stream->compid,
                                      msg,
                                      time_keeper_get_us(),
                                      0,
                                      (uint16_t)(1500 + 500 * servos_telemetry->servos[0]->read()),
                                      (uint16_t)(1500 + 500 * servos_telemetry->servos[1]->read()),
                                      (uint16_t)(1500 + 500 * servos_telemetry->servos[2]->read()),
                                      (uint16_t)(1500 + 500 * servos_telemetry->servos[3]->read()),
                                      0,
                                      0,
                                      0,
                                      0
                                      //(uint16_t)( 1500 + 500 * servos_telemetry->servos[4]->read() ),
                                      //(uint16_t)( 1500 + 500 * servos_telemetry->servos[5]->read() ),
                                      //(uint16_t)( 1500 + 500 * servos_telemetry->servos[6]->read() ),
                                      //(uint16_t)( 1500 + 500 * servos_telemetry->servos[7]->read() )
                                     );
}