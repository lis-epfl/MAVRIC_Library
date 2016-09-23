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
 * \author Basil Huber
 *
 * \brief   Telemetry for Inertial Navigation System
 *
 ******************************************************************************/

#include "sensing/ins_telemetry.hpp"
#include "util/print_util.hpp"


/**
 * \brief   Callback for receiving SET_GPS_GLOBAL_ORIGIN messages
 * \details Sets INS::origin to the position sent indicated in the message
 */
void ins_telemetry_set_gps_global_origin_callback(INS* ins, uint32_t sysid, mavlink_message_t* msg);



void ins_telemetry_set_gps_global_origin_callback(INS* ins, uint32_t sysid, mavlink_message_t* msg)
{
    (void) ins;   // unused
    (void) sysid; // unused

    /* decode message */
    mavlink_set_gps_global_origin_t set_gps_global_origin;
    mavlink_msg_set_gps_global_origin_decode(msg, &set_gps_global_origin);

    /* create origin */
    global_position_t origin;
    origin.latitude =  ((double)set_gps_global_origin.latitude) / 1.0e7;
    origin.longitude = ((double)set_gps_global_origin.longitude) / 1.0e7;
    origin.altitude =  ((float)set_gps_global_origin.altitude) / 1.0e3;

    /* set the origin */
    INS::set_origin(origin);
}


bool ins_telemetry_init(INS* ins, Mavlink_message_handler* message_handler)
{
    bool init_success = true;

    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id    = MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN; // 48
    callbackcmd.sysid_filter  = MAV_SYS_ID_ALL;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL;
    callbackcmd.function      = (Mavlink_message_handler::cmd_callback_func_t)   &ins_telemetry_set_gps_global_origin_callback;
    callbackcmd.module_struct =                                     ins;
    init_success &= message_handler->add_cmd_callback(&callbackcmd);

    return init_success;
}


void ins_telemetry_send_local_position_ned_cov(const INS* ins, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
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


void ins_telemetry_send_local_position_ned(const INS* ins, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    local_position_t pos_lf    = ins->position_lf();
    std::array<float,3> vel_lf = ins->velocity_lf();
    mavlink_msg_local_position_ned_pack(mavlink_stream->sysid(),
                                        mavlink_stream->compid(),
                                        msg,
                                        time_keeper_get_ms(),
                                        pos_lf[X],
                                        pos_lf[Y],
                                        pos_lf[Z],
                                        vel_lf[X],
                                        vel_lf[Y],
                                        vel_lf[Z]);
}

void ins_telemetry_send_global_position_int(const INS* ins, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    // send integrated position (for now there is no GPS error correction...!!!)
    local_position_t pos_lf    = ins->position_lf();
    std::array<float,3> vel_lf = ins->velocity_lf();

    global_position_t pos_gf;
    coord_conventions_local_to_global_position(pos_lf, ins->origin(), pos_gf);


    //mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
    mavlink_msg_global_position_int_pack(mavlink_stream->sysid(),
                                         mavlink_stream->compid(),
                                         msg,
                                         time_keeper_get_ms(),
                                         pos_gf.latitude * 10000000,
                                         pos_gf.longitude * 10000000,
                                         pos_gf.altitude * 1000.0f,
                                         -pos_lf[Z] * 1000,
                                         vel_lf[X] * 100.0f,
                                         vel_lf[Y] * 100.0f,
                                         vel_lf[Z] * 100.0f,
                                         0.0f);
}


void ins_telemetry_send_gps_global_origin(const INS* ins, const Mavlink_stream* mavlink_stream, mavlink_message_t* msg)
{
    (void) ins; // unused
    
    global_position_t origin = INS::origin();
    mavlink_msg_gps_global_origin_pack( mavlink_stream->sysid(),
                                        mavlink_stream->compid(),
                                        msg,
                                        origin.latitude * 1e7,
                                        origin.longitude * 1e7,
                                        origin.altitude * 1e3);
}
