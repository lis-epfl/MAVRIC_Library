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
 * \file waypoint.cpp
 *
 * \author MAV'RIC Team
 * \author Matthew Douglas
 *
 * \brief The waypoint class
 *
 ******************************************************************************/


#include "communication/waypoint.hpp"
#include <cstdlib>
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Waypoint::Waypoint(const Mavlink_stream& mavlink_stream_, mavlink_mission_item_t packet):
            mavlink_stream_(mavlink_stream_)
{
    command_ = packet.command;

    x_ = packet.x; // longitude
    y_ = packet.y; // latitude
    z_ = packet.z; // altitude

    autocontinue_ = packet.autocontinue;
    frame_ = packet.frame

    param1_ = packet.param1;
    param2_ = packet.param2;
    param3_ = packet.param3;
    param4_ = packet.param4;
}

Waypoint::Waypoint( const Mavlink_stream& mavlink_stream_,
                    uint8_t frame,
                    uint16_t command,
                    uint8_t autocontinue,
                    float param1,
                    float param2,
                    float param3,
                    float param4,
                    float x,
                    float y,
                    float z) :
            frame_(frame),
            command_(command),
            autocontinue_(autocontinue),
            param1_(param1),
            param2_(param2),
            param3_(param3),
            param4_(param4),
            x_(x),
            y_(y),
            z_(z),
            mavlink_stream_(mavlink_stream_)
{
}

void Waypoint::send_waypoint(uint32_t sysid, mavlink_message_t* msg, uint16_t seq, uint8_t current)
{
    //  Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
    // mavlink_msg_mission_item_send (  mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq,
    //                                  uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1,
    //                                  float param2, float param3, float param4, float x, float y, float z)
    mavlink_message_t _msg;
    mavlink_msg_mission_item_pack(sysid,
                                  mavlink_stream_.compid(),
                                  &_msg,
                                  msg->sysid,
                                  msg->compid,
                                  seq,
                                  frame_,
                                  command_,
                                  current,
                                  autocontinue_,
                                  param1_,
                                  param2_,
                                  param3_,
                                  param4_,
                                  x_,
                                  y_,
                                  z_);
    mavlink_stream_.send(&_msg);
}

void Mavlink_waypoint_handler::calculate_waypoint_local_structure(global_position_t origin, dubin_state_t* dubin_state)
{
    global_position_t waypoint_global;
    local_position_t waypoint_coor;
    global_position_t origin_relative_alt;

    for (uint8_t i = 0; i < 3; i++)
    {
        waypoint_coor.pos[i] = 0.0f;
    }
    waypoint_coor.origin = origin;
    waypoint_coor.heading = maths_deg_to_rad(param4_);

    switch (frame_)
    {
        case MAV_FRAME_GLOBAL:
            waypoint_global.latitude    = x_;
            waypoint_global.longitude   = y_;
            waypoint_global.altitude    = z_;
            waypoint_global.heading     = maths_deg_to_rad(param4_);
            waypoint_coor = coord_conventions_global_to_local_position(waypoint_global, origin);

            print_util_dbg_print("waypoint_global: lat (x1e7):");
            print_util_dbg_print_num(waypoint_global.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(waypoint_global.longitude * 10000000, 10);
            print_util_dbg_print(" alt (x1000):");
            print_util_dbg_print_num(waypoint_global.altitude * 1000, 10);
            print_util_dbg_print(" waypoint_coor: x (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[X] * 100, 10);
            print_util_dbg_print(", y (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Y] * 100, 10);
            print_util_dbg_print(", z (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Z] * 100, 10);
            print_util_dbg_print(" localOrigin lat (x1e7):");
            print_util_dbg_print_num(origin.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(origin.longitude * 10000000, 10);
            print_util_dbg_print(" alt (x1000):");
            print_util_dbg_print_num(origin.altitude * 1000, 10);
            print_util_dbg_print("\r\n");
            break;

        case MAV_FRAME_LOCAL_NED:
            waypoint_coor.pos[X] = x_;
            waypoint_coor.pos[Y] = y_;
            waypoint_coor.pos[Z] = z_;
            waypoint_coor.heading = maths_deg_to_rad(param4_);
            waypoint_coor.origin = coord_conventions_local_to_global_position(waypoint_coor);
            break;

        case MAV_FRAME_MISSION:
            // Problem here: rec is not defined here
            //mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
            break;
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            waypoint_global.latitude = x_;
            waypoint_global.longitude = y_;
            waypoint_global.altitude = z_;
            waypoint_global.heading     = maths_deg_to_rad(param4_);

            origin_relative_alt = origin;
            origin_relative_alt.altitude = 0.0f;
            waypoint_coor = coord_conventions_global_to_local_position(waypoint_global, origin_relative_alt);

            waypoint_coor.heading = maths_deg_to_rad(param4_);

            print_util_dbg_print("LocalOrigin: lat (x1e7):");
            print_util_dbg_print_num(origin_relative_alt.latitude * 10000000, 10);
            print_util_dbg_print(" long (x1e7):");
            print_util_dbg_print_num(origin_relative_alt.longitude * 10000000, 10);
            print_util_dbg_print(" global alt (x1000):");
            print_util_dbg_print_num(origin.altitude * 1000, 10);
            print_util_dbg_print(" waypoint_coor: x (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[X] * 100, 10);
            print_util_dbg_print(", y (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Y] * 100, 10);
            print_util_dbg_print(", z (x100):");
            print_util_dbg_print_num(waypoint_coor.pos[Z] * 100, 10);
            print_util_dbg_print("\r\n");

            break;
        case MAV_FRAME_LOCAL_ENU:
            // Problem here: rec is not defined here
            //mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
            break;

    }

    waypoint_ = waypoint_coor;
    // WARNING: Acceptance radius (param2) is used as the waypoint radius (should be param3) for a fixed-wing
    radius_ = current_waypoint->param2;
    loiter_time_ = current_waypoint->param1;

    *dubin_state_ = DUBIN_INIT;

    return wpt;
}
