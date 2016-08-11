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


#include "control/waypoint.hpp"

#include "sensing/ins.hpp"

#include <cstdlib>
#include "hal/common/time_keeper.hpp"

extern "C"
{
#include "util/print_util.h"
#include "util/maths.h"
#include "util/constants.hpp"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------



//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Waypoint::Waypoint() :
            frame_(MAV_FRAME_LOCAL_NED),
            command_(MAV_CMD_NAV_WAYPOINT),
            autocontinue_(0),
            param1_(0.0f),
            param2_(0.0f),
            param3_(0.0f),
            param4_(0.0f),
            param5_(0.0f),
            param6_(0.0f),
            param7_(0.0f)
{
}

Waypoint::Waypoint(mavlink_mission_item_t& packet)
{
    command_ = packet.command;
    autocontinue_ = packet.autocontinue;
    frame_ = packet.frame;
    param1_ = packet.param1;
    param2_ = packet.param2;
    param3_ = packet.param3;
    param4_ = packet.param4;
    param5_ = packet.x; // longitude
    param6_ = packet.y; // latitude
    param7_ = packet.z; // altitude
}

Waypoint::Waypoint( uint8_t frame,
                    uint16_t command,
                    uint8_t autocontinue,
                    float param1,
                    float param2,
                    float param3,
                    float param4,
                    float param5,
                    float param6,
                    float param7) :
            frame_(frame),
            command_(command),
            autocontinue_(autocontinue),
            param1_(param1),
            param2_(param2),
            param3_(param3),
            param4_(param4),
            param5_(param5),
            param6_(param6),
            param7_(param7)
{
}

void Waypoint::send(const Mavlink_stream& mavlink_stream, uint32_t sysid, mavlink_message_t* msg, uint16_t seq, uint8_t current)
{
    //  Prototype of the function "mavlink_msg_mission_item_send" found in mavlink_msg_mission_item.h :
    // mavlink_msg_mission_item_send (  mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, uint16_t seq,
    //                                  uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1,
    //                                  float param2, float param3, float param4, float x, float y, float z)
    mavlink_message_t _msg;
    mavlink_msg_mission_item_pack(sysid,
                                  mavlink_stream.compid(),
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
                                  param5_,
                                  param6_,
                                  param7_);
    mavlink_stream.send(&_msg);
}

uint8_t Waypoint::frame() const
{
    return frame_;
}

uint16_t Waypoint::command() const
{
    return command_;
}

uint8_t Waypoint::autocontinue() const
{
    return autocontinue_;
}

float Waypoint::param1() const
{
    return param1_;
}

float Waypoint::param2() const
{
    return param2_;
}

float Waypoint::param3() const
{
    return param3_;
}

float Waypoint::param4() const
{
    return param4_;
}

float Waypoint::param5() const
{
    return param5_;
}

float Waypoint::param6() const
{
    return param6_;
}

float Waypoint::param7() const
{
    return param7_;
}

float Waypoint::heading() const
{
    switch (command_)
    {
    case MAV_CMD_NAV_WAYPOINT:      // 16
    case MAV_CMD_NAV_LOITER_UNLIM:  // 17
    case MAV_CMD_NAV_LOITER_TURNS:  // 18
    case MAV_CMD_NAV_LOITER_TIME:   // 19
    case MAV_CMD_NAV_LAND:          // 21
    case MAV_CMD_NAV_TAKEOFF:       // 22
    case MAV_CMD_OVERRIDE_GOTO:     // 252
        return param4_;
        
    case MAV_CMD_NAV_LOITER_TO_ALT: // 31
    default:
        return 0.0f;
    }
}

float Waypoint::radius() const
{
    switch (command_)
    {
    case MAV_CMD_NAV_WAYPOINT:      // 16
    case MAV_CMD_NAV_LOITER_TO_ALT: // 31
        return param2_;

    case MAV_CMD_NAV_LOITER_UNLIM:  // 17
    case MAV_CMD_NAV_LOITER_TURNS:  // 18
    case MAV_CMD_NAV_LOITER_TIME:   // 19
        return param3_;

    case MAV_CMD_NAV_LAND:          // 21
    case MAV_CMD_NAV_TAKEOFF:       // 22
    case MAV_CMD_OVERRIDE_GOTO:     // 252
    default:
        return 0.0f;
    }
}

local_position_t Waypoint::local_pos() const
{
    global_position_t waypoint_global;
    local_position_t waypoint_local;

    switch (frame_)
    {
        case MAV_FRAME_GLOBAL_INT:
            // Convert from int to degrees
            waypoint_global.latitude    = param5_ / 10000000.0f;
            waypoint_global.longitude   = param6_ / 10000000.0f;
            waypoint_global.altitude    = param7_;
            coord_conventions_global_to_local_position(waypoint_global, INS::origin(), waypoint_local);
            break;

        case MAV_FRAME_GLOBAL:
            waypoint_global.latitude    = param5_;
            waypoint_global.longitude   = param6_;
            waypoint_global.altitude    = param7_;
            coord_conventions_global_to_local_position(waypoint_global, INS::origin(), waypoint_local);
            break;

        case MAV_FRAME_LOCAL_ENU:
            waypoint_local[X]   =  param6_;
            waypoint_local[Y]   =  param5_;
            waypoint_local[Z]   = -param7_;
            break;

        case MAV_FRAME_LOCAL_NED:
            waypoint_local[X]   = param5_;
            waypoint_local[Y]   = param6_;
            waypoint_local[Z]   = param7_;
            break;

/*          THIS WOULD NEED TO BE CALCULATED AT THE INSTANT THAT IT IS NEEDED
        case MAV_FRAME_BODY_NED:
            waypoint_local.pos[X]   = x + position_estimation_.local_position.pos[X];
            waypoint_local.pos[Y]   = y + position_estimation_.local_position.pos[Y];
            waypoint_local.pos[Z]   = z + position_estimation_.local_position.pos[Z];
            waypoint_local.heading  = maths_deg_to_rad(heading);
            waypoint_local.origin   = origin;
            waypoint_global         = coord_conventions_local_to_global_position(waypoint_local);
            break;
*/
        case MAV_FRAME_MISSION:
            // Problem here: rec is not defined here
            //mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
            break;

        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            // Convert from int to degrees
            waypoint_global.latitude    = param5_ / 10000000.0f;
            waypoint_global.longitude   = param6_ / 10000000.0f;
            waypoint_global.altitude    = param7_ + INS::origin().altitude;
            coord_conventions_global_to_local_position(waypoint_global, INS::origin(), waypoint_local);
            break;

        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            waypoint_global.latitude    = param5_ / 10000000.0f;
            waypoint_global.longitude   = param6_ / 10000000.0f;
            waypoint_global.altitude    = param7_ + INS::origin().altitude;
            coord_conventions_global_to_local_position(waypoint_global, INS::origin(), waypoint_local);
            break;
    }

    return waypoint_local;
}
