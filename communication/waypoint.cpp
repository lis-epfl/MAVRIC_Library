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

global_position_t Waypoint::get_global_position(uint8_t frame, double x, double y, double z, global_position_t origin)
{
    global_position_t waypoint_global;
    local_position_t waypoint_local;

    switch (frame)
    {
        case MAV_FRAME_GLOBAL_INT:
            // Convert from int to degrees
            x = x / 10000000.0f;
            y = x / 10000000.0f;
            // NO BREAK, NEED TO CONVERT TO GLOBAL
        case MAV_FRAME_GLOBAL:
            waypoint_global.latitude    = x;
            waypoint_global.longitude   = y;
            waypoint_global.altitude    = z;
            break;

        case MAV_FRAME_LOCAL_ENU:
        {
            // convert XYZ to NED
            double e = x;
            double n = y;
            double u = z;
            x = n;
            y = e;
            z = -u;
        }
            // NO BREAK, NEED TO CONVERT FROM NED TO GLOBAL
        case MAV_FRAME_LOCAL_NED:
            waypoint_local[X]   = x;
            waypoint_local[Y]   = y;
            waypoint_local[Z]   = z;
            coord_conventions_local_to_global_position(waypoint_local, origin, waypoint_global);
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
            x = x / 10000000.0f;
            y = x / 10000000.0f;
            // NO BREAK, NEED TO CONVERT FROM GLOBAL_REL_ALT TO GLOBAL
        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            waypoint_global.latitude    = x;
            waypoint_global.longitude   = y;
            waypoint_global.altitude    = z + origin.altitude;
            break;
    }

    return waypoint_global;
}

void Waypoint::get_waypoint_parameters(double& x, double& y, double& z, uint8_t frame) const
{
    switch (frame)
    {
        case MAV_FRAME_GLOBAL_INT:
            // Convert from int to degrees
            x = wpt_position_.longitude * 10000000.0f;
            y = wpt_position_.latitude * 10000000.0f;
            z = wpt_position_.altitude;
            break;

        case MAV_FRAME_GLOBAL:
            x = wpt_position_.longitude;
            y = wpt_position_.latitude;
            z = wpt_position_.altitude;
            break;

        case MAV_FRAME_LOCAL_ENU:
        {
            local_position_t pos;
            coord_conventions_global_to_local_position(wpt_position_, INS::origin(), pos);
            x =  pos[1];
            y =  pos[0];
            z = -pos[2];
        }
            break;

        case MAV_FRAME_LOCAL_NED:
        {
            local_position_t pos;
            coord_conventions_global_to_local_position(wpt_position_, INS::origin(), pos);
            x = pos[0];
            y = pos[1];
            z = pos[2];
        }
            break;

        case MAV_FRAME_MISSION:
            // Problem here: rec is not defined here
            //mavlink_msg_mission_ack_send(MAVLINK_COMM_0,rec->msg.sysid,rec->msg.compid,MAV_CMD_ACK_ERR_NOT_SUPPORTED);
            break;

        case MAV_FRAME_GLOBAL_TERRAIN_ALT_INT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT_INT:
            // Convert from int to degrees
            x = wpt_position_.longitude * 10000000.0f;
            y = wpt_position_.latitude * 10000000.0f;
            z = wpt_position_.altitude - INS::origin().altitude;
            break;

        case MAV_FRAME_GLOBAL_TERRAIN_ALT:
        case MAV_FRAME_GLOBAL_RELATIVE_ALT:
            x = wpt_position_.longitude;
            y = wpt_position_.latitude;
            z = wpt_position_.altitude - INS::origin().altitude;
            break;
    }
}

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
            radius_(0.0f),
            loiter_time_(0.0f)
{
}

Waypoint::Waypoint(mavlink_mission_item_t& packet)
{
    command_ = packet.command;

    double x = packet.x; // longitude
    double y = packet.y; // latitude
    double z = packet.z; // altitude

    autocontinue_ = packet.autocontinue;
    frame_ = packet.frame;

    param1_ = packet.param1;
    param2_ = packet.param2;
    param3_ = packet.param3;
    param4_ = packet.param4;

    wpt_position_ = get_global_position(frame_, x, y, z, INS::origin());
    radius_ = param2_;
    loiter_time_ = param1_;
    // dubin_ = ???;
}

Waypoint::Waypoint( uint8_t frame,
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
            radius_(param2),
            loiter_time_(param1)
{
    wpt_position_ = get_global_position(frame, x, y, z, INS::origin());
    // dubin_ = ???;
}

Waypoint::Waypoint( uint8_t frame,
                    uint16_t command,
                    uint8_t autocontinue,
                    float param1,
                    float param2,
                    float param3,
                    float param4,
                    float x,
                    float y,
                    float z,
                    float radius,
                    float loiter_time,
                    dubin_t dubin) :
            frame_(frame),
            command_(command),
            autocontinue_(autocontinue),
            param1_(param1),
            param2_(param2),
            param3_(param3),
            param4_(param4),
            radius_(radius),
            loiter_time_(loiter_time),
            dubin_(dubin)
{
    wpt_position_ = get_global_position(frame, x, y, z, INS::origin());
}

void Waypoint::send(const Mavlink_stream& mavlink_stream, uint32_t sysid, mavlink_message_t* msg, uint16_t seq, uint8_t current)
{
    // These are outputs
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    get_waypoint_parameters(x, y, z, frame_);

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
                                  x,
                                  y,
                                  z);
    mavlink_stream.send(&_msg);
}

uint8_t Waypoint::frame() const
{
    return frame_;
}

void Waypoint::set_frame(uint8_t frame)
{
    frame_ = frame;
}

uint16_t Waypoint::command() const
{
    return command_;
}

void Waypoint::set_command(uint16_t command)
{
    command_ = command;
}

uint8_t Waypoint::autocontinue() const
{
    return autocontinue_;
}

float Waypoint::param1() const
{
    return param1_;
}

void Waypoint::set_param1(float param1)
{
    param1_ = param1;
}

float Waypoint::param2() const
{
    return param2_;
}

void Waypoint::set_param2(float param2)
{
    param2_ = param2;
}

float Waypoint::param3() const
{
    return param3_;
}

void Waypoint::set_param3(float param3)
{
    param3_ = param3;
}

float Waypoint::param4() const
{
    return param4_;
}

void Waypoint::set_param4(float param4)
{
    param4_ = param4;
}

float Waypoint::heading() const
{
    return param4_;
}

void Waypoint::set_heading(float heading)
{
    param4_ = heading;
}

local_position_t Waypoint::local_pos() const
{
    local_position_t pos;

    // These are outputs
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    get_waypoint_parameters(x, y, z, MAV_FRAME_LOCAL_NED);


    pos[X] = x;
    pos[Y] = y;
    pos[Z] = z;

    return pos;
}

void Waypoint::set_local_pos(local_position_t local_pos)
{
    wpt_position_ = get_global_position(MAV_FRAME_LOCAL_NED, local_pos[X], local_pos[Y], local_pos[Z], INS::origin());
}

void Waypoint::set_local_pos(local_position_t local_pos, float heading)
{
    set_heading(heading);
    set_local_pos(local_pos);
}

float Waypoint::radius() const
{
    return radius_;
}

void Waypoint::set_radius(float radius)
{
    radius_ = radius;
}

float Waypoint::loiter_time() const
{
    return loiter_time_;
}

void Waypoint::set_loiter_time(float loiter_time)
{
    loiter_time_ = loiter_time;
}

dubin_t& Waypoint::dubin()
{
    return dubin_;
}
