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
 * \file mavlink_waypoint_handler.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * \author Matthew Douglas
 *
 * \brief The MAVLink waypoint handler
 *
 ******************************************************************************/


#include "communication/mavlink_waypoint_handler.hpp"
#include "mission/mission_handler.hpp"

#include "hal/common/time_keeper.hpp"
#include "util/constants.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "util/maths.h"
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

mav_result_t Mavlink_waypoint_handler::set_home(Mavlink_waypoint_handler* waypoint_handler, mavlink_command_long_t* packet)
{
    if (packet->param1 == 0) // Use indicated location
    {
        waypoint_handler->home_waypoint_ = Waypoint(MAV_FRAME_LOCAL_NED,
                                                    MAV_CMD_NAV_WAYPOINT,
                                                    0,
                                                    packet->param1,
                                                    packet->param2,
                                                    packet->param3,
                                                    packet->param4,
                                                    packet->param5,
                                                    packet->param6,
                                                    packet->param7);
        print_util_dbg_print("New home location set to (");
        print_util_dbg_putfloat(packet->param5, 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(packet->param6, 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(packet->param7, 3);
        print_util_dbg_print(")\r\n");
    }
    else if (packet->param1 == 1) // Use current position
    {
        waypoint_handler->home_waypoint_ = Waypoint(MAV_FRAME_LOCAL_NED,
                                                    MAV_CMD_NAV_WAYPOINT,
                                                    0,
                                                    packet->param1,
                                                    packet->param2,
                                                    packet->param3,
                                                    packet->param4,
                                                    waypoint_handler->ins_.position_lf()[X],
                                                    waypoint_handler->ins_.position_lf()[Y],
                                                    waypoint_handler->ins_.position_lf()[Z]);
        print_util_dbg_print("New home location set to current position: (");
        print_util_dbg_putfloat(waypoint_handler->ins_.position_lf()[X], 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(waypoint_handler->ins_.position_lf()[Y], 3);
        print_util_dbg_print(", ");
        print_util_dbg_putfloat(waypoint_handler->ins_.position_lf()[Z], 3);
        print_util_dbg_print(")\r\n");
    }
    else
    {
        return MAV_RESULT_DENIED;
    }

    return MAV_RESULT_ACCEPTED;
}

void Mavlink_waypoint_handler::request_list_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_request_list_t packet;

    mavlink_msg_mission_request_list_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        mavlink_message_t _msg;
        mavlink_msg_mission_count_pack(sysid,
                                       waypoint_handler->mavlink_stream_.compid(),
                                       &_msg,
                                       msg->sysid,
                                       msg->compid,
                                       waypoint_handler->waypoint_count_);
        waypoint_handler->mavlink_stream_.send(&_msg);

        if (waypoint_handler->waypoint_count_ != 0)
        {
            waypoint_handler->is_sending_waypoint_ = true;
            waypoint_handler->is_receiving_waypoint_ = false;
            waypoint_handler->start_timeout_ = time_keeper_get_ms();
        }

        waypoint_handler->sending_waypoint_num_ = 0;
        print_util_dbg_print("Will send ");
        print_util_dbg_print_num(waypoint_handler->waypoint_count_, 10);
        print_util_dbg_print(" waypoints\r\n");
    }
}

void Mavlink_waypoint_handler::mission_request_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    if (waypoint_handler->is_sending_waypoint_)
    {
        mavlink_mission_request_t packet;

        mavlink_msg_mission_request_decode(msg, &packet);

        print_util_dbg_print("Asking for waypoint number ");
        print_util_dbg_print_num(packet.seq, 10);
        print_util_dbg_print("\r\n");

        // Check if this message is for this system and subsystem
        if (((uint8_t)packet.target_system == (uint8_t)sysid)
                && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER
                || (uint8_t)packet.target_component == 50)) // target_component = 50 is sent by dronekit
        {
            waypoint_handler->sending_waypoint_num_ = packet.seq;
            if (waypoint_handler->sending_waypoint_num_ < waypoint_handler->waypoint_count_)
            {
                uint8_t is_current = 0;
                if (    waypoint_handler->sending_waypoint_num_ == waypoint_handler->current_waypoint_index_ && // This is the current waypoint
                        !waypoint_handler->navigation_.waiting_at_waypoint())                                   // And we are en route
                {
                    is_current = 1;
                }

                waypoint_handler->waypoint_list_[waypoint_handler->sending_waypoint_num_].send(waypoint_handler->mavlink_stream_, sysid, msg, packet.seq, is_current);

                print_util_dbg_print("Sending waypoint ");
                print_util_dbg_print_num(waypoint_handler->sending_waypoint_num_, 10);
                print_util_dbg_print("\r\n");

                waypoint_handler->start_timeout_ = time_keeper_get_ms();
            }
        }
    }
}

void Mavlink_waypoint_handler::mission_ack_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_ack_t packet;

    mavlink_msg_mission_ack_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        waypoint_handler->is_sending_waypoint_ = false;
        waypoint_handler->sending_waypoint_num_ = 0;
        print_util_dbg_print("Acknowledgment received, end of waypoint sending.\r\n");
    }
}

void Mavlink_waypoint_handler::mission_count_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_count_t packet;

    mavlink_msg_mission_count_decode(msg, &packet);

    print_util_dbg_print("Count:");
    print_util_dbg_print_num(packet.count, 10);
    print_util_dbg_print("\r\n");

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        if (waypoint_handler->is_receiving_waypoint_ == false)
        {
            // Erase current waypoint list
            waypoint_handler->waypoint_count_ = 0;

            // Save the number of waypoints GCS is about to send
            if (packet.count > MAX_WAYPOINTS)
            {
                packet.count = MAX_WAYPOINTS;
            }
            waypoint_handler->requested_waypoint_count_ = packet.count;

            print_util_dbg_print("Receiving ");
            print_util_dbg_print_num(packet.count, 10);
            print_util_dbg_print(" new waypoints. ");
            print_util_dbg_print("New total number of waypoints:");
            print_util_dbg_print_num(packet.count + waypoint_handler->waypoint_count_, 10);
            print_util_dbg_print("\r\n");

            // we are about to receive a list of waypoints, so we stop sending
            waypoint_handler->is_receiving_waypoint_   = true;
            waypoint_handler->is_sending_waypoint_     = false;

            // the next waypoint to request is number 0, this count will be increment until all
            // waypoints are received (ie. waypoint_request_number_ == requested_waypoint_count_ - 1)
            waypoint_handler->waypoint_request_number_ = 0;


            waypoint_handler->start_timeout_ = time_keeper_get_ms();
        }

        mavlink_message_t _msg;
        mavlink_msg_mission_request_pack(sysid,
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         waypoint_handler->waypoint_request_number_);
        waypoint_handler->mavlink_stream_.send(&_msg);

        print_util_dbg_print("Asking for waypoint ");
        print_util_dbg_print_num(waypoint_handler->waypoint_request_number_, 10);
        print_util_dbg_print("\r\n");
    }

}

void Mavlink_waypoint_handler::mission_item_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_item_t packet;

    mavlink_msg_mission_item_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        waypoint_handler->start_timeout_ = time_keeper_get_ms();

        Waypoint new_waypoint(packet);

        print_util_dbg_print("New waypoint received. requested num :");
        print_util_dbg_print_num(waypoint_handler->waypoint_request_number_, 10);
        print_util_dbg_print(" receiving num :");
        print_util_dbg_print_num(packet.seq, 10);
        //print_util_dbg_print(" is it receiving :");
        //print_util_dbg_print_num(waypoint_handler->is_receiving_waypoint_,10); // boolean value
        print_util_dbg_print("\r\n");

        //current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
        if (packet.current == 2)
        {
            // verify we received the command;
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(sysid,
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         MAV_MISSION_UNSUPPORTED);
            waypoint_handler->mavlink_stream_.send(&_msg);
        }
        else if (packet.current == 3)
        {
            //current = 3 is a flag to tell us this is a alt change only

            // verify we received the command
            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         MAV_MISSION_UNSUPPORTED);
            waypoint_handler->mavlink_stream_.send(&_msg);
        }
        else
        {
            // Check if receiving waypoints
            if (waypoint_handler->is_receiving_waypoint_)
            {
                // check if this is the requested waypoint
                if (packet.seq == waypoint_handler->waypoint_request_number_)
                {
                    if (waypoint_handler->mission_handler_registry_.get_mission_handler(new_waypoint) != NULL)
                    {
                        print_util_dbg_print("Receiving good waypoint, number ");
                        print_util_dbg_print_num(waypoint_handler->waypoint_request_number_, 10);
                        print_util_dbg_print(" of ");
                        print_util_dbg_print_num(waypoint_handler->requested_waypoint_count_, 10);
                        print_util_dbg_print("\r\n");

                        waypoint_handler->waypoint_list_[waypoint_handler->waypoint_count_] = new_waypoint;
                        waypoint_handler->waypoint_request_number_++;
                        waypoint_handler->waypoint_count_++;

                        if (waypoint_handler->waypoint_request_number_ == waypoint_handler->requested_waypoint_count_)
                        {
                            MAV_MISSION_RESULT type = MAV_MISSION_ACCEPTED;

                            mavlink_message_t _msg;
                            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                                         waypoint_handler->mavlink_stream_.compid(),
                                                         &_msg,
                                                         msg->sysid,
                                                         msg->compid, type);
                            waypoint_handler->mavlink_stream_.send(&_msg);

                            print_util_dbg_print("flight plan received!\n");
                            waypoint_handler->is_receiving_waypoint_ = false;

                            // TODO Should this auto start moving towards the point
                        }
                        else
                        {
                            mavlink_message_t _msg;
                            mavlink_msg_mission_request_pack(waypoint_handler->mavlink_stream_.sysid(),
                                                             waypoint_handler->mavlink_stream_.compid(),
                                                             &_msg,
                                                             msg->sysid,
                                                             msg->compid,
                                                             waypoint_handler->waypoint_request_number_);
                            waypoint_handler->mavlink_stream_.send(&_msg);

                            print_util_dbg_print("Asking for waypoint ");
                            print_util_dbg_print_num(waypoint_handler->waypoint_request_number_, 10);
                            print_util_dbg_print("\n");
                        }
                    }
                    else    // No mission handler registered in registry
                    {
                        print_util_dbg_print("Waypoint not registered in registry\r\n");
                        MAV_MISSION_RESULT type = MAV_MISSION_UNSUPPORTED;

                        mavlink_message_t _msg;
                        mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                                     waypoint_handler->mavlink_stream_.compid(),
                                                     &_msg,
                                                     msg->sysid,
                                                     msg->compid,
                                                     type);
                        waypoint_handler->mavlink_stream_.send(&_msg);
                    }

                } //end of if (packet.seq == waypoint_handler->waypoint_request_number_)
                else
                {
                    MAV_MISSION_RESULT type = MAV_MISSION_INVALID_SEQUENCE;

                    mavlink_message_t _msg;
                    mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                                 waypoint_handler->mavlink_stream_.compid(),
                                                 &_msg,
                                                 msg->sysid,
                                                 msg->compid,
                                                 type);
                    waypoint_handler->mavlink_stream_.send(&_msg);
                }
            } //end of if (waypoint_handler->is_receiving_waypoint_)
            else
            {
                MAV_MISSION_RESULT type = MAV_MISSION_ERROR;
                print_util_dbg_print("Not ready to receive waypoints right now!\r\n");

                mavlink_message_t _msg;
                mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                             waypoint_handler->mavlink_stream_.compid(),
                                             &_msg,
                                             msg->sysid,
                                             msg->compid,
                                             type);
                waypoint_handler->mavlink_stream_.send(&_msg);
            } //end of else of if (waypoint_handler->is_receiving_waypoint_)
        } //end of else (packet.current != 2 && !=3 )
    } //end of if this message is for this system and subsystem
}

void Mavlink_waypoint_handler::mission_clear_all_callback(Mavlink_waypoint_handler* waypoint_handler, uint32_t sysid, mavlink_message_t* msg)
{
    mavlink_mission_clear_all_t packet;

    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // Check if this message is for this system and subsystem
    if (((uint8_t)packet.target_system == (uint8_t)sysid)
            && ((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER))
    {
        if (waypoint_handler->waypoint_count_ > 0)
        {
            waypoint_handler->waypoint_count_ = 0;
            //Mission_handler::reset_hold_waypoint(); TODO

            mavlink_message_t _msg;
            mavlink_msg_mission_ack_pack(waypoint_handler->mavlink_stream_.sysid(),
                                         waypoint_handler->mavlink_stream_.compid(),
                                         &_msg,
                                         msg->sysid,
                                         msg->compid,
                                         MAV_CMD_ACK_OK);
            waypoint_handler->mavlink_stream_.send(&_msg);

            print_util_dbg_print("Cleared Waypoint list.\r\n");
        }
    }
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_waypoint_handler::Mavlink_waypoint_handler( const INS& ins,
                                                    const Navigation& navigation,
                                                    Mavlink_message_handler& message_handler,
                                                    const Mavlink_stream& mavlink_stream,
                                                    Mission_handler_registry& mission_handler_registry,
                                                    conf_t config):
    waypoint_count_(0),
    current_waypoint_index_(0),
    mavlink_stream_(mavlink_stream),
    ins_(ins),
    navigation_(navigation),
    message_handler_(message_handler),
    mission_handler_registry_(mission_handler_registry),
    is_sending_waypoint_(false),
    is_receiving_waypoint_(false),
    sending_waypoint_num_(0),
    waypoint_request_number_(0),
    requested_waypoint_count_(0),
    start_timeout_(time_keeper_get_ms()),
    timeout_max_waypoint_(10000),
    config_(config)
{
    for (int i = 0; i < MAX_WAYPOINTS; i++)
    {
        waypoint_list_[i] = Waypoint();
    }

    home_waypoint_ = Waypoint(  MAV_FRAME_LOCAL_NED,
                                MAV_CMD_NAV_WAYPOINT,
                                0,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                0.0f,
                                navigation_.takeoff_altitude);
}

bool Mavlink_waypoint_handler::init()
{
    bool init_success = true;

    // Add callbacks for waypoint handler messages requests
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_MISSION_ITEM; // 39
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &mission_item_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_REQUEST; // 40
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &mission_request_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_REQUEST_LIST; // 43
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &request_list_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_COUNT; // 44
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &mission_count_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    callback.message_id     = MAVLINK_MSG_ID_MISSION_CLEAR_ALL; // 45
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &mission_clear_all_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;

    callback.message_id     = MAVLINK_MSG_ID_MISSION_ACK; // 47
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t)      &mission_ack_callback;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_msg_callback(&callback);

    // Add callbacks for waypoint handler commands requests
    Mavlink_message_handler::cmd_callback_t callbackcmd;

    callbackcmd.command_id = MAV_CMD_DO_SET_HOME; // 179
    callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
    callbackcmd.compid_filter = MAV_COMP_ID_ALL;
    callbackcmd.compid_target = MAV_COMP_ID_ALL; // 0
    callbackcmd.function = (Mavlink_message_handler::cmd_callback_func_t)           &set_home;
    callbackcmd.module_struct = (Mavlink_message_handler::handling_module_struct_t) this;
    init_success &= message_handler_.add_cmd_callback(&callbackcmd);

    init_homing_waypoint();

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK_WAYPOINT_HANDLER] constructor: ERROR\r\n");
    }

    return init_success;
}

void Mavlink_waypoint_handler::init_homing_waypoint()
{
    /*
    Constructor order:

    uint8_t frame,
    uint16_t command,
    uint8_t autocontinue,
    float param1,   Minimum pitch (if airspeed sensor present), desired pitch without sensor [rad]
    float param2,   Empty
    float param3,   Takeoff ascend rate [ms^-1]
    float param4,   Yaw angle [rad] (if magnetometer or another yaw estimation source present), ignored without one of these
    float param5,   Y-axis position [m] (I dont know why they made it this order)
    float param6,   X-axis position [m] (I dont know why they made it this order)
    float param7    Z-axis position [m]
    */
    Waypoint waypoint(  MAV_FRAME_LOCAL_NED,
                        MAV_CMD_NAV_TAKEOFF,
                        0,
                        0.0f,
                        0.0f,
                        0.0f,
                        0.0f,
                        0.0f,
                        0.0f,
                        navigation_.takeoff_altitude);

    waypoint_count_ = 1;
    set_current_waypoint_index(0);

    waypoint_list_[0] = waypoint;
}

const Waypoint& Mavlink_waypoint_handler::current_waypoint()
{
    // If there are no waypoints set, go to home
    if (waypoint_count_ == 0)
    {
        return home_waypoint();
    }

    // If it is a good index
    if (current_waypoint_index_ >= 0 && current_waypoint_index_ < waypoint_count_)
    {
        return waypoint_list_[current_waypoint_index_];
    }
    else
    {
        // For now, set to home
        return home_waypoint();
    }
}

const Waypoint& Mavlink_waypoint_handler::next_waypoint()
{
    // If there are no waypoints set, go to home
    if (waypoint_count_ == 0)
    {
        return home_waypoint();
    }

    // If it is a good index
    if (current_waypoint_index_ >= 0 && current_waypoint_index_ < waypoint_count_)
    {
        // Check if the next waypoint exists
        if ((current_waypoint_index_ + 1) != waypoint_count_)
        {
            return waypoint_list_[current_waypoint_index_ + 1];
        }
        else // No next waypoint, set to first
        {
            return waypoint_list_[0];
        }
    }
    else
    {
        // For now, set to home
        return home_waypoint();
    }
}

const Waypoint& Mavlink_waypoint_handler::waypoint_from_index(int i)
{
    // If there are no waypoints set, go to home
    if (waypoint_count_ == 0)
    {
        return home_waypoint();
    }

    if (i >= 0 && i < waypoint_count_)
    {
        return waypoint_list_[i];
    }
    else
    {
        return home_waypoint();
    }
}

const Waypoint& Mavlink_waypoint_handler::home_waypoint()
{
    return home_waypoint_;
}

void Mavlink_waypoint_handler::advance_to_next_waypoint()
{
    // If the current waypoint index is the last waypoint, go to first waypoint
    if (current_waypoint_index_ == (waypoint_count_-1))
    {
        set_current_waypoint_index(0);
    }
    else // Update current in both waypoints
    {
        set_current_waypoint_index(current_waypoint_index_+1);
    }
}

void Mavlink_waypoint_handler::control_time_out_waypoint_msg()
{
    if (is_sending_waypoint_ || is_receiving_waypoint_)
    {
        uint32_t tnow = time_keeper_get_ms();

        if ((tnow - start_timeout_) > timeout_max_waypoint_)
        {
            start_timeout_ = tnow;
            if (is_sending_waypoint_)
            {
                is_sending_waypoint_ = false;
                print_util_dbg_print("Sending waypoint timeout\r\n");
            }
            if (is_receiving_waypoint_)
            {
                is_receiving_waypoint_ = false;

                print_util_dbg_print("Receiving waypoint timeout\r\n");
                waypoint_count_ = 0;
            }
        }
    }
}

uint16_t Mavlink_waypoint_handler::current_waypoint_index() const
{
    return current_waypoint_index_;
}

bool Mavlink_waypoint_handler::set_current_waypoint_index(int index)
{
    if (index >= 0 && index < waypoint_count_)
    {
        current_waypoint_index_ = index;

        print_util_dbg_print("Set current waypoint to number");
        print_util_dbg_print_num(index, 10);
        print_util_dbg_print("\r\n");

        return true;
    }
    else
    {
        return false;
    }
}
