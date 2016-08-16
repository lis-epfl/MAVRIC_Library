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
 * \file mavlink_message_handler.cpp
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module handles of all incoming MAVLink message by calling the
 * appropriate functions
 *
 ******************************************************************************/


 #include <cstdbool>

#include "communication/mavlink_message_handler.hpp"
#include "util/print_util.hpp"

extern "C"
{
#include "hal/piezo_speaker.h"
}



//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool Mavlink_message_handler::match_msg(msg_callback_t* msg_callback, mavlink_message_t* msg)
{
    bool match = false;

    uint8_t sysid = mavlink_stream_.sysid();

    if (msg->sysid != sysid)    // This message is not from this system
    {
        if (msg_callback->message_id == msg->msgid) // The message has the good ID
        {
            if (msg_callback->sysid_filter == MAV_SYS_ID_ALL || msg_callback->sysid_filter == msg->sysid)   // The message is from the good system
            {
                if (msg_callback->compid_filter == MAV_COMP_ID_ALL || msg_callback->compid_filter == msg->compid)   // The system is from the good component
                {
                    match = true;
                }
            }
        }
    }

    return match;
}


bool Mavlink_message_handler::match_cmd(cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd)
{
    bool match = false;

    uint8_t sysid = mavlink_stream_.sysid();

    if (msg->sysid != sysid)    // This message is not from this system
    {
        if (cmd_callback->command_id == cmd->command)       // The message has the good ID
        {
            if (cmd_callback->sysid_filter == MAV_SYS_ID_ALL || cmd_callback->sysid_filter == msg->sysid)   // The message is from the good system
            {
                if (cmd_callback->compid_filter == MAV_COMP_ID_ALL || cmd_callback->compid_filter == msg->compid)   // The system is from the good component
                {
                    if (cmd_callback->compid_target == MAV_COMP_ID_ALL || cmd_callback->compid_target == cmd->target_component)   // This system is the target of the command
                    {
                        match = true;
                    }
                }
            }
        }
    }

    return match;
}


void Mavlink_message_handler::sort_latest_cmd_callback()
{
    uint32_t j = 0;

    //as the list is already sorted, we just need to compare the latest element from the list with previous one,
    //until we find a command_id lower than the current one
    cmd_callback_t temp = cmd_callback_list()[cmd_callback_count_];
    j = cmd_callback_count_;
    while ((j > 0) && (cmd_callback_list()[j - 1].command_id > temp.command_id))
    {
        //swap them
        cmd_callback_list()[j] = cmd_callback_list()[j - 1];
        j = j - 1;
    }
    cmd_callback_list()[j] = temp;
}

void Mavlink_message_handler::sort_latest_msg_callback()
{
    uint32_t j = 0;

    //as the list is already sorted, we just need to compare the latest element from the list with previous one,
    //until we find a message_id lower than the current one
    msg_callback_t temp = msg_callback_list()[msg_callback_count_];
    j =  msg_callback_count_;
    while ((j > 0) && (msg_callback_list()[j - 1].message_id > temp.message_id))
    {
        //swap them
        msg_callback_list()[j] = msg_callback_list()[j - 1];
        j = j - 1;
    }
    msg_callback_list()[j] = temp;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_message_handler::Mavlink_message_handler(Mavlink_stream& mavlink_stream, const conf_t& config) :
    mavlink_stream_(mavlink_stream),
    debug_(config.debug),
    msg_callback_count_(0),
    cmd_callback_count_(0)
{}


bool Mavlink_message_handler::add_msg_callback(msg_callback_t* msg_callback)
{
    bool add_callback_success = true;

    if (msg_callback == NULL)
    {
        print_util_dbg_print("[MESSAGE HANDLER] Error: null pointer.\r\n");

        add_callback_success &= false;
    }
    else
    {
        if (msg_callback_count_ <  msg_callback_max_count())
        {
            msg_callback_t* new_callback = &msg_callback_list()[msg_callback_count_];

            new_callback->message_id    = msg_callback->message_id;
            new_callback->sysid_filter  = msg_callback->sysid_filter;
            new_callback->compid_filter = msg_callback->compid_filter;
            new_callback->function      = msg_callback->function;
            new_callback->module_struct = msg_callback->module_struct;

            sort_latest_msg_callback();

            msg_callback_count_ += 1;

            add_callback_success &= true;
        }
        else
        {
            print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");

            add_callback_success &= false;
        }
    }

    return add_callback_success;
}


bool Mavlink_message_handler::add_cmd_callback(cmd_callback_t* cmd_callback)
{
    bool add_callback_success = true;

    if (cmd_callback == NULL)
    {
        print_util_dbg_print("[MESSAGE HANDLER] Error: null pointer.\r\n");

        add_callback_success &= false;
    }
    else
    {
        if (cmd_callback_count_ <  cmd_callback_max_count())
        {
            cmd_callback_t* new_callback = &cmd_callback_list()[cmd_callback_count_];

            new_callback->command_id    = cmd_callback->command_id;
            new_callback->sysid_filter  = cmd_callback->sysid_filter;
            new_callback->compid_filter = cmd_callback->compid_filter;
            new_callback->compid_target = cmd_callback->compid_target;
            new_callback->function      = cmd_callback->function;
            new_callback->module_struct = cmd_callback->module_struct;

            sort_latest_cmd_callback();

            cmd_callback_count_ += 1;

            add_callback_success &= true;
        }
        else
        {
            print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");

            add_callback_success &= false;
        }
    }

    return add_callback_success;
}


void Mavlink_message_handler::msg_default_dbg(mavlink_message_t* msg)
{
    if ((msg->sysid == MAVLINK_BASE_STATION_ID)
            && (msg->msgid != MAVLINK_MSG_ID_MANUAL_CONTROL)
            && (msg->msgid != MAVLINK_MSG_ID_HEARTBEAT))
    {
        print_util_dbg_print("Received message with ID ");
        print_util_dbg_print_num(msg->msgid,10);
        print_util_dbg_print(" from system ");
        print_util_dbg_print_num(msg->sysid,10);
        print_util_dbg_print(" from component");
        print_util_dbg_print_num(msg->compid,10);
        print_util_dbg_print("\r\n");
    }
}


void Mavlink_message_handler::cmd_default_dbg(mavlink_command_long_t* cmd)
{
    print_util_dbg_print("Received command with ID");
    print_util_dbg_print_num(cmd->command,10);
    print_util_dbg_print(" with parameters: [");
    print_util_dbg_print_num(cmd->param1,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param2,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param3,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param4,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param5,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param6,10);
    print_util_dbg_print(", ");
    print_util_dbg_print_num(cmd->param7,10);
    print_util_dbg_print("] confirmation: ");
    print_util_dbg_print_num(cmd->confirmation,10);
    print_util_dbg_print("\r\n");
}


void Mavlink_message_handler::receive(Mavlink_stream::msg_received_t* rec)
{
    mavlink_message_t* msg = &rec->msg;

    if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
    {
        // The message is a command
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);

        //print packet command and parameters for debug
        if (debug_)
        {
            cmd_default_dbg(&cmd);
        }

        if (cmd.command < MAV_CMD_ENUM_END)
        {
            // The command has valid command ID
            if ((cmd.target_system == mavlink_stream_.sysid()) || (cmd.target_system == MAV_SYS_ID_ALL))
            {
                mav_result_t result = MAV_RESULT_UNSUPPORTED;

                // The command is for this system
                for (uint32_t i = 0; i < cmd_callback_count_; ++i)
                {
                    if (match_cmd(&cmd_callback_list()[i], msg, &cmd))
                    {
                        cmd_callback_func_t function             = cmd_callback_list()[i].function;
                        handling_module_struct_t module_struct   = cmd_callback_list()[i].module_struct;

                        // Call appropriate function callback
                        result = function(module_struct, &cmd);

                        if (((i + 1) != cmd_callback_count_) && ((cmd_callback_list()[i + 1].command_id) > cmd.command))
                        {
                            //as callback_list is sorted by command_id, no need to go further in the list
                            break;
                        }
                    }
                }
                // Send acknowledgment message
                mavlink_message_t msg;
                mavlink_msg_command_ack_pack(mavlink_stream_.sysid(),
                                             mavlink_stream_.compid(),
                                             &msg,
                                             cmd.command,
                                             result);
                mavlink_stream_.send(&msg);
            }
        }
    }
    else if (msg->msgid < MAV_MSG_ENUM_END)
    {
        if (debug_)
        {
            msg_default_dbg(msg);
        }

        // The message has a valid message ID, and is not a command
        for (uint32_t i = 0; i < msg_callback_count_; ++i)
        {
            if (match_msg(&msg_callback_list()[i], msg))
            {
                Mavlink_message_handler::msg_callback_func_t function        = msg_callback_list()[i].function;
                handling_module_struct_t        module_struct   = msg_callback_list()[i].module_struct;
                // Call appropriate function callback
                function(module_struct, mavlink_stream_.sysid(), msg);

                if (((i + 1) != msg_callback_count_) && ((msg_callback_list()[i + 1].message_id) > msg->msgid))
                {
                    //as callback_list is sorted by message_id, no need to go further in the list
                    break;
                }
            }
        }
    }
}
