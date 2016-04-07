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
 * \file mavlink_communication.c
 *
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *
 * \brief This module takes care of sending periodic telemetric messages and
 * handling incoming messages
 *
 ******************************************************************************/


#include "communication/mavlink_communication.hpp"
#include "util/print_util.h"
extern "C"
{
#include "hal/common/time_keeper.hpp"
#include <stdlib.h>
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------
/**
 * \brief   Toggle mavlink telemetry stream
 *
 * \param scheduler     scheduler of the MAV
 * \param sysid         MAV sysid
 * \param msg           pointer to the stream you want to toggle
 */
static void mavlink_communication_toggle_telemetry_stream(Scheduler* scheduler, uint32_t sysid, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void mavlink_communication_toggle_telemetry_stream(Scheduler* scheduler, uint32_t sysid, mavlink_message_t* msg)
{
    // Decode message
    mavlink_request_data_stream_t request;
    mavlink_msg_request_data_stream_decode(msg, &request);

    if (((request.target_system == sysid) || (request.target_system == MAV_SYS_ID_ALL)))
        //&&(request.target_component == 0))
    {
        if (scheduler->is_debug())
        {
            print_util_dbg_print("stream request:");
            print_util_dbg_print_num(request.target_component,10);
            print_util_dbg_print(" stream=");
            print_util_dbg_print_num(request.req_stream_id,10);
            print_util_dbg_print(" start_stop=");
            print_util_dbg_print_num(request.start_stop,10);
            print_util_dbg_print(" rate=");
            print_util_dbg_print_num(request.req_message_rate,10);
            print_util_dbg_print("\r\n");
        }

        if (request.req_stream_id == 255)
        {
            // send full list of streams
            scheduler->run_all_tasks_now();
        }
        else
        {
            Scheduler_task* task = scheduler->get_task_by_id(request.req_stream_id);

            if (task != NULL)
            {
                if (request.start_stop)
                {
                    task->set_run_mode(Scheduler_task::Scheduler_task::RUN_REGULAR);
                }
                else
                {
                    task->set_run_mode(Scheduler_task::Scheduler_task::RUN_NEVER);
                }

                if (request.req_message_rate > 0)
                {
                    task->change_period(SchedulerIMEBASE / (uint32_t)request.req_message_rate);
                }
            }
            else
            {
                print_util_dbg_print("This stream ID is not registred and cannot be activated.\r\n");
            }
        } //end of else request.req_stream_id != 255
    }
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Mavlink_communication::Mavlink_communication(Serial& serial, State& state, File& file_storage, const conf_t& config) : 
    scheduler(config.scheduler_config),
    mavlink_stream(serial, config.mavlink_stream_config),
    message_handler(mavlink_stream, config.message_handler_config),
    onboard_parameters(scheduler, file_storage, state, message_handler, mavlink_stream, config.onboard_parameters_config),
    msg_sending_count(0)
{
    bool init_success = true;

    // try to allocate memory for config.max_msg_sending_count messages, if not possible
    // reduce max_msg_sending_count messages, until enough space is available
    for(max_msg_sending_count = config.max_msg_sending_count; max_msg_sending_count > 0; max_msg_sending_count--)
    {
        msg_send_list = (send_msg_handler_t*)malloc(sizeof(send_msg_handler_t[max_msg_sending_count]));
        if(msg_send_list != NULL)
        {
            break;
        }
    }

    if(max_msg_sending_count != config.max_msg_sending_count)
    {
        init_success = false;
        print_util_dbg_print("[MAVLINK COMMUNICATION].configure ERROR ! reserved ");
        print_util_dbg_print_num(max_msg_sending_count,10);
        print_util_dbg_print(" instead of ");
        print_util_dbg_print_num(config.max_msg_sending_count,10);
        print_util_dbg_print("\r\n");
    }

    // Add callback to activate / disactivate streams
    Mavlink_message_handler::msg_callback_t callback;

    callback.message_id     = MAVLINK_MSG_ID_REQUEST_DATA_STREAM; // 66
    callback.sysid_filter   = MAVLINK_BASE_STATION_ID;
    callback.compid_filter  = MAV_COMP_ID_ALL;
    callback.function       = (Mavlink_message_handler::msg_callback_func_t) &mavlink_communication_toggle_telemetry_stream;
    callback.module_struct  = (Mavlink_message_handler::handling_module_struct_t)        &scheduler;
    init_success &= message_handler.add_msg_callback(&callback);

    if(!init_success)
    {
        print_util_dbg_print("[MAVLINK COMMUNICATION] constructor error\r\n");
    }
}

void Mavlink_communication::suspend_downstream(uint32_t delay)
{
    scheduler.suspend_all_tasks(delay);
}


bool Mavlink_communication::add_msg_send(uint32_t repeat_period, Scheduler_task::run_mode_t run_mode, Scheduler_task::timing_mode_t timing_mode, Scheduler_task::priority_t priority, Mavlink_communication::send_msg_function_t function, handling_telemetry_module_struct_t module_structure, uint32_t task_id)
{
    bool add_success = true;

    if (msg_sending_count <  max_msg_sending_count)
    {
        send_msg_handler_t* new_msg_send = &msg_send_list[msg_sending_count++];

        new_msg_send->mavlink_stream = &mavlink_stream;
        new_msg_send->function = function;
        new_msg_send->module_struct = module_structure;

        add_success &= true;

        add_success &= scheduler.add_task(repeat_period,
                                          run_mode,
                                          timing_mode,
                                          priority,
                                          (Scheduler_task::task_function_t)&send_message,
                                          (Scheduler_task::task_argument_t)new_msg_send,
                                          task_id);
    }
    else
    {
        print_util_dbg_print("[MAVLINK COMMUNICATION] Error: Cannot add more send msg\r\n");

        add_success &= false;
    }

    return add_success;
}


bool Mavlink_communication::send_message(send_msg_handler_t* msg_send)
{
    bool success = true;

    Mavlink_communication::send_msg_function_t function = msg_send->function;
    handling_telemetry_module_struct_t module_struct = msg_send->module_struct;

    mavlink_message_t msg;
    function(module_struct, msg_send->mavlink_stream, &msg);

    success &= msg_send->mavlink_stream->send(&msg);

    return success;
}


Scheduler& Mavlink_communication::get_scheduler()
{
    return scheduler;
}

Mavlink_message_handler& Mavlink_communication::get_message_handler()
{
    return message_handler;
}

Mavlink_stream& Mavlink_communication::get_mavlink_stream()
{
    return mavlink_stream;
}

Onboard_parameters& Mavlink_communication::get_onboard_parameters()
{
    return onboard_parameters;
}


uint32_t Mavlink_communication::get_sysid()
{
    return mavlink_stream.sysid;
}

uint32_t* Mavlink_communication::get_sysid_ptr()
{
    return &mavlink_stream.sysid;
}



bool Mavlink_communication::update(Mavlink_communication* mavlink_communication)
{
    // Receive new message
    Mavlink_stream::msg_received_t rec;
    while (mavlink_communication->mavlink_stream.receive(&rec))
    {
            mavlink_communication->message_handler.receive(&rec);
    }

    // Send messages
    mavlink_communication->scheduler.update();

    return true;
}
