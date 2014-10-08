/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
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


#include "mavlink_communication.h"
#include "print_util.h"
#include <stdlib.h>

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

static void mavlink_communication_toggle_telemetry_stream(scheduler_t* scheduler, uint32_t sysid, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void mavlink_communication_toggle_telemetry_stream(scheduler_t* scheduler, uint32_t sysid, mavlink_message_t* msg)
{
	// Get task set
	task_set_t* mavlink_task_set = scheduler->task_set;

	// Decode message
	mavlink_request_data_stream_t request;
	mavlink_msg_request_data_stream_decode(msg, &request);
	
	if (((request.target_system == sysid)||(request.target_system == MAV_SYS_ID_ALL)))
		//&&(request.target_component == 0))
	{
		if ( scheduler->debug )
		{
			print_util_dbg_print("stream request:");
			print_util_dbg_print_num(request.target_component, 10);
			print_util_dbg_print(" stream="); 
			print_util_dbg_print_num(request.req_stream_id, 10);
			print_util_dbg_print(" start_stop=");
			print_util_dbg_print_num(request.start_stop, 10);
			print_util_dbg_print(" rate=");
			print_util_dbg_print_num(request.req_message_rate, 10);
			print_util_dbg_print("\r\n");
		}

		if ( request.req_stream_id==255 ) 
		{
			// send full list of streams
			for (int32_t i = 0; i < mavlink_task_set->task_count; i++) 
			{
				task_entry_t* task = scheduler_get_task_by_index(scheduler, i);
				scheduler_run_task_now(task);
			}					
		} 
		else 
		{
			task_entry_t* task = scheduler_get_task_by_id(scheduler, request.req_stream_id);
		
			if ( task != NULL )
			{
				if (request.start_stop) 
				{
					scheduler_change_run_mode(task, RUN_REGULAR);
				}
				else 
				{
					scheduler_change_run_mode(task, RUN_NEVER);
				}

				if (request.req_message_rate > 0) 
				{
					scheduler_change_task_period(task, SCHEDULER_TIMEBASE / (uint32_t)request.req_message_rate);
				}
			}
			else
			{
				print_util_dbg_print("This stream ID is not registred and cannot be activated.\r\n");
			}
		}
	}	
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_communication_init(mavlink_communication_t* mavlink_communication, const mavlink_communication_conf_t* config)
{
	// Init MAVLink schedule
	scheduler_init(	&mavlink_communication->scheduler, 
					&config->scheduler_config);

	// Init MAVLink stream
	mavlink_stream_init(	&mavlink_communication->mavlink_stream, 
							&config->mavlink_stream_config	);

	mavlink_message_handler_init(	&mavlink_communication->message_handler, 
									&config->message_handler_config,
									&mavlink_communication->mavlink_stream);

	// Init onboard parameters
	onboard_parameters_init(	&mavlink_communication->onboard_parameters, 
								&config->onboard_parameters_config, 
								&mavlink_communication->scheduler, 
								&mavlink_communication->message_handler,
								&mavlink_communication->mavlink_stream); 

	mavlink_communication->send_msg_handler_set = malloc( sizeof(mavlink_send_msg_handler_set_t) + sizeof(mavlink_send_msg_handler_t[config->max_msg_sending_count]) );


	if ( mavlink_communication->send_msg_handler_set != NULL )
	{
		mavlink_communication->send_msg_handler_set->max_msg_sending_count = config->max_msg_sending_count;
		mavlink_communication->send_msg_handler_set->msg_sending_count = 0;
	}
	else
	{
		print_util_dbg_print("[MAVLINK COMMUNICATION] ERROR ! Bad memory allocation\r\n");

		mavlink_communication->send_msg_handler_set->max_msg_sending_count = 0;
		mavlink_communication->send_msg_handler_set->msg_sending_count = 0;
	}

	mavlink_communication->send_msg_handler_set->max_msg_sending_count = config->max_msg_sending_count;

	// Add callback to activate / disactivate streams
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_REQUEST_DATA_STREAM; // 66
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&mavlink_communication_toggle_telemetry_stream;
	callback.module_struct 	= (handling_module_struct_t)		&mavlink_communication->scheduler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );

	print_util_dbg_print("[MAVLINK COMMUNICATION] Initialised\r\n");
}


task_return_t mavlink_communication_update(mavlink_communication_t* mavlink_communication) 
{
	task_return_t result = 0;

	mavlink_stream_t* mavlink_stream = &mavlink_communication->mavlink_stream;
	mavlink_message_handler_t* handler = &mavlink_communication->message_handler;

	// Receive new message
	mavlink_stream_receive(mavlink_stream);

	// Handle message
	if (mavlink_stream->msg_available == true)
	{
		mavlink_message_handler_receive(handler, &mavlink_stream->rec);
		mavlink_stream->msg_available = false;
	}
	
	// Send messages
	if (mavlink_stream->tx->buffer_empty(mavlink_stream->tx->data) == true) 
	{
		result = scheduler_update(&mavlink_communication->scheduler);
	}
	
	return result;
}


void mavlink_communication_suspend_downstream(mavlink_communication_t* mavlink_communication, uint32_t delay) 
{
	int32_t i;
	for (i = 0; i < mavlink_communication->scheduler.task_set->task_count; i++) 
	{
		scheduler_suspend_task(&mavlink_communication->scheduler.task_set->tasks[i], delay);
	}	
}


void mavlink_communication_add_msg_send(	mavlink_communication_t* mavlink_communication, uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, mavlink_send_msg_function_t function, handling_telemetry_module_struct_t module_structure, uint32_t task_id)
{
	mavlink_send_msg_handler_set_t* send_handler = mavlink_communication->send_msg_handler_set;
	
	
	if ( send_handler->msg_sending_count <  send_handler->max_msg_sending_count )
	{
		mavlink_send_msg_handler_t* new_msg_send = &send_handler->msg_send_list[send_handler->msg_sending_count];
		
		new_msg_send->mavlink_stream = &mavlink_communication->mavlink_stream;
		new_msg_send->function = function;
		new_msg_send->module_struct = module_structure;

		send_handler->msg_sending_count += 1;
		
		scheduler_add_task(	&mavlink_communication->scheduler,
							repeat_period,
							run_mode,
							timing_mode,
							priority,
							(task_function_t)&mavlink_communication_send_message,
							(task_argument_t)new_msg_send,
							task_id	);
	}
	else
	{
		print_util_dbg_print("[MAVLINK COMMUNICATION] Error: Cannot add more send msg\r\n");
	}
}


task_return_t mavlink_communication_send_message(mavlink_send_msg_handler_t* msg_send)
{
	mavlink_send_msg_function_t function = msg_send->function;
	handling_telemetry_module_struct_t module_struct = msg_send->module_struct;
	
	mavlink_message_t msg;
	function(module_struct, msg_send->mavlink_stream, &msg);
	
	mavlink_stream_send(msg_send->mavlink_stream,&msg);
	
	return TASK_RUN_SUCCESS;
}