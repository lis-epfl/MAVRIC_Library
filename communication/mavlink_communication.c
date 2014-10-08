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

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

static void mavlink_communication_toggle_telemetry_stream(scheduler_t* scheduler, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void mavlink_communication_toggle_telemetry_stream(scheduler_t* scheduler, mavlink_message_t* msg)
{
	// Get task set
	task_set_t* mavlink_task_set = scheduler->task_set;

	// Decode message
	mavlink_request_data_stream_t request;
	mavlink_msg_request_data_stream_decode(msg, &request);
	
	if (((request.target_system == scheduler->mavlink_stream->sysid)||(request.target_system == MAV_SYS_ID_ALL))
		&&(request.target_component == 0))
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
	// Init mavlink schedule
	scheduler_init(	&mavlink_communication->scheduler, 
					&config->scheduler_config,
					&mavlink_communication->mavlink_stream);

	// Init mavlink stream
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

	// Add callback to activate / disactivate streams
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_REQUEST_DATA_STREAM; // 66
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&mavlink_communication_toggle_telemetry_stream;
	callback.module_struct 	= (handling_module_struct_t)		&mavlink_communication->scheduler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
	
	print_util_dbg_print("MAVLink communication initlialized\r\n");
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