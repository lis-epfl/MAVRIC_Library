/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file mavlink_communication.c
 *
 * This module takes care of sending periodic telemetric messages and handling incoming messages 
 */



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
		print_util_dbg_print("\n");
		print_util_dbg_print("\n");	
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

	callback.message_id 	= MAVLINK_MSG_ID_REQUEST_DATA_STREAM;
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&mavlink_communication_toggle_telemetry_stream;
	callback.module_struct 	= (handling_module_struct_t)		&mavlink_communication->scheduler;
	mavlink_message_handler_add_msg_callback( &mavlink_communication->message_handler, &callback );
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