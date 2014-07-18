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


void mavlink_communication_init(mavlink_communication_t* mavlink_communication, const mavlink_communication_conf_t* config)
{
	// Init mavlink schedule
	scheduler_init(	&mavlink_communication->scheduler, 
					&config->scheduler_config	);

	// Init mavlink stream
	mavlink_stream_init(	&mavlink_communication->mavlink_stream, 
							&config->mavlink_stream_config	);

	mavlink_message_handler_init(	&mavlink_communication->message_handler, 
									&config->message_handler_config	);

	// Init onboard parameters
	onboard_parameters_init(	&mavlink_communication->onboard_parameters, 
								&config->onboard_parameters_config, 
								&mavlink_communication->scheduler, 
								&mavlink_communication->message_handler ); 
}


task_return_t mavlink_communication_update(mavlink_communication_t* mavlink_communication) 
{
	task_return_t result = 0;

	mavlink_stream_t* mavlink_stream = &mavlink_communication->mavlink_stream;
	mavlink_message_handler_t* handler = &mavlink_communication->message_handler;

	// Receive new message
	mavlink_stream_receive(mavlink_stream);

	// Handle message
	if (mavlink_stream->message_available == true)
	{
		mavlink_message_handler_receive(handler, &mavlink_stream->rec);
		mavlink_stream->message_available = false;
	}
	
	// Send messages
	if (mavlink_stream->out_stream->buffer_empty(mavlink_stream->out_stream->data) == true) 
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