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
 * \file mavlink_communication.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This module takes care of sending periodic telemetric messages and 
 * handling incoming messages 
 *
 ******************************************************************************/


#ifndef MAVLINK_COMMUNICATION_H_
#define MAVLINK_COMMUNICATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "scheduler.h"
#include "mavlink_stream.h"
#include "mavlink_message_handler.h"
#include "onboard_parameters.h"


/**
 * \brief 		Pointer a module's data structure
 * 
 * \details 	This is used as an alias to any data structure in the prototype of callback functions 
 */
typedef void* handling_telemetry_module_struct_t;


/**
 * \brief  		Prototype of callback functions for MAVLink messages
 */
typedef void (*mavlink_send_msg_function_t) (handling_telemetry_module_struct_t, mavlink_stream_t*, mavlink_message_t*);


typedef struct  
{
	mavlink_stream_t* mavlink_stream;								///<	Pointer to the MAVLink stream structure
	mavlink_send_msg_function_t function;							///<	Pointer to the function to be executed
	handling_telemetry_module_struct_t 		module_struct;			///<	Pointer to module data structure to be given as argument to the function
}mavlink_send_msg_handler_t;

typedef struct  
{
	uint32_t msg_sending_count;										///<	Number of message callback currently registered
	uint32_t max_msg_sending_count;									///<	Maximum number of callback that can be registered
	mavlink_send_msg_handler_t msg_send_list[];						///<	List of message callbacks
}mavlink_send_msg_handler_set_t;

/**
 * \brief 	Main MAVLink Communication structure
 */
typedef struct 
{
	scheduler_t 					scheduler;						///<	Task set for scheduling of down messages
	mavlink_stream_t 				mavlink_stream;					///< 	Mavlink interface using streams
	mavlink_message_handler_t 		message_handler;				///< 	Message handler
	onboard_parameters_t 			onboard_parameters;				///< 	Onboard parameters
	
	mavlink_send_msg_handler_set_t*	send_msg_handler_set;			///<	Pointer to the sending message handler set

} mavlink_communication_t;


/**
 * \brief 	Configuratioon of the module Mavlink Communication
 */
typedef struct
{
	scheduler_conf_t				scheduler_config;
	mavlink_stream_conf_t 			mavlink_stream_config;			///< 	Configuration for the module MAVLink stream
	mavlink_message_handler_conf_t	message_handler_config;			///< 	Configuration for the module message handler
	onboard_parameters_conf_t		onboard_parameters_config;		///< 	Configuration for the module onboard parameters
	
	uint32_t						max_msg_sending_count;			///<	Configuration for the sending message handler
} mavlink_communication_conf_t;



/**
 * \brief 	Initialisation of the module MAVLink communication
 * 
 * \param 	mavlink_communication 	Pointer to the MAVLink communication structure
 * \param 	config 					Configuration
 */
void mavlink_communication_init(mavlink_communication_t* mavlink_communication, const mavlink_communication_conf_t* config);


/**
 * \brief	Run task scheduler update if the buffer is empty 
 *
 * \param 	mavlink_communication 	Pointer to the MAVLink communication structure
 *
 * \return	Task status return
 */
task_return_t mavlink_communication_update(mavlink_communication_t* mavlink_communication);


/**
 * \brief	Suspending sending of messages
 *
 * \param 	mavlink_communication 	Pointer to the MAVLink communication structure
 * \param 	delay					Delay of suspension in microsecond
 */
void mavlink_communication_suspend_downstream(mavlink_communication_t* mavlink_communication, uint32_t delay);

/**
 * \brief	Adding new message to the MAVLink scheduler
 *
 * \param 	mavlink_communication 	Pointer to the MAVLink communication structure
 * \param 	repeat_period			Repeat period (us)
 * \param	run_mode				Run mode
 * \param	timing_mode				Timing mode
 * \param	priority				Priority
 * \param	function 				Function pointer to be called
 * \param	module_structure		Argument to be passed to the function
 * \param	task_id			    	Unique task identifier
 */
void mavlink_communication_add_msg_send(	mavlink_communication_t* mavlink_communication, uint32_t repeat_period, task_run_mode_t run_mode, task_timing_mode_t timing_mode, task_priority_t priority, mavlink_send_msg_function_t function, handling_telemetry_module_struct_t module_structure, uint32_t task_id);

/**
 * \brief	Suspending sending of messages
 *
 * \param 	msg_send 	The MAVLink message sending handler
 *
 * \return	The result of execution of the task
 */
task_return_t mavlink_communication_send_message(mavlink_send_msg_handler_t* msg_send);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_COMMUNICATION_H_ */