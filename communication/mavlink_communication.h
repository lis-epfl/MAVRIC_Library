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
 * \brief 	Main Mavlink Communication structure
 */
typedef struct 
{
	scheduler_t 					scheduler;					///<	Task set for scheduling of down messages
	mavlink_stream_t 				mavlink_stream;				///< 	Mavlink interface using streams
	mavlink_message_handler_t 		message_handler;			///< 	Message handler
	onboard_parameters_t 			onboard_parameters;			///< 	Onboard parameters
} mavlink_communication_t;


/**
 * \brief 	Configuratioon of the module Mavlink Communication
 */
typedef struct
{
	scheduler_conf_t				scheduler_config;
	mavlink_stream_conf_t 			mavlink_stream_config;		///< 	Configuration for the module mavlink stream
	mavlink_message_handler_conf_t	message_handler_config;		///< 	Configuration for the module message handler
	onboard_parameters_conf_t		onboard_parameters_config;	///< 	Configuration for the module onboard parameters
} mavlink_communication_conf_t;


/**
 * \brief 	Initialisation of the module mavlink communication
 * 
 * \param 	mavlink_communication 	Pointer to the mavlink communication structure
 * \param 	config 					Configuration
 */
void mavlink_communication_init(mavlink_communication_t* mavlink_communication, const mavlink_communication_conf_t* config);


/**
 * \brief		Run task scheduler update if the buffer is empty 
 *
 * \return		Task status return
 */
task_return_t mavlink_communication_update(mavlink_communication_t* mavlink_communication);


/**
 * \brief			Suspending sending of messages
 *
 * \param 	delay	Delay of suspension in microsecond
 */
void mavlink_communication_suspend_downstream(mavlink_communication_t* mavlink_communication, uint32_t delay);


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_COMMUNICATION_H_ */