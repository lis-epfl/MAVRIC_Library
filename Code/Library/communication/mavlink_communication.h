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
 * \file mavlink_communication.h
 *
 * This module takes care of sending periodic telemetric messages and handling incoming messages 
 */


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
	task_set_t 						task_set;					///<	Task set for scheduling of down messages
	mavlink_stream_t 				mavlink_stream;				///< 	Mavlink interface using streams
	mavlink_message_handler_t 		message_handler;			///< 	Message handler
	onboard_parameters_t 			onboard_parameters;			///< 	Onboard parameters
} mavlink_communication_t;


/**
 * \brief 	Configuratioon of the module Mavlink Communication
 */
typedef struct
{
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