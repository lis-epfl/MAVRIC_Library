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

// TODO: update documentation


typedef struct 
{
	task_set_t task_set;
	mavlink_stream_t mavlink_stream;				
	mavlink_message_handler_t message_handler;
	onboard_parameter_set_t onboard_parameters;
} mavlink_communication_t;


typedef struct
{
	byte_stream_t* up_stream;
	byte_stream_t* down_stream;
	uint8_t sysid;
	uint8_t compid;
} mavlink_communication_conf_t;


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