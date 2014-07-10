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
 * \file mavlink_message_handler.h
 *
 * This module handles of all incoming mavlink message by calling the appropriate functions
 */


#ifndef MAVLINK_MESSAGE_HANDLING_H_
#define MAVLINK_MESSAGE_HANDLING_H_

#ifdef __cplusplus
extern "C" {
#endif

// #include "onboard_parameters.h"
#include "mavlink_stream.h"


// TODO : Update documentation

#define MAV_MSG_ENUM_END 255



typedef void (*mavlink_msg_callback_t) (mavlink_received_t*);


typedef void (*mavlink_cmd_callback_t) (mavlink_command_long_t*);


typedef struct
{
	mavlink_msg_callback_t msg_from_ground_station[MAV_MSG_ENUM_END];
	mavlink_cmd_callback_t cmd_from_ground_station[MAV_CMD_ENUM_END];
	mavlink_msg_callback_t msg_from_other_mav[MAV_MSG_ENUM_END];	
	mavlink_cmd_callback_t cmd_from_other_mav[MAV_CMD_ENUM_END];
} mavlink_message_handler_t;


typedef enum
{
	MAV_MSG_HANDLER_INIT_DO_NOTHING = 0,	
	MAV_MSG_HANDLER_INIT_DEBUG = 1,	
} mavlink_message_handler_init_method_t;


void mavlink_message_handler_init(mavlink_message_handler_t* mavlink_message_handler, mavlink_message_handler_init_method_t method);


/**
 * \brief		handle specific mavlink message
 *
 * \param rec	Pointer to the mavlink receive message structure
 */
void mavlink_message_handler_receive(mavlink_message_handler_t* message_handler, mavlink_received_t* rec);


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_MESSAGE_HANDLING_H */