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

typedef void* handling_module_struct_t;


typedef void (*mavlink_msg_callback_function_t) (handling_module_struct_t, mavlink_received_t*);


typedef void (*mavlink_cmd_callback_function_t) (handling_module_struct_t, mavlink_command_long_t*);


typedef enum MAV_COMPONENT mav_component_t;


typedef enum
{
	FROM_GROUND_STATION = 0,
	FROM_OTHER_MAV = 1,
	FROM_BOTH = 2
} mavlink_message_source_t;


typedef struct
{
	uint8_t 						message_id;
	mavlink_message_source_t 		sysid_filter;
	mav_component_t 				compid_filter;
	mavlink_msg_callback_function_t function;
	handling_module_struct_t 		module_struct;
} mavlink_message_handler_msg_callback_t;


typedef struct
{
	uint8_t 						command_id;
	mavlink_message_source_t 		sysid_filter;
	mav_component_t 				compid_filter;
	mavlink_cmd_callback_function_t function;
	handling_module_struct_t 		module_struct;
} mavlink_message_handler_cmd_callback_t;


typedef struct
{
	uint32_t callback_count;
	uint32_t max_callback_count;
	mavlink_message_handler_msg_callback_t callback_list[];
} mavlink_message_handler_msg_callback_set_t;


typedef struct
{
	uint32_t callback_count;
	uint32_t max_callback_count;
	mavlink_message_handler_cmd_callback_t callback_list[];
} mavlink_message_handler_cmd_callback_set_t;


typedef struct
{
	mavlink_message_handler_msg_callback_set_t* msg_callback_set;	
	mavlink_message_handler_cmd_callback_set_t* cmd_callback_set;
	bool debug;
} mavlink_message_handler_t;


typedef struct
{
	uint32_t max_msg_callback_count;	
	uint32_t max_cmd_callback_count;
	bool debug;
} mavlink_message_handler_conf_t;


void mavlink_message_handler_init(	mavlink_message_handler_t* 		mavlink_message_handler, 
									mavlink_message_handler_conf_t 	config);


void mavlink_message_handler_add_msg_callback(	mavlink_message_handler_t* 				message_handler, 
												mavlink_message_handler_msg_callback_t 	msg_callback);


void mavlink_message_handler_add_cmd_callback(	mavlink_message_handler_t* 				message_handler, 
												mavlink_message_handler_cmd_callback_t 	cmd_callback);


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