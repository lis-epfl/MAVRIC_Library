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


#define MAV_SYS_ID_ALL 0
#define MAV_MSG_ENUM_END 255

/**
 * \brief 		Pointer a module's data structure
 * 
 * \details 	This is used as an alias to any data structure in the prototype of callback functions 
 */
typedef void* handling_module_struct_t;


/**
 * \brief  		Prototype of callback functions for mavlink messages
 */
typedef void (*mavlink_msg_callback_function_t) (handling_module_struct_t, mavlink_message_t*);


/**
 * \brief  		Prototype of callback functions for mavlink commands
 */
typedef void (*mavlink_cmd_callback_function_t) (handling_module_struct_t, mavlink_command_long_t*);


/**
 * \brief		Enumeration of MAV components 
 * \details 	The enumeration MAV_COMPONENT is defined by mavlink
 */
typedef enum MAV_COMPONENT mav_component_t;


/**
 * \brief 		Message callback
 */
typedef struct
{
	uint8_t 						message_id;						///<	The function will be called only for messages with ID message_id
	uint8_t					 		sysid_filter;					///<	The function will be called only for messages coming from MAVs with ID sysid_filter (0 for all)
	mav_component_t 				compid_filter;					///<	The function will be called only for messages coming from component compid_filter (0 for all)
	mavlink_msg_callback_function_t function;						///<	Pointer to the function to be executed
	handling_module_struct_t 		module_struct;					///<	Pointer to module data structure to be given as argument to the function
} mavlink_message_handler_msg_callback_t;


/**
 * \brief 		Command callback
 */
typedef struct
{
	uint16_t 						command_id;						///< 	The function will be called only for commands with ID command_id
	uint8_t					 		sysid_filter;					///<	The function will be called only for commands coming from MAVs with ID sysid_filter (0 for all)
	mav_component_t 				compid_filter;					///<	The function will be called only for commands coming from component compid_filter (0 for all)
	mav_component_t 				compid_target;					///<	The function will be called only if the commands targets the component compid_target of this system (0 for all)
	mavlink_cmd_callback_function_t function;						///<	Pointer to the function to be executed
	handling_module_struct_t 		module_struct;					///<	Pointer to module data structure to be given as argument to the function
} mavlink_message_handler_cmd_callback_t;


/**
 * \brief 		Set of message callbacks
 * 
 * \details 	Uses C99's flexible member arrays: it is required to 
 * 				allocate memory for the callback_list 
 */
typedef struct
{
	uint32_t callback_count;										///<	Number of message callback currently registered
	uint32_t max_callback_count;									///<	Maximum number of callback that can be registered 
	mavlink_message_handler_msg_callback_t callback_list[];			///<	List of message callbacks
} mavlink_message_handler_msg_callback_set_t;


/**
 * \brief 		Set of command callbacks
 * 
 * \details 	Uses C99's flexible member arrays: it is required to 
 * 				allocate memory for this structure
 */
typedef struct
{
	uint32_t callback_count;										///<	Number of command callback currently registered
	uint32_t max_callback_count;									///<	Maximum number of callback that can be registered
	mavlink_message_handler_cmd_callback_t callback_list[];			///<	List of message callbacks
} mavlink_message_handler_cmd_callback_set_t;


/**
 * \brief 		Main message handler structure
 * 
 * \details  	msg_callback_set and cmd_callback_set are implemented as pointer
 * 				because their memory will be allocated during initialisation
 */
typedef struct
{
	mavlink_message_handler_msg_callback_set_t* msg_callback_set;	///<	Set of message callbacks
	mavlink_message_handler_cmd_callback_set_t* cmd_callback_set;	///<	Set of command callbacks
	bool debug;														///<	Indicates whether debug message are written for every incoming message
} mavlink_message_handler_t;


/**
 * \brief	Structure used to hold parameters during initialisation
 */
typedef struct
{
	uint32_t max_msg_callback_count;								///<	Maximum number of message callbacks
	uint32_t max_cmd_callback_count;								///<	Maximum number of command callbacks
	bool debug;														///<	Indicates whether debug message are written for every incoming message
} mavlink_message_handler_conf_t;


/**
 * \brief 						Initialises the message handler module 
 * 
 * \param 	message_handler 	Pointer to message handler data structure
 * \param 	config 				Config parameters
 */
void mavlink_message_handler_init(	mavlink_message_handler_t* 			message_handler, 
									const mavlink_message_handler_conf_t* 	config);


/**
 * \brief 						Registers a new callback for a message
 * 
 * \param 	message_handler 	Pointer to message handler data structure
 * \param 	msg_callback 		Pointer to new message callback (this structure 
 * 								is copied internally, so it does not need to be 
 * 								kept in memory after the function is called)
 */
void mavlink_message_handler_add_msg_callback(	mavlink_message_handler_t* 					message_handler, 
												mavlink_message_handler_msg_callback_t* 	msg_callback);


/**
 * \brief 						Registers a new callback for a command
 * 
 * \param 	message_handler 	Pointer to message handler data structure
 * \param 	cmd_callback 		Pointer to new command callback (this structure 
 * 								is copied internally, so it does not need to be 
 * 								kept in memory after the function is called)
 */
void mavlink_message_handler_add_cmd_callback(	mavlink_message_handler_t* 					message_handler, 
												mavlink_message_handler_cmd_callback_t* 	cmd_callback);


/**
 * \brief 			Dummy message callback for debug purpose 
 * \details  		Prints the fields of the incoming message to the debug console
 * 
 * \param 	msg 	Pointer to incoming message
 */
void mavlink_message_handler_msg_default_dbg(mavlink_message_t* msg);


/**
 * \brief 			Dummy command callback for debug purpose 
 * \details  		Prints the fields of the incoming command to the debug console
 * 
 * \param 	cmd 	Pointer to incoming command
 */
void mavlink_message_handler_cmd_default_dbg(mavlink_command_long_t* cmd);


/**
 * \brief		Main update function, handles the incoming message according to 
 * 				the registered message and command callbacks
 *
 * \param rec	Pointer to the mavlink receive message structure
 */
void mavlink_message_handler_receive(mavlink_message_handler_t* message_handler, mavlink_received_t* rec);


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_MESSAGE_HANDLING_H */