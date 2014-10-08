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
 * \file mavlink_message_handler.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This module handles of all incoming MAVLink message by calling the 
 * appropriate functions
 *
 ******************************************************************************/


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
 * \brief		Enumeration of command MAV result
 * \details 	The enumeration MAV_RESULT is defined by MAVLink
 */
typedef enum MAV_RESULT mav_result_t;

/**
 * \brief  		Prototype of callback functions for MAVLink messages
 */
typedef void (*mavlink_msg_callback_function_t) (handling_module_struct_t, uint32_t sysid, mavlink_message_t*);


/**
 * \brief  		Prototype of callback functions for MAVLink commands
 */
typedef mav_result_t (*mavlink_cmd_callback_function_t) (handling_module_struct_t, mavlink_command_long_t*);


/**
 * \brief		Enumeration of MAV components 
 * \details 	The enumeration MAV_COMPONENT is defined by MAVLink
 */
typedef enum MAV_COMPONENT mav_component_t;


/**
 * \brief 		Message callback
 */
typedef struct
{
	const uint32_t*						sys_id;						///<	Pointer to the system ID
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
	const mavlink_stream_t* mavlink_stream;
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
									const mavlink_message_handler_conf_t* 	config,
									const mavlink_stream_t* mavlink_stream);


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
 * \param rec	Pointer to the MAVLink receive message structure
 */
void mavlink_message_handler_receive(mavlink_message_handler_t* message_handler, mavlink_received_t* rec);


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_MESSAGE_HANDLING_H */