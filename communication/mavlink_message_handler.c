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
 * \file mavlink_message_handler.c
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief This module handles of all incoming MAVLink message by calling the 
 * appropriate functions
 *
 ******************************************************************************/


#include "mavlink_message_handler.h"
#include "print_util.h"
#include "piezo_speaker.h"
#include <stdlib.h>
#include <stdbool.h>


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief 					Checks whether a message matches with a registered callback
 * 
 * \param 	msg_callback 	Pointer to a registered message callback
 * \param 	msg 			Incoming message
 * 
 * \return 					Boolean (true if the message matches, false if not)
 */
static bool match_msg(mavlink_message_handler_t* message_handler, mavlink_message_handler_msg_callback_t* msg_callback, mavlink_message_t* msg);


/**
 * \brief 					Checks whether a command matches with a registered callback
 * 
 * \param 	msg_callback 	Pointer to a registered command callback
 * \param 	msg 			Incoming message containing the command
 * \param 	cmd 			Incoming command encoded in the message
 * 
 * \return 					Boolean (true if the message matches, false if not)
 */
static bool match_cmd(mavlink_message_handler_t* message_handler, mavlink_message_handler_cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static bool match_msg(mavlink_message_handler_t* message_handler, mavlink_message_handler_msg_callback_t* msg_callback, mavlink_message_t* msg)
{
	bool match = false;

	uint8_t sysid = message_handler->mavlink_stream->sysid;

	if ( msg->sysid != sysid )																		// This message is not from this system
	{
		if ( msg_callback->message_id == msg->msgid )																// The message has the good ID
		{
			if ( msg_callback->sysid_filter == MAV_SYS_ID_ALL || msg_callback->sysid_filter == msg->sysid )			// The message is from the good system
			{
				if ( msg_callback->compid_filter == MAV_COMP_ID_ALL || msg_callback->compid_filter == msg->compid )	// The system is from the good component 
				{
					match = true;
				}
			}
		}
	}

	return match;
}


static bool match_cmd(mavlink_message_handler_t* message_handler, mavlink_message_handler_cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd)
{
	bool match = false;
	
	uint8_t sysid = message_handler->mavlink_stream->sysid;

	if ( msg->sysid != sysid )																							// This message is not from this system
	{
		if ( cmd_callback->command_id == cmd->command )																					// The message has the good ID
		{
			if ( cmd_callback->sysid_filter == MAV_SYS_ID_ALL || cmd_callback->sysid_filter == msg->sysid )								// The message is from the good system
			{
				if ( cmd_callback->compid_filter == MAV_COMP_ID_ALL || cmd_callback->compid_filter == msg->compid )						// The system is from the good component 
				{
					if ( cmd_callback->compid_target == MAV_COMP_ID_ALL || cmd_callback->compid_target == cmd->target_component )		// This system is the target of the command
					{				
						match = true;
					}
				}
			}
		}
	}

	return match;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_message_handler_init(mavlink_message_handler_t* message_handler, const mavlink_message_handler_conf_t* config, const mavlink_stream_t* mavlink_stream)
{
	// Init dependencies
	message_handler->mavlink_stream = mavlink_stream;

	// Init debug mode
	message_handler->debug = config->debug;

	// Allocate memory for msg handling
	message_handler->msg_callback_set = malloc( sizeof(mavlink_message_handler_msg_callback_set_t) + sizeof(mavlink_message_handler_msg_callback_t[config->max_msg_callback_count]) );
    
    if ( message_handler->msg_callback_set != NULL )
    {
	    message_handler->msg_callback_set->max_callback_count = config->max_msg_callback_count;
		message_handler->msg_callback_set->callback_count = 0;
	}
	else
	{
		print_util_dbg_print("[MESSAGE HANDLER] ERROR ! Bad memory allocation\r\n");

		message_handler->msg_callback_set->max_callback_count = 0;
		message_handler->msg_callback_set->callback_count = 0;	
	}


	// Allocate memory for msg handling
	message_handler->cmd_callback_set = malloc( sizeof(mavlink_message_handler_cmd_callback_set_t) + sizeof(mavlink_message_handler_cmd_callback_t[config->max_cmd_callback_count]) ); 
    if ( message_handler->cmd_callback_set != NULL )
    {
	    message_handler->cmd_callback_set->max_callback_count = config->max_cmd_callback_count;
		message_handler->cmd_callback_set->callback_count = 0;	
	}
	else
	{
		print_util_dbg_print("[COMMAND HANDLER] ERROR ! Bad memory allocation\r\n");
		message_handler->cmd_callback_set->max_callback_count = 0;
		message_handler->cmd_callback_set->callback_count = 0;		
	}
}


void mavlink_message_handler_add_msg_callback(	mavlink_message_handler_t* 				message_handler, 
												mavlink_message_handler_msg_callback_t* msg_callback)
{
	mavlink_message_handler_msg_callback_set_t* msg_callback_set = message_handler->msg_callback_set;
	
	if ( msg_callback_set->callback_count <  msg_callback_set->max_callback_count )
	{
		mavlink_message_handler_msg_callback_t* new_callback = &msg_callback_set->callback_list[msg_callback_set->callback_count];

		new_callback->sys_id		= &(message_handler->mavlink_stream->sysid);
		new_callback->message_id 	= msg_callback->message_id;
		new_callback->sysid_filter 	= msg_callback->sysid_filter;
	 	new_callback->compid_filter = msg_callback->compid_filter;
		new_callback->function 		= msg_callback->function;
		new_callback->module_struct = msg_callback->module_struct;

		msg_callback_set->callback_count += 1;
	}
	else
	{
		print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");
	}
}


void mavlink_message_handler_add_cmd_callback(	mavlink_message_handler_t* 				message_handler, 
												mavlink_message_handler_cmd_callback_t*	cmd_callback)
{
	mavlink_message_handler_cmd_callback_set_t* cmd_callback_set = message_handler->cmd_callback_set;
	
	if ( cmd_callback_set->callback_count <  cmd_callback_set->max_callback_count )
	{
		mavlink_message_handler_cmd_callback_t* new_callback = &cmd_callback_set->callback_list[cmd_callback_set->callback_count];

		new_callback->command_id = cmd_callback->command_id;
		new_callback->sysid_filter = cmd_callback->sysid_filter;
	 	new_callback->compid_filter = cmd_callback->compid_filter;
		new_callback->compid_target = cmd_callback->compid_target;
		new_callback->function = cmd_callback->function;
		new_callback->module_struct = cmd_callback->module_struct;

		cmd_callback_set->callback_count += 1;
	}
	else
	{
		print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback\r\n");
	}
}


void mavlink_message_handler_msg_default_dbg(mavlink_message_t* msg)
{
	if ((msg->sysid == MAVLINK_BASE_STATION_ID)&&(msg->msgid != MAVLINK_MSG_ID_MANUAL_CONTROL))
	{
		print_util_dbg_print("Received message with ID");
		print_util_dbg_print_num(msg->msgid, 10);
		print_util_dbg_print(" from system");
		print_util_dbg_print_num(msg->sysid, 10);
		print_util_dbg_print(" from component");
		print_util_dbg_print_num(msg->compid,10);
		print_util_dbg_print("\r\n");
	}
}


void mavlink_message_handler_cmd_default_dbg(mavlink_command_long_t* cmd)
{
	print_util_dbg_print("Received command with ID");
	print_util_dbg_print_num(cmd->command,10);
	print_util_dbg_print(" with parameters: [");
	print_util_dbg_print_num(cmd->param1,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param2,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param3,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param4,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param5,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param6,10);
	print_util_dbg_print(", ");
	print_util_dbg_print_num(cmd->param7,10);
	print_util_dbg_print("] confirmation: ");
	print_util_dbg_print_num(cmd->confirmation,10);
	print_util_dbg_print("\r\n");
}


void mavlink_message_handler_receive(mavlink_message_handler_t* message_handler, mavlink_received_t* rec) 
{
	mavlink_message_t* msg = &rec->msg;	

	if ( message_handler->debug )
	{
		mavlink_message_handler_msg_default_dbg(msg);
	}

	if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG)
	{
		// The message is a command
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(msg, &cmd);
		
		 //print packet command and parameters for debug
		 print_util_dbg_print("target sysID:");
		 print_util_dbg_print_num(cmd.target_system,10);
		 print_util_dbg_print(", target compID:");
		 print_util_dbg_print_num(cmd.target_component,10);
		 print_util_dbg_print("\r\n");
		 print_util_dbg_print("parameters: ");
		 print_util_dbg_print_num(cmd.param1,10);
		 print_util_dbg_print_num(cmd.param2,10);
		 print_util_dbg_print_num(cmd.param3,10);
		 print_util_dbg_print_num(cmd.param4,10);
		 print_util_dbg_print_num(cmd.param5,10);
		 print_util_dbg_print_num(cmd.param6,10);
		 print_util_dbg_print_num(cmd.param7,10);
		 print_util_dbg_print("\r\n");
		 print_util_dbg_print("command id:");
		 print_util_dbg_print_num(cmd.command,10);
		 print_util_dbg_print(", confirmation:");
		 print_util_dbg_print_num(cmd.confirmation,10);
		 print_util_dbg_print("\r\n");
		
		if (cmd.command >= 0 && cmd.command < MAV_CMD_ENUM_END)
		{
			// The command has valid command ID 
			if(	(cmd.target_system == message_handler->mavlink_stream->sysid)||(cmd.target_system == MAV_SYS_ID_ALL) )
			{
				mav_result_t result = MAV_RESULT_UNSUPPORTED;
				
				// The command is for this system
				for (uint32_t i = 0; i < message_handler->cmd_callback_set->callback_count; ++i)
				{
					if ( match_cmd(message_handler, &message_handler->cmd_callback_set->callback_list[i], msg, &cmd) )
					{
						mavlink_cmd_callback_function_t function 		= message_handler->cmd_callback_set->callback_list[i].function;
						handling_module_struct_t 		module_struct 	= message_handler->cmd_callback_set->callback_list[i].module_struct;
						
						// Call appropriate function callback
						result = function(module_struct, &cmd);
						break;
					}
				}
				// Send acknowledgment message 
				mavlink_message_t msg;
				mavlink_msg_command_ack_pack( 	message_handler->mavlink_stream->sysid,
												message_handler->mavlink_stream->compid,
												&msg,
												cmd.command,
												result);
				mavlink_stream_send(message_handler->mavlink_stream, &msg);
			}
		}
	}
	else if ( msg->msgid >= 0 && msg->msgid < MAV_MSG_ENUM_END )
	{
		// The message has a valid message ID, and is not a command
		for (uint32_t i = 0; i < message_handler->msg_callback_set->callback_count; ++i)
		{
			if ( match_msg(message_handler, &message_handler->msg_callback_set->callback_list[i], msg) )
			{
				mavlink_msg_callback_function_t function 		= message_handler->msg_callback_set->callback_list[i].function;
				handling_module_struct_t 		module_struct 	= message_handler->msg_callback_set->callback_list[i].module_struct;
				uint32_t						sys_id			= *message_handler->msg_callback_set->callback_list[i].sys_id;
				
				// Call appropriate function callback
				function(module_struct, sys_id, msg);
			}
		}
	}
}