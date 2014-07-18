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
 * \file mavlink_message_handler.c
 *
 * This module handles of all incoming mavlink message by calling the appropriate functions
 */

// TODO: update documentation

#include "mavlink_message_handler.h"
#include "print_util.h"

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
static bool match_msg(mavlink_message_handler_msg_callback_t* msg_callback, mavlink_message_t* msg);


/**
 * \brief 					Checks whether a command matches with a registered callback
 * 
 * \param 	msg_callback 	Pointer to a registered command callback
 * \param 	msg 			Incoming message containing the command
 * \param 	cmd 			Incoming command encoded in the message
 * 
 * \return 					Boolean (true if the message matches, false if not)
 */
static bool match_cmd(mavlink_message_handler_cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static bool match_msg(mavlink_message_handler_msg_callback_t* msg_callback, mavlink_message_t* msg)
{
	bool match = false;

	if ( msg->sysid != mavlink_system.sysid )																		// This message is not from this system
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


static bool match_cmd(mavlink_message_handler_cmd_callback_t* cmd_callback, mavlink_message_t* msg, mavlink_command_long_t* cmd)
{
	bool match = false;
	
	if ( msg->sysid != mavlink_system.sysid )																							// This message is not from this system
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

void mavlink_message_handler_init(mavlink_message_handler_t* message_handler, const mavlink_message_handler_conf_t* config)
{
	// Init debug mode
	message_handler->debug = config->debug;

	// Allocate memory for msg handling
	message_handler->msg_callback_set = malloc( sizeof(mavlink_message_handler_msg_callback_set_t) + sizeof(mavlink_message_handler_msg_callback_t[config->max_msg_callback_count]) );
    message_handler->msg_callback_set->max_callback_count = config->max_msg_callback_count;
	message_handler->msg_callback_set->callback_count = 0;

	// Allocate memory for msg handling
	message_handler->cmd_callback_set = malloc( sizeof(mavlink_message_handler_cmd_callback_set_t) + sizeof(mavlink_message_handler_cmd_callback_t[config->max_cmd_callback_count]) );
    message_handler->cmd_callback_set->max_callback_count = config->max_cmd_callback_count;
	message_handler->cmd_callback_set->callback_count = 0;	
}


void mavlink_message_handler_add_msg_callback(	mavlink_message_handler_t* 				message_handler, 
												mavlink_message_handler_msg_callback_t* msg_callback)
{
	mavlink_message_handler_msg_callback_set_t* msg_callback_set = message_handler->msg_callback_set;
	
	if ( msg_callback_set->callback_count <  msg_callback_set->max_callback_count )
	{
		mavlink_message_handler_msg_callback_t* new_callback = &msg_callback_set->callback_list[msg_callback_set->callback_count];

		new_callback->message_id 	= msg_callback->message_id;
		new_callback->sysid_filter 	= msg_callback->sysid_filter;
	 	new_callback->compid_filter = msg_callback->compid_filter;
		new_callback->function 		= msg_callback->function;
		new_callback->module_struct = msg_callback->module_struct;

		msg_callback_set->callback_count += 1;
	}
	else
	{
		print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback");
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
		print_util_dbg_print("[MESSAGE HANDLER] Error: Cannot add more msg callback");
	}
}


void mavlink_message_handler_msg_default_dbg(mavlink_message_t* msg)
{
	print_util_dbg_print("\n Received message with ID");
	print_util_dbg_print_num(msg->msgid, 10);
	print_util_dbg_print(" from system");
	print_util_dbg_print_num(msg->sysid, 10);
	print_util_dbg_print(" from component");
	print_util_dbg_print_num(msg->compid,10);
	print_util_dbg_print( "\n");
}


void mavlink_message_handler_cmd_default_dbg(mavlink_command_long_t* cmd)
{
	print_util_dbg_print("\n Received command with ID");
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
	print_util_dbg_print("\n");
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
		 print_util_dbg_print("target compID:");
		 print_util_dbg_print_num(cmd.target_component,10);
		 print_util_dbg_print("parameters:");
		 print_util_dbg_print_num(cmd.param1,10);
		 print_util_dbg_print_num(cmd.param2,10);
		 print_util_dbg_print_num(cmd.param3,10);
		 print_util_dbg_print_num(cmd.param4,10);
		 print_util_dbg_print_num(cmd.param5,10);
		 print_util_dbg_print_num(cmd.param6,10);
		 print_util_dbg_print_num(cmd.param7,10);
		 print_util_dbg_print(", command id:");
		 print_util_dbg_print_num(cmd.command,10);
		 print_util_dbg_print(", confirmation:");
		 print_util_dbg_print_num(cmd.confirmation,10);
		 print_util_dbg_print("\n");
		
		if (cmd.command >= 0 && cmd.command < MAV_CMD_ENUM_END)
		{
			// The command has valid command ID 
			if(	(cmd.target_system == mavlink_system.sysid)||(cmd.target_system == 255) ) //TODO: modfiy to MAV_SYS_ID_ALL when QGroundControl modified
			{
				// The command is for this system
				for (uint32_t i = 0; i < message_handler->cmd_callback_set->callback_count; ++i)
				{
					if ( match_cmd(&message_handler->cmd_callback_set->callback_list[i], msg, &cmd) )
					{
						mavlink_cmd_callback_function_t function 		= message_handler->cmd_callback_set->callback_list[i].function;
						handling_module_struct_t 		module_struct 	= message_handler->cmd_callback_set->callback_list[i].module_struct;
						
						// Call appropriate function callback
						function(module_struct, &cmd);
					}
				}
			}
		}
	}
	else if ( msg->msgid >= 0 && msg->msgid < MAV_MSG_ENUM_END )
	{
		// The message has a valid message ID, and is not a command
		for (uint32_t i = 0; i < message_handler->msg_callback_set->callback_count; ++i)
		{
			if ( match_msg(&message_handler->msg_callback_set->callback_list[i], msg) )
			{
				mavlink_msg_callback_function_t function 		= message_handler->msg_callback_set->callback_list[i].function;
				handling_module_struct_t 		module_struct 	= message_handler->msg_callback_set->callback_list[i].module_struct;
				
				// Call appropriate function callback
				function(module_struct, msg);
			}
		}
	}
}


// ################
// TODO : add this (move to mavlink_communication ?)
// ##############
// 		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM: 
// 		{ // 66
// 			volatile mavlink_request_data_stream_t request;
// 			mavlink_msg_request_data_stream_decode(&rec->msg, (mavlink_request_data_stream_t*) &request);
// 			// TODO: control target_component == compid!
// 			if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid)
// 			//&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid)
// 			{
// 				print_util_dbg_print("stream request:");
// 				print_util_dbg_print_num(request.target_component,10);
// 				if (request.req_stream_id==255) 
// 				{
// 					int32_t i;
// 					print_util_dbg_print("send all\n");
// 					// send full list of streams
// 					for (i = 0; i < mavlink_task_set.number_of_tasks; i++) 
// 					{
// 						task_entry_t *task=scheduler_get_task_by_index(&mavlink_task_set, i);
// 						scheduler_run_task_now(task);
// 					}					
// 				} 
// 				else 
// 				{
// 					task_entry_t *task =scheduler_get_task_by_id(&mavlink_task_set, request.req_stream_id);
// 					print_util_dbg_print(" stream="); print_util_dbg_print_num(request.req_stream_id, 10);
// 					print_util_dbg_print(" start_stop=");print_util_dbg_print_num(request.start_stop, 10);
// 					print_util_dbg_print(" rate=");print_util_dbg_print_num(request.req_message_rate,10);
// 					print_util_dbg_print("\n");
// 					print_util_dbg_print("\n");
// 					if (request.start_stop) 
// 					{
// 						scheduler_change_run_mode(task, RUN_REGULAR);
// 					}
// 					else 
// 					{
// 						scheduler_change_run_mode(task, RUN_NEVER);
// 					}
// 					if (request.req_message_rate>0) 
// 					{
// 						scheduler_change_task_period(task, SCHEDULER_TIMEBASE / (uint32_t)request.req_message_rate);
// 					}
// 				}
// 			}
// 		}	
// 		break;
// 	}
// }

// // handle all platform-specific messages in mavlink-actions:
// mavlink_actions_handle_specific_messages(rec);