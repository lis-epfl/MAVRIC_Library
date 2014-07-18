/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file onboard_parameters.c
 * 
 * Mav'ric Onboard parameters
 */


#include "onboard_parameters.h"
#include "print_util.h"
#include "flashc.h"
#include "mavlink_communication.h"
#include <stdlib.h>


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Sends all parameters that have been scheduled via MAVlink
 */
static task_return_t onboard_parameters_send_scheduled_parameters(onboard_parameters_t* onboard_parameters);


/**
 * \brief	Marks all parameters to be scheduled for transmission
 * 
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_send_all_parameters(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg);


/**
 * \brief	Callback to a MAVlink parameter request
 *
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_send_parameter(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg);


/**
 * \brief	Callback to a MAVlink parameter set
 *
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_receive_parameter(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static task_return_t onboard_parameters_send_scheduled_parameters(onboard_parameters_t* onboard_parameters) 
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	for (uint8_t i = 0; i < param_set->param_count; i++)
	{
		if (param_set->parameters[i].schedule_for_transmission) 
		{
			mavlink_msg_param_value_send(	MAVLINK_COMM_0,
											(char*)param_set->parameters[i].param_name,
											*(param_set->parameters[i].param),
											// onboard_parameters_read_parameter(onboard_parameters, i),
											param_set->parameters[i].data_type,
											param_set->param_count,
											i 	);
										
			param_set->parameters[i].schedule_for_transmission=false;
		}			
	}
	
	return TASK_RUN_SUCCESS;
}


static void onboard_parameters_send_all_parameters(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg) 
{	
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	// schedule all parameters for transmission
	for (uint8_t i = 0; i < param_set->param_count; i++)
	{
		param_set->parameters[i].schedule_for_transmission=true;
	}		
}


static void onboard_parameters_send_parameter(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg) 
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	mavlink_param_request_read_t request;
	mavlink_msg_param_request_read_decode(msg, &request);

	// Check param_index to determine if the request is made by name (== -1) or by index (!= -1)
	if(request.param_index != -1) 
	{	
		// Control if the index is in the range of existing parameters and schedule it for transmission
		if ( request.param_index <= param_set->param_count)
		{
			param_set->parameters[request.param_index].schedule_for_transmission = true;
		}
	}
	else 
	{
		char* key = (char*) request.param_id;
		for (uint16_t i = 0; i < param_set->param_count; i++) 
		{
			bool match = true;

			// Get pointer to parameter number i
			onboard_parameters_entry_t* param = &param_set->parameters[i];

			for (uint16_t j = 0; j < param->param_name_length; j++) 
			{
				// Compare
				if ( (char)param->param_name[j] != (char)key[j] ) 
				{
					match = false;
				}
 
				// End matching if null termination is reached
				if ( ((char)param->param_name[j]) == '\0' ) 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if ( match ) 
			{
				param->schedule_for_transmission = true;

				// Exit external (i) for() loop
				break;
			}					
		}
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void onboard_parameters_init(onboard_parameters_t* onboard_parameters, const onboard_parameters_conf_t* config, scheduler_t* scheduler, mavlink_message_handler_t* message_handler) 
{
	// Init debug mode
	onboard_parameters->debug = config->debug;

	// Allocate memory for the onboard parameters
	onboard_parameters->param_set = malloc( sizeof(onboard_parameters_set_t) + sizeof(onboard_parameters_entry_t[config->max_param_count]) );
	onboard_parameters->param_set->max_param_count = config->max_param_count;
	onboard_parameters->param_set->param_count = 0;

	// Add onboard parameter telemetry to the scheduler
	scheduler_add_task(	scheduler, 
						100000, 
						RUN_REGULAR, 
						PERIODIC_ABSOLUTE,
						(task_function_t)&onboard_parameters_send_scheduled_parameters, 
						(task_argument_t)onboard_parameters, 
						MAVLINK_MSG_ID_PARAM_VALUE);

	// Add callbacks for onboard parameters requests
	mavlink_message_handler_msg_callback_t callback;

	callback.message_id 	= MAVLINK_MSG_ID_PARAM_REQUEST_LIST;
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&onboard_parameters_send_all_parameters;
	callback.module_struct 	= (handling_module_struct_t)		onboard_parameters;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );
	
	callback.message_id 	= MAVLINK_MSG_ID_PARAM_REQUEST_READ;
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&onboard_parameters_send_parameter;
	callback.module_struct 	= (handling_module_struct_t)		onboard_parameters;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );	
	
	callback.message_id 	= MAVLINK_MSG_ID_PARAM_SET;
	callback.sysid_filter 	= MAVLINK_BASE_STATION_ID;
	callback.compid_filter 	= MAV_COMP_ID_ALL;
	callback.function 		= (mavlink_msg_callback_function_t)	&onboard_parameters_receive_parameter;
	callback.module_struct 	= (handling_module_struct_t)		onboard_parameters;
	mavlink_message_handler_add_msg_callback( message_handler, &callback );

	// Add callbacks for waypoint handler commands requests
	mavlink_message_handler_cmd_callback_t callbackcmd;
	
	callbackcmd.command_id = MAV_CMD_PREFLIGHT_STORAGE; // 20
	callbackcmd.sysid_filter = MAVLINK_BASE_STATION_ID;
	callbackcmd.compid_filter = MAV_COMP_ID_ALL;
	callbackcmd.compid_target = MAV_COMP_ID_ALL;
	callbackcmd.function = (mavlink_cmd_callback_function_t)	&onboard_parameters_preflight_storage;
	callbackcmd.module_struct =									onboard_parameters;
	mavlink_message_handler_add_cmd_callback( message_handler, &callbackcmd);
	

	print_util_dbg_print("Onboard parameters initialised.\n");	
}


void onboard_parameters_add_parameter_uint32(onboard_parameters_t* onboard_parameters, uint32_t* val, const char* param_name) 
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	if( param_set->param_count < param_set->max_param_count )
	{
		onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

		new_param->param                     = (float*) val;
		strcpy( new_param->param_name, 		param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_UINT32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_transmission = true;
		
		param_set->param_count += 1;
	}
	else
	{
		print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param");
	}
}


void onboard_parameters_add_parameter_int32(onboard_parameters_t* onboard_parameters, int32_t* val, const char* param_name) 
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	if( param_set->param_count < param_set->max_param_count )
	{
		onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

		new_param->param                     = (float*) val;
		strcpy( new_param->param_name, 		param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_INT32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_transmission = true;
		
		param_set->param_count += 1;
	}
	else
	{
		print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param");
	}
}


void onboard_parameters_add_parameter_float(onboard_parameters_t* onboard_parameters, float* val, const char* param_name) 
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	if( param_set->param_count < param_set->max_param_count )
	{
		onboard_parameters_entry_t* new_param = &param_set->parameters[param_set->param_count];

		new_param->param                     = val;
		strcpy( new_param->param_name, 		param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_REAL32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_transmission = true;
		
		param_set->param_count += 1;
	}
	else
	{
		print_util_dbg_print("[ONBOARD PARAMETER] Error: Cannot add more param");
	}
}


void onboard_parameters_receive_parameter(onboard_parameters_t* onboard_parameters, mavlink_message_t* msg) 
{
	bool match = true;
	
	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(msg, &set);
 
	// Check if this message is for this system and subsystem
	if ( 	set.target_system 	 == mavlink_system.sysid 
		 &&	set.target_component == mavlink_system.compid	)
	{
		char* key = (char*) set.param_id;
		onboard_parameters_entry_t* param;

		if ( onboard_parameters->debug == true )
		{
			print_util_dbg_print("Setting parameter ");
			print_util_dbg_print(set.param_id);
			print_util_dbg_print(" to ");
			print_util_dbg_putfloat(set.param_value, 2);
			print_util_dbg_print("\n");
		}
				
		for (uint16_t i = 0; i < onboard_parameters->param_set->param_count; i++) 
		{
			param = &onboard_parameters->param_set->parameters[i];
			match = true;

			for (uint16_t j = 0; j < param->param_name_length; j++) 
			{
				// Compare
				if ((char)param->param_name[j] != (char)key[j]) 
				{
					match = false;
				}
		
				// End matching if null termination is reached
				if (((char)param->param_name[j]) == '\0') 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if (match) 
			{
				// Only write and emit changes if there is actually a difference
				if (*(param->param) != set.param_value && set.param_type == param->data_type) 
				{
					// onboard_parameters_update_parameter(onboard_parameters, i, set.param_value);
					(*param->param) = set.param_value;

					// schedule parameter for transmission downstream
					param->schedule_for_transmission=true;
				}
				break;
			}
		}
	}
}

void onboard_parameters_preflight_storage(onboard_parameters_t* onboard_parameters, mavlink_command_long_t* msg)
{
	// Onboard parameters storage
	if (msg->param1 == 0)
	{
	 	// read parameters from flash
	 	print_util_dbg_print("Reading from flashc...\n");
		if(onboard_parameters_read_parameters_from_flashc(onboard_parameters))
		{
			// TODO: update simulation calibration values
			//simulation_calib_set(&sim_model);
	 	}
	}
	else if (msg->param1 == 1)
	{
	 	// write parameters to flash
	 	//print_util_dbg_print("No Writing to flashc\n");
	 	print_util_dbg_print("Writing to flashc\n");
	 	onboard_parameters_write_parameters_to_flashc(onboard_parameters);
	}

	mavlink_msg_command_ack_send(MAVLINK_COMM_0, MAV_CMD_PREFLIGHT_STORAGE, MAV_RESULT_ACCEPTED);
	
	//// Mission parameters storage
	//if (packet.param2 == 0)
	//{
	 	//// read mission from flash
	//}
	//else if (packet.param2 == 1)
	//{
	 	//// write mission to flash
	//}
	//break;
}


bool onboard_parameters_read_parameters_from_flashc(onboard_parameters_t* onboard_parameters)
{
	uint8_t i;
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	nvram_data_t* nvram_array = (nvram_data_t *) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_t local_array;
	
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;

	bool flash_read_successful = false;

	for (i = 0; i < (param_set->param_count +1);i++)
	{
		local_array.values[i] = nvram_array->values[i];
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	
	if ( 	(param_set->param_count == local_array.values[0] )
		&&	(cksum1 == nvram_array->values[param_set->param_count + 1])
		&&	(cksum2 == nvram_array->values[param_set->param_count + 2]) )
	{
		print_util_dbg_print("Flash read successful! New Parameters inserted. \n");
		for (i = 1; i < (param_set->param_count + 1); i++)
		{
			*(param_set->parameters[i-1].param) = local_array.values[i];
			// onboard_parameters_update_parameter(onboard_parameters, i-1, local_array.values[i]);
		}
		flash_read_successful = true;
	}
	else
	{
		print_util_dbg_print("Flash memory corrupted! Hardcoded values taken.\n");
	}
	
	return flash_read_successful;
}


void onboard_parameters_write_parameters_to_flashc(onboard_parameters_t* onboard_parameters)
{
	onboard_parameters_set_t* param_set = onboard_parameters->param_set;

	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;

	uint8_t i;
	size_t bytes_to_write = 0;
	
	nvram_data_t* nvram_array = (nvram_data_t*) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_t local_array;
	
	local_array.values[0] = param_set->param_count;
	cksum1 += local_array.values[0];
	cksum2 += cksum1;
	
	print_util_dbg_print("Begin write to flashc...\n");
	
	for (i = 1; i <= param_set->param_count; i++)
	{
		// local_array.values[i] = onboard_parameters_read_parameter(onboard_parameters, i-1);
		local_array.values[i] = *(param_set->parameters[i-1].param);

		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	local_array.values[param_set->param_count + 1] = cksum1;
	local_array.values[param_set->param_count + 2] = cksum2;

	bytes_to_write = 4 * (param_set->param_count + 3);	// (1 param_count + parameters + 2 checksums) floats
	
	if(bytes_to_write < MAVERIC_FLASHC_USER_PAGE_FREE_SPACE)
	{
		flashc_memcpy((void *)nvram_array, &local_array, bytes_to_write, true);
		print_util_dbg_print("Write to flashc completed.\n");
	}
	else
	{
		print_util_dbg_print("Attempted to write too many parameters on flash user page, aborted.\n");
	}
}