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


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Sends all parameters that have been scheduled via MAVlink
 */
static task_return_t onboard_parameters_send_scheduled_parameters(onboard_parameter_set_t* onboard_parameters);


/**
 * \brief	Callback to a MAVlink parameter request
 *
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_send_parameter(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg);


/**
 * \brief	Callback to a MAVlink parameter set
 *
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_receive_parameter(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg);


/**
 * \brief	Marks all parameters to be scheduled for transmission
 * 
 * \param   onboard_parameters		Pointer to module structure
 * \param   msg 					Incoming mavlink message
 */
static void onboard_parameters_send_all_parameters(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg);


/**
 * \brief		Reads linked memory location and returns parameter value, with care of necessary size/type conversions
 * 
 * \details 	This method takes care of necessary size/type conversions.
 *
 * \param 		param_index		Set index of the parameter to update
 *
 * \return						Value of the parameter read. Note that the parameter might not be a float, but float is the default data type for the MAVlink message.
 */
static float onboard_parameters_read_parameter(onboard_parameter_set_t* onboard_parameters, int32_t param_index);


/**
 * \brief	Updates linked memory location of a parameter with given value, with care of necessary size/type conversions
 *
 * \param 	param_index		Set index of the parameter to update
 * \param 	value			Value of the parameter to update
 */
static void onboard_parameters_update_parameter(onboard_parameter_set_t* onboard_parameters, int32_t param_index, float value);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static task_return_t onboard_parameters_send_scheduled_parameters(onboard_parameter_set_t* onboard_parameters) 
{
	for (uint8_t i = 0; i < onboard_parameters->param_count; i++)
	{
		if (onboard_parameters->parameters[i].schedule_for_transmission) 
		{
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(char*)onboard_parameters->parameters[i].param_name,
										onboard_parameters_read_parameter(onboard_parameters, i),
										onboard_parameters->parameters[i].data_type,
										onboard_parameters->param_count,
										i);
										
			onboard_parameters->parameters[i].schedule_for_transmission=false;
		}			
	}
	
	return TASK_RUN_SUCCESS;
}


static void onboard_parameters_send_all_parameters(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg) 
{	
	// schedule all parameters for transmission
	for (uint8_t i = 0; i < onboard_parameters->param_count; i++)
	{
		onboard_parameters->parameters[i].schedule_for_transmission=true;
	}		
}


static void onboard_parameters_send_parameter(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg) 
{
	mavlink_param_request_read_t request;
	mavlink_msg_param_request_read_decode(msg, &request);

	// Check param_index to determine if the request is made by name (== -1) or by index (!= -1)
	if(request.param_index != -1) 
	{	
		// Control if the index is in the range of existing parameters and schedule it for transmission
		if ( request.param_index <= onboard_parameters->param_count)
		{
			onboard_parameters->parameters[request.param_index].schedule_for_transmission = true;
		}
	}
	else 
	{
		char* key = (char*) request.param_id;
		for (uint16_t i = 0; i < onboard_parameters->param_count; i++) 
		{
			bool match = true;
			for (uint16_t j = 0; j < onboard_parameters->parameters[i].param_name_length; j++) 
			{
				// Compare
				if ((char)onboard_parameters->parameters[i].param_name[j] != (char)key[j]) 
				{
					match = false;
				}
 
				// End matching if null termination is reached
				if (((char)onboard_parameters->parameters[i].param_name[j]) == '\0') 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if (match) 
			{
				onboard_parameters->parameters[i].schedule_for_transmission = true;

				// Exit external (i) for() loop
				break;
			}					
		}
	}
}


static float onboard_parameters_read_parameter(onboard_parameter_set_t* onboard_parameters, int32_t param_index) 
{
	float return_value=0;
	float converted=0;
	
	switch (onboard_parameters->parameters[param_index].data_type) 
	{
		case MAVLINK_TYPE_CHAR:
		case MAVLINK_TYPE_UINT8_T:
		case MAVLINK_TYPE_INT8_T:
			memcpy(&return_value, onboard_parameters->parameters[param_index].param, 1);
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4((char *)&converted, (char *)&return_value);
			#else
				memcpy(&converted, &return_value, 4);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT16_T:
		case MAVLINK_TYPE_INT16_T:
			memcpy(&return_value, onboard_parameters->parameters[param_index].param, 2);
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4((char *)&converted, (char *)&return_value);
			#else
				memcpy(&converted, &return_value, 4);
			#endif
			break;
			
		case MAVLINK_TYPE_UINT32_T:
		case MAVLINK_TYPE_INT32_T:
		case MAVLINK_TYPE_FLOAT:
			converted= *onboard_parameters->parameters[param_index].param;
			break;
		
		// these following types are not supported
		case MAVLINK_TYPE_UINT64_T:
		case MAVLINK_TYPE_INT64_T:
		case MAVLINK_TYPE_DOUBLE:
			break;
	}	
	return converted;
}


static void onboard_parameters_update_parameter(onboard_parameter_set_t* onboard_parameters, int32_t param_index, float value) 
{
	float converted=0;
	
	switch (onboard_parameters->parameters[param_index].data_type) 
	{
		case MAVLINK_TYPE_CHAR:
		case MAVLINK_TYPE_UINT8_T:
		case MAVLINK_TYPE_INT8_T:
			// take care of different Endianness (usually MAVLINK does this, but here MAVLINK assumes all parameters are 4-byte so we need to swap it back)
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4((char *)&converted, (char *)&value);
				memcpy(onboard_parameters->parameters[param_index].param, &converted, 1);
			#else
				memcpy(onboard_parameters->parameters[param_index].param, &value, 1);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT16_T:
		case MAVLINK_TYPE_INT16_T:
			// take care of different Endianness (usually MAVLINK does this, but here MAVLINK assumes all parameters are 4-byte so we need to swap it back)
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4((char *)&converted, (char *)&value);
				memcpy(onboard_parameters->parameters[param_index].param, &converted, 2);
			#else
				memcpy(onboard_parameters->parameters[param_index].param, &value, 2);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT32_T:
		case MAVLINK_TYPE_INT32_T:
		case MAVLINK_TYPE_FLOAT:
			*onboard_parameters->parameters[param_index].param= value;
			break;
		
		// these following types are not supported
		case MAVLINK_TYPE_UINT64_T:
		case MAVLINK_TYPE_INT64_T:
		case MAVLINK_TYPE_DOUBLE:
			break;
	}
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void onboard_parameters_init(onboard_parameter_set_t* onboard_parameters, task_set_t* task_set, mavlink_message_handler_t* message_handler) 
{
	// Init param_count
	onboard_parameters->param_count = 0;
	
	// Add onboard parameter telemetry to the scheduler
	scheduler_add_task(	task_set, 
						100000, 
						RUN_REGULAR, 
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

	print_util_dbg_print("Onboard parameters initialised.\n");	
}


void onboard_parameters_add_parameter_uint32(onboard_parameter_set_t* onboard_parameters, uint32_t* val, const char* param_name) 
{
	onboard_parameters->parameters[onboard_parameters->param_count].param                     = (float*) val;
	strcpy(onboard_parameters->parameters[onboard_parameters->param_count].param_name, param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].data_type                 = MAV_PARAM_TYPE_UINT32;
	onboard_parameters->parameters[onboard_parameters->param_count].param_name_length         = strlen(param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].schedule_for_transmission = true;
	onboard_parameters->param_count++;
}


void onboard_parameters_add_parameter_int32(onboard_parameter_set_t* onboard_parameters, int32_t* val, const char* param_name) 
{
	onboard_parameters->parameters[onboard_parameters->param_count].param                     = (float*) val;
	strcpy(onboard_parameters->parameters[onboard_parameters->param_count].param_name, param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].data_type                 = MAV_PARAM_TYPE_INT32;
	onboard_parameters->parameters[onboard_parameters->param_count].param_name_length         = strlen(param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].schedule_for_transmission = true;
	onboard_parameters->param_count++;
}


void onboard_parameters_add_parameter_float(onboard_parameter_set_t* onboard_parameters, float* val, const char* param_name) 
{
	onboard_parameters->parameters[onboard_parameters->param_count].param                     = val;
	strcpy(onboard_parameters->parameters[onboard_parameters->param_count].param_name, param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].data_type                 = MAV_PARAM_TYPE_REAL32;
	onboard_parameters->parameters[onboard_parameters->param_count].param_name_length         = strlen(param_name);
	onboard_parameters->parameters[onboard_parameters->param_count].schedule_for_transmission = true;
	onboard_parameters->param_count++;
}


void onboard_parameters_receive_parameter(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg) 
{
	bool match = true;
	
	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(msg, &set);
 
	// Check if this message is for this system and subsystem
	if ( 	set.target_system 	 == mavlink_system.sysid 
		 &&	set.target_component == mavlink_system.compid	)
	{
		print_util_dbg_print("Setting parameter ");
		print_util_dbg_print(set.param_id);
		print_util_dbg_print(" to ");
		print_util_dbg_putfloat(set.param_value, 2);
		print_util_dbg_print("\n");
		
		char* key = (char*) set.param_id;
				
		for (uint16_t i = 0; i < onboard_parameters->param_count; i++) 
		{
			match = true;
			for (uint16_t j = 0; j < onboard_parameters->parameters[i].param_name_length; j++) 
			{
				// Compare
				if ((char)onboard_parameters->parameters[i].param_name[j] != (char)key[j]) 
				{
					match = false;
				}
		
				// End matching if null termination is reached
				if (((char)onboard_parameters->parameters[i].param_name[j]) == '\0') 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if (match) 
			{
				// Only write and emit changes if there is actually a difference
				if (*onboard_parameters->parameters[i].param != set.param_value && set.param_type == onboard_parameters->parameters[i].data_type) 
				{
					onboard_parameters_update_parameter(onboard_parameters, i, set.param_value);

					// schedule parameter for transmission downstream
					onboard_parameters->parameters[i].schedule_for_transmission=true;
				}
				break;
			}
		}
	}
}


void onboard_parameters_read_parameters_from_flashc(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg)
{
	uint8_t i;
	
	nvram_data_t* nvram_array = (nvram_data_t *) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_t local_array;
	
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;

	for (i=0;i<(onboard_parameters->param_count +1);i++)
	{
		local_array.values[i] = nvram_array->values[i];
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	
	if ((onboard_parameters->param_count==local_array.values[0])&&(cksum1 == nvram_array->values[onboard_parameters->param_count + 1])&&(cksum2 == nvram_array->values[onboard_parameters->param_count + 2]))
	{
		print_util_dbg_print("Flash read successful! New Parameters inserted. \n");
		for (i=1;i<(onboard_parameters->param_count + 1);i++)
		{
			onboard_parameters_update_parameter(onboard_parameters, i-1, local_array.values[i]);
		}
	}
	else
	{
		print_util_dbg_print("Flash memory corrupted! Hardcoded values taken.\n");
	}
}


void onboard_parameters_write_parameters_from_flashc(onboard_parameter_set_t* onboard_parameters, mavlink_message_t* msg)
{
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;
	uint8_t i;
	size_t bytes_to_write = 0;
	
	nvram_data_t* nvram_array = (nvram_data_t*) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_t local_array;
	
	local_array.values[0] = onboard_parameters->param_count;
	cksum1 += local_array.values[0];
	cksum2 += cksum1;
	
	print_util_dbg_print("Begin write to flashc...\n");
	
	for (i=1;i<=onboard_parameters->param_count;i++)
	{
		local_array.values[i] = onboard_parameters_read_parameter(onboard_parameters, i-1);
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	local_array.values[onboard_parameters->param_count + 1] = cksum1;
	local_array.values[onboard_parameters->param_count + 2] = cksum2;

	bytes_to_write = 4 * (onboard_parameters->param_count + 3);	// (1 param_count + parameters + 2 checksums) floats
	
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