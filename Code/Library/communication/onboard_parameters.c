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
#include "stabilisation.h"
#include "print_util.h"

#ifdef __cplusplus
	extern "C" {
#endif
	#include "flashc.h"
#ifdef __cplusplus
	}
#endif

Parameter_Set_t param_set;

void onboard_parameters_init(void) 
{
	param_set.param_count = 0;
	dbg_print("Onboard parameters initialised.\n");	
}

void onboard_parameters_add_parameter_uint8(uint8_t* val, const char* param_name) 
{
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type= MAV_PARAM_TYPE_INT8;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void onboard_parameters_add_parameter_uint32(uint32_t* val, const char* param_name) 
{
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type= MAV_PARAM_TYPE_UINT32;
    param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void onboard_parameters_add_parameter_int32(int32_t* val, const char* param_name) 
{
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAV_PARAM_TYPE_INT32;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void onboard_parameters_add_parameter_float(float* val, const char* param_name) 
{
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAV_PARAM_TYPE_REAL32;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}


void onboard_parameters_update_parameter(int param_index, float value) 
{
	float converted=0;
	
	switch (param_set.parameters[param_index].data_type) 
	{
		case MAVLINK_TYPE_CHAR:
		case MAVLINK_TYPE_UINT8_T:
		case MAVLINK_TYPE_INT8_T:
			// take care of different Endianness (usually MAVLINK does this, but here MAVLINK assumes all parameters are 4-byte so we need to swap it back)
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4(&converted, &value);
				memcpy(param_set.parameters[param_index].param, &converted, 1);
			#else
				memcpy(param_set.parameters[param_index].param, &value, 1);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT16_T:
		case MAVLINK_TYPE_INT16_T:
			// take care of different Endianness (usually MAVLINK does this, but here MAVLINK assumes all parameters are 4-byte so we need to swap it back)
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4(&converted, &value);
				memcpy(param_set.parameters[param_index].param, &converted, 2);
			#else
				memcpy(param_set.parameters[param_index].param, &value, 2);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT32_T:
		case MAVLINK_TYPE_INT32_T:
		case MAVLINK_TYPE_FLOAT:
			*param_set.parameters[param_index].param= value;
			break;
		
		// these following types are not supported
		case MAVLINK_TYPE_UINT64_T:
		case MAVLINK_TYPE_INT64_T:
		case MAVLINK_TYPE_DOUBLE:
			break;
	}
}

float onboard_parameters_read_parameter(int param_index) 
{
	float return_value=0;
	float converted=0;
	
	switch (param_set.parameters[param_index].data_type) 
	{
		case MAVLINK_TYPE_CHAR:
		case MAVLINK_TYPE_UINT8_T:
		case MAVLINK_TYPE_INT8_T:
			memcpy(&return_value, param_set.parameters[param_index].param, 1);
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4(&converted, &return_value);
			#else
				memcpy(&converted, &return_value, 4);
			#endif
			break;
		
		case MAVLINK_TYPE_UINT16_T:
		case MAVLINK_TYPE_INT16_T:
			memcpy(&return_value, param_set.parameters[param_index].param, 2);
			#if MAVLINK_NEED_BYTE_SWAP
				byte_swap_4(&converted, &return_value);
			#else
				memcpy(&converted, &return_value, 4);
			#endif
			break;
			
		case MAVLINK_TYPE_UINT32_T:
		case MAVLINK_TYPE_INT32_T:
		case MAVLINK_TYPE_FLOAT:
			converted= *param_set.parameters[param_index].param;
			break;
		
		// these following types are not supported
		case MAVLINK_TYPE_UINT64_T:
		case MAVLINK_TYPE_INT64_T:
		case MAVLINK_TYPE_DOUBLE:
			break;
	}	
	return converted;
}



void onboard_parameters_send_all_parameters() 
{
	// schedule all parameters for transmission
	for (uint8_t i = 0; i < param_set.param_count; i++)
	{
		param_set.parameters[i].schedule_for_transmission=true;
	}		
}

void onboard_parameters_send_all_parameters_now() 
{
	for (uint8_t i = 0; i < param_set.param_count; i++)
	{
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(int8_t*)param_set.parameters[i].param_name,
										onboard_parameters_read_parameter(i),
										param_set.parameters[i].data_type,
										param_set.param_count,
										i);
										
		param_set.parameters[i].schedule_for_transmission=false;
	}
}


void onboard_parameters_send_scheduled_parameters() 
{
	for (uint8_t i = 0; i < param_set.param_count; i++)
	{
		if (param_set.parameters[i].schedule_for_transmission) 
		{
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(int8_t*)param_set.parameters[i].param_name,
										onboard_parameters_read_parameter(i),
										param_set.parameters[i].data_type,
										param_set.param_count,
										i);
										
			param_set.parameters[i].schedule_for_transmission=false;
		}			
	}
}

void onboard_parameters_send_parameter(mavlink_param_request_read_t* request) 
{
	// Check param_index to determine if the request is made by name (== -1) or by index (!= -1)
	if(request->param_index!=-1) 
	{	
		// Control if the index is in the range of existing parameters and schedule it for transmission
		if ((request->param_index<=param_set.param_count) == true)
		{
			param_set.parameters[request->param_index].schedule_for_transmission=true;
		}
	}
	else 
	{
		char* key = (char*) request->param_id;		
		for (uint16_t i = 0; i < param_set.param_count; i++) 
		{
			bool match = true;
			for (uint16_t j = 0; j < param_set.parameters[i].param_name_length; j++) 
			{
				// Compare
				if ((char)param_set.parameters[i].param_name[j] != (char)key[j]) 
				{
					match = false;
				}
 
				// End matching if null termination is reached
				if (((char)param_set.parameters[i].param_name[j]) == '\0') 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if (match) 
			{
				param_set.parameters[i].schedule_for_transmission=true;

				// Exit external (i) for() loop
				break;
			}					
		}
	}
}

void onboard_parameters_receive_parameter(Mavlink_Received_t* rec) 
{
	bool match = true;
	
	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(&rec->msg, &set);
 
	// Check if this message is for this system and subsystem
	if ((uint8_t)set.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)set.target_component == (uint8_t)mavlink_system.compid) 
	{
		dbg_print("Setting parameter ");
		dbg_print(set.param_id);
		dbg_print(" to ");
		dbg_putfloat(set.param_value, 2);
		dbg_print("\n");
		
		char* key = (char*) set.param_id;
				
		for (uint16_t i = 0; i < param_set.param_count; i++) 
		{
			match = true;
			for (uint16_t j = 0; j < param_set.parameters[i].param_name_length; j++) 
			{
				// Compare
				if ((char)param_set.parameters[i].param_name[j] != (char)key[j]) 
				{
					match = false;
				}
		
				// End matching if null termination is reached
				if (((char)param_set.parameters[i].param_name[j]) == '\0') 
				{
					// Exit internal (j) for() loop
					break;
				}
			}
 
			// Check if matched
			if (match) 
			{
				// Only write and emit changes if there is actually a difference
				if (*param_set.parameters[i].param != set.param_value && set.param_type == param_set.parameters[i].data_type) 
				{
					onboard_parameters_update_parameter(i, set.param_value);

					// schedule parameter for transmission downstream
					param_set.parameters[i].schedule_for_transmission=true;
				}
				break;
			}
		}
	}
}

void onboard_parameters_read_parameters_from_flashc()
{
	uint8_t i;
	
	nvram_array = MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_ttt local_array;
	
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;

	for (i=0;i<(param_set.param_count+1);i++)
	{
		local_array.values[i] = nvram_array->values[i];
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	
	if ((param_set.param_count==local_array.values[0])&&(cksum1 == nvram_array->values[param_set.param_count+1])&&(cksum2 == nvram_array->values[param_set.param_count+2]))
	{
		dbg_print("Flash read successful! New Parameters inserted. \n");
		for (i=1;i<(param_set.param_count+1);i++)
		{
			onboard_parameters_update_parameter(i-1, local_array.values[i]);
		}
	}
	else
	{
		dbg_print("Flash memory corrupted! Hardcoded values taken.\n");
	}
}

void onboard_parameters_write_parameters_from_flashc()
{
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;
	uint8_t i;
	size_t bytes_to_write = 0;
	
	nvram_array = MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;
	nvram_data_ttt local_array;
	
	local_array.values[0] = param_set.param_count;
	cksum1 += local_array.values[0];
	cksum2 += cksum1;
	
	dbg_print("Begin write to flashc...\n");
	
	for (i=1;i<=param_set.param_count;i++)
	{
		local_array.values[i] = onboard_parameters_read_parameter(i-1);
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	local_array.values[param_set.param_count+1] = cksum1;
	local_array.values[param_set.param_count+2] = cksum2;

	bytes_to_write = 4 * (param_set.param_count + 3);	// (1 param_count + parameters + 2 checksums) floats
	
	if(bytes_to_write < MAVERIC_FLASHC_USER_PAGE_FREE_SPACE)
	{
		flashc_memcpy((void *)nvram_array, &local_array, bytes_to_write, true);
		dbg_print("Write to flashc completed.\n");
	}
	else
	{
		dbg_print("Attempted to write too many parameters on flash user page, aborted.\n");
	}
}