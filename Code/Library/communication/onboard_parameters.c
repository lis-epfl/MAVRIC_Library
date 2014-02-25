/*
 * onboard_parameters.c
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#include "onboard_parameters.h"
#include "stabilisation.h"
#include "flashc.h"
#include "print_util.h"

Parameter_Set_t param_set;

void init_onboard_parameters(void) {
	param_set.param_count = 0;
	param_set.enumerate=false;
	param_set.transmit_parameter_index=0;
	
}

void add_parameter_uint8(uint8_t* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type= MAV_PARAM_TYPE_UINT8;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void add_parameter_uint32(uint32_t* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type= MAV_PARAM_TYPE_UINT32;
    param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void add_parameter_int32(int32_t* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAV_PARAM_TYPE_INT32;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void add_parameter_float(float* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAV_PARAM_TYPE_REAL32;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.parameters[param_set.param_count].schedule_for_transmission=true;
	param_set.param_count++;
}

void send_all_parameters() {
	// schedule all parameters for transmission
	for (uint8_t i = 0; i < param_set.param_count; i++) {
		param_set.parameters[i].schedule_for_transmission=true;
	}		
}

void send_all_parameters_now() {
	for (uint8_t i = 0; i < param_set.param_count; i++) {
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(int8_t*)param_set.parameters[i].param_name,
										*param_set.parameters[i].param,
										param_set.parameters[i].data_type,
										param_set.param_count,
										i);
		param_set.parameters[i].schedule_for_transmission=false;

	}

}


void send_scheduled_parameters() {
	for (uint8_t i = 0; i < param_set.param_count; i++) {
		if (param_set.parameters[i].schedule_for_transmission) {
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(int8_t*)param_set.parameters[i].param_name,
										*param_set.parameters[i].param,
										param_set.parameters[i].data_type,
										param_set.param_count,
										i);
			param_set.parameters[i].schedule_for_transmission=false;
			return;
		}			

	}
}


void send_parameter(mavlink_param_request_read_t* request) {
	if(request->param_index!=-1) {
		/*
		mavlink_msg_param_value_send(MAVLINK_COMM_0,
									(int8_t*)param_set.parameters[request->param_index].param_name,
									*param_set.parameters[request->param_index].param,
									param_set.parameters[request->param_index].data_type,
									param_set.param_count,
									request->param_index);*/
		if (request->param_index>param_set.param_count) return;
		param_set.parameters[request->param_index].schedule_for_transmission=true;

	}
	else {
		char* key = (char*) request->param_id;		
		for (uint16_t i = 0; i < param_set.param_count; i++) {
			bool match = true;
			for (uint16_t j = 0; j < param_set.parameters[i].param_name_length; j++) {
				// Compare
				if ((char)param_set.parameters[i].param_name[j] != (char)key[j]) {
					match = false;
				}
 
				// End matching if null termination is reached
				if (((char)param_set.parameters[i].param_name[j]) == '\0') {
					break;
				}
			}
 
			// Check if matched
			if (match) {
				/*
				mavlink_msg_param_value_send(MAVLINK_COMM_0,
											(int8_t*)param_set.parameters[i].param_name,
											*param_set.parameters[i].param, param_set.parameters[i].data_type, 
											param_set.param_count, i);*/
				param_set.parameters[i].schedule_for_transmission=true;

				break;
			}					
		}
	}
}

void receive_parameter(Mavlink_Received_t* rec) {
	mavlink_param_set_t set;
	mavlink_msg_param_set_decode(&rec->msg, &set);
 
	// Check if this message is for this system and subsystem
	if ((uint8_t)set.target_system == (uint8_t)mavlink_system.sysid
	&& (uint8_t)set.target_component == (uint8_t)mavlink_system.compid) {
		char* key = (char*) set.param_id;
				
		for (uint16_t i = 0; i < param_set.param_count; i++) {
			bool match = true;
			for (uint16_t j = 0; j < param_set.parameters[i].param_name_length; j++) {
				// Compare
				if ((char)param_set.parameters[i].param_name[j] != (char)key[j]) {
					match = false;
				}
		
				// End matching if null termination is reached
				if (((char)param_set.parameters[i].param_name[j]) == '\0') {
					break;
				}
			}
 
			// Check if matched
			if (match) {
				// Only write and emit changes if there is actually a difference
				if (*param_set.parameters[i].param != set.param_value && set.param_type == param_set.parameters[i].data_type) {
					*param_set.parameters[i].param = set.param_value;
					// Report back new value
//					mavlink_msg_param_value_send(MAVLINK_COMM_0,
//												(int8_t*)param_set.parameters[i].param_name,
//												*param_set.parameters[i].param, param_set.parameters[i].data_type, 
//												param_set.param_count, i);
					// schedule parameter for transmission downstream
					param_set.parameters[i].schedule_for_transmission=true;
				}
				break;
			}
		}
	}
}

void read_parameters_from_flashc()
{
	uint8_t i;
	nvram_array = AVR32_FLASHC_USER_PAGE_ADDRESS + 0x04;
	
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
			*param_set.parameters[i-1].param = local_array.values[i];
		}
	}else{
		dbg_print("Flash memory corrupted! Hardcoded values taken.\n");
	}
}

void write_parameters_to_flashc()
{
	float cksum1, cksum2;
	cksum1 = 0;
	cksum2 = 0;

	uint8_t i;
	nvram_array = AVR32_FLASHC_USER_PAGE_ADDRESS + 0x04;
	
	nvram_data_ttt local_array;
	
	local_array.values[0] = param_set.param_count;
	cksum1 += local_array.values[0];
	cksum2 += cksum1;
	
	dbg_print("Begin write to flashc...\n");
	
	for (i=1;i<(param_set.param_count+1);i++)
	{
		//flashc_memcpy((void *)&(nvram_array->values[i]),   param_set.parameters[i].param, sizeof((nvram_array->values[i])),   true);
		local_array.values[i] = *param_set.parameters[i-1].param;
		cksum1 += local_array.values[i];
		cksum2 += cksum1;
	}
	local_array.values[param_set.param_count+1] = cksum1;
	local_array.values[param_set.param_count+2] = cksum2;
	
	flashc_memcpy((void *)nvram_array, &local_array, sizeof(*nvram_array) ,   true);
	dbg_print("Write to flashc completed.\n");
}