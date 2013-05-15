/*
 * onboard_parameters.c
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#include "onboard_parameters.h"
#include "stabilisation.h"

Parameter_Set_t param_set;

void init_onboard_parameters(void) {
	param_set.param_count = 0;
	param_set.enumerate=false;
	param_set.transmit_parameter_index=0;
	
}

void add_parameter_uint32(uint32_t* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type= MAVLINK_TYPE_UINT32_T;
    param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.param_count++;
}

void add_parameter_int32(int32_t* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAVLINK_TYPE_INT32_T;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.param_count++;
}

void add_parameter_float(float* val, const char* param_name) {
	param_set.parameters[param_set.param_count].param = val;
	strcpy(param_set.parameters[param_set.param_count].param_name, param_name);
	param_set.parameters[param_set.param_count].data_type = MAVLINK_TYPE_FLOAT;
	param_set.parameters[param_set.param_count].param_name_length = strlen(param_name);
	param_set.param_count++;

}


void send_all_parameters(Mavlink_Received_t* rec) {
	/* 
	TODO : if the number of parameters becomes large, 
	it would be better to send the values at a lower rate to reduce bandwidth footprint
	*/
	mavlink_param_request_list_t request;
	mavlink_msg_param_request_list_decode(&rec->msg, &request);
	// Check if this message is for this system
	if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid) {
		for (uint8_t i = 0; i < param_set.param_count; i++) {
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
											(int8_t*)param_set.parameters[i].param_name,
											*param_set.parameters[i].param,
											param_set.parameters[i].data_type,
											param_set.param_count,
											i);
		}
	}		
}

void send_parameter(Mavlink_Received_t* rec) {
	mavlink_param_request_read_t request;
	mavlink_msg_param_request_read_decode(&rec->msg, &request);
	// Check if this message is for this system and subsystem
	if ((uint8_t)request.target_system == (uint8_t)mavlink_system.sysid
		&& (uint8_t)request.target_component == (uint8_t)mavlink_system.compid) {
		if(request.param_index!=-1) {
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
										(int8_t*)param_set.parameters[request.param_index].param_name,
										*param_set.parameters[request.param_index].param,
										param_set.parameters[request.param_index].data_type,
										param_set.param_count,
										request.param_index);
		}
		else {
			char* key = (char*) request.param_id;		
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
					mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)param_set.parameters[i].param_name,
												*param_set.parameters[i].param, param_set.parameters[i].data_type, 
												param_set.param_count, i);
					break;
				}					
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
					mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)param_set.parameters[i].param_name,
												*param_set.parameters[i].param, param_set.parameters[i].data_type, 
												param_set.param_count, i);
				}
				break;
			}
		}
	}
}