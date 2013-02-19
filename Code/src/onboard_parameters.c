/*
 * onboard_parameters.c
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#include "onboard_parameters.h"
#include "stabilisation.h"

Onboard_Parameters_t params;

void init_onboard_parameters(void) {
	params.param_count = 0;
	params.param_name_length = 0;
	add_PID_parameters();
}

void add_parameter_uint32(uint32_t* val, const char* param_name) {
	params.param[params.param_count] = val;
	strcpy(params.param_name[params.param_count], param_name);
	params.data_types[params.param_count] = MAVLINK_TYPE_UINT32_T;
	if(params.param_name_length < strlen(param_name)) {
		params.param_name_length = strlen(param_name);
	}
	params.param_count++;
}

void add_parameter_int32(int32_t* val, const char* param_name) {
	params.param[params.param_count] = val;
	strcpy(params.param_name[params.param_count], param_name);
	params.data_types[params.param_count] = MAVLINK_TYPE_INT32_T;
	if(params.param_name_length < strlen(param_name)) {
		params.param_name_length = strlen(param_name);
	}
	params.param_count++;
}

void add_parameter_float(float* val, const char* param_name) {
	params.param[params.param_count] = val;
	strcpy(params.param_name[params.param_count], param_name);
	params.data_types[params.param_count] = MAVLINK_TYPE_FLOAT;
	if(params.param_name_length < strlen(param_name)) {
		params.param_name_length = strlen(param_name);
	}
	params.param_count++;
}

void add_PID_parameters(void) {
	Stabiliser_t* rate_stabiliser = get_rate_stabiliser();
	Stabiliser_t* attitude_stabiliser = get_attitude_stabiliser();
	// Roll rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain, "RollRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].clip_max, "RollRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].clip_min, "RollRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.clip, "RollRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollRPid_D_LPF");
	
	// Roll attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain, "RollAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].clip_max, "RollAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].clip_min, "RollAPid_P_CLmn");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.clip, "RollAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollAPid_D_LPF");

	// Pitch rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain, "PitchRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].clip_max, "PitchRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].clip_min, "PitchRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.clip, "PitchRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.clip, "PitchRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchRPid_D_LPF");
	
	// Pitch attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain, "PitchAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].clip_max, "PitchAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].clip_min, "PitchAPid_P_CLmn");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.clip, "PitchAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.clip, "PitchAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchAPid_D_LPF");	

	// Yaw rate PID
/*	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].p_gain, "YawRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_max, "YawRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_min, "YawRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.clip, "YawRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.postgain, "YawRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.pregain, "YawRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.clip, "YawRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.gain, "YawRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawRPid_D_LPF");
	
	// Yaw attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].p_gain, "YawAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_max, "YawAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_min, "YawAPid_P_CLmn");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.clip, "YawAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.postgain, "YawAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.pregain, "YawAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.clip, "YawAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain, "YawAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawAPid_D_LPF");	
*/
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
		for (uint8_t i = 0; i < params.param_count; i++) {
			mavlink_msg_param_value_send(MAVLINK_COMM_0,
											(int8_t*)params.param_name[i],
											*params.param[i],
											params.data_types[i],
											params.param_count,
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
										(int8_t*)params.param_name[request.param_index],
										*params.param[request.param_index],
										params.data_types[request.param_index],
										params.param_count,
										request.param_index);
		}
		else {
			char* key = (char*) request.param_id;		
			for (uint16_t i = 0; i < params.param_count; i++) {
				bool match = true;
				for (uint16_t j = 0; j < params.param_name_length; j++) {
					// Compare
					if ((char)params.param_name[i][j] != (char)key[j]) {
						match = false;
					}
 
					// End matching if null termination is reached
					if (((char)params.param_name[i][j]) == '\0') {
						break;
					}
				}
 
				// Check if matched
				if (match) {
					mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)params.param_name[i],
												*params.param[i], params.data_types[i], 
												params.param_count, i);
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
				
		for (uint16_t i = 0; i < params.param_count; i++) {
			bool match = true;
			for (uint16_t j = 0; j < params.param_name_length; j++) {
				// Compare
				if ((char)params.param_name[i][j] != (char)key[j]) {
					match = false;
				}
 
				// End matching if null termination is reached
				if (((char)params.param_name[i][j]) == '\0') {
					break;
				}
			}
 
			// Check if matched
			if (match) {
				// Only write and emit changes if there is actually a difference
				if (*params.param[i] != set.param_value && set.param_type == params.data_types[i]) {
					*params.param[i] = set.param_value;
					// Report back new value
					mavlink_msg_param_value_send(MAVLINK_COMM_0,
												(int8_t*)params.param_name[i],
												*params.param[i], params.data_types[i], 
												params.param_count, i);
				}
				break;
			}
		}
	}
}