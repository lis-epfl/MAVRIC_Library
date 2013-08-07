/*
 * onboard_parameters.h
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#ifndef ONBOARD_PARAMETERS_H_
#define ONBOARD_PARAMETERS_H_

#include "mavlink_stream.h"

#define MAX_ONBOARD_PARAM_COUNT 60

typedef struct {
	float* param;
	char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
	mavlink_message_type_t data_type;
	uint8_t param_name_length;
	uint8_t param_id;
	bool  schedule_for_transmission;
} Onboard_Parameter_t;

typedef struct {
	Onboard_Parameter_t parameters[MAX_ONBOARD_PARAM_COUNT];
	int param_count;
	bool enumerate;
	int transmit_parameter_index;
	
}Parameter_Set_t;
	
void init_onboard_parameters(void);
void add_parameter_uint32(uint32_t* val, const char* param_name);
void add_parameter_int32(int32_t* val, const char* param_name);
void add_parameter_float(float* val, const char* param_name);
void send_all_parameters_now(void);
void send_all_parameters(void);
void send_scheduled_parameters(void);
void send_parameter(mavlink_param_request_read_t* request);
void receive_parameter(Mavlink_Received_t* rec);

#endif /* ONBOARD_PARAMETERS_H */