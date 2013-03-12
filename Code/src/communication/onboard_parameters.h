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
	float* param[MAX_ONBOARD_PARAM_COUNT];
	char param_name[MAX_ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
	mavlink_message_type_t data_types[MAX_ONBOARD_PARAM_COUNT];
	uint8_t param_name_length;
	uint8_t param_count;
} Onboard_Parameters_t;

void init_onboard_parameters(void);
void add_parameter_uint32(uint32_t* val, const char* param_name);
void add_parameter_int32(int32_t* val, const char* param_name);
void add_parameter_float(float* val, const char* param_name);
void add_PID_parameters(void);
void send_all_parameters(Mavlink_Received_t* rec);
void send_parameter(Mavlink_Received_t* rec);
void receive_parameter(Mavlink_Received_t* rec);

#endif /* ONBOARD_PARAMETERS_H */