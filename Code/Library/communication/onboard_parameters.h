/*
 * onboard_parameters.h
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#ifndef ONBOARD_PARAMETERS_H_
#define ONBOARD_PARAMETERS_H_

#include "mavlink_stream.h"

#define MAX_ONBOARD_PARAM_COUNT 120

#define USER_PAGE_FIRST_FREE_WORD 0x8080020C

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

typedef struct{
	float values[MAX_ONBOARD_PARAM_COUNT+3]; // for number of parameters and two checksums
} nvram_data_ttt;	

nvram_data_ttt *nvram_array;

void init_onboard_parameters(void);

/**
 * registers parameter in the internal parameter list that gets published to MAVlink
 */
void add_parameter_uint8(uint8_t* val, const char* param_name);
void add_parameter_uint32(uint32_t* val, const char* param_name);
void add_parameter_int32(int32_t* val, const char* param_name);
void add_parameter_float(float* val, const char* param_name);

/** updates linked memory location of a parameter with given value
 *  This method takes care of necessary size/type conversions
*/
void update_parameter(int param_index, float value);

/** reads linked memory location and returns parameter value
 *  This method takes care of necessary size/type conversions.
 *  Note that the parameter might not be a float, but float is the 
 *  default data type for the MAVlink message.
*/
float read_parameter(int param_index);

/**
 * Immediately sends all parameters via MAVlink. This might block for a while.
 */
void send_all_parameters_now(void);

/**
 * marks all parameters to be scheduled for transmission
 */
void send_all_parameters(void);
void send_scheduled_parameters(void);

/**
 * responds to a MAVlink parameter request
 */
void send_parameter(mavlink_param_request_read_t* request);

/**
 * responds to a MAVlink parameter set
 */
void receive_parameter(Mavlink_Received_t* rec);

void read_parameters_from_flashc();
void write_parameters_to_flashc();

#endif /* ONBOARD_PARAMETERS_H */