/*
 * onboard_parameters.h
 *
 * Created: 19/02/2013 10:34:25
 *  Author: julien
 */ 

#ifndef ONBOARD_PARAMETERS_H_
#define ONBOARD_PARAMETERS_H_

#include "mavlink_stream.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_ONBOARD_PARAM_COUNT 120	// should be < 122 to fit on user page on AT32UC3C1512

#define MAVERIC_FLASHC_USER_PAGE_START_ADDRESS AVR32_FLASHC_USER_PAGE_ADDRESS + 0x04	// +4bytes for unknown reason
#define MAVERIC_FLASHC_USER_PAGE_FREE_SPACE 500	// 	512bytes user page, 
												//	-4bytes at the start, 
												//  -8bytes for the protected fuses at the end of the user page

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
	uint16_t param_count;
	bool enumerate;
	int transmit_parameter_index;
	
}Parameter_Set_t;

typedef struct{
	float values[MAVERIC_FLASHC_USER_PAGE_FREE_SPACE];
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

void read_parameters_from_flashc(void);
void write_parameters_to_flashc(void);

#ifdef __cplusplus
}
#endif

#endif /* ONBOARD_PARAMETERS_H */