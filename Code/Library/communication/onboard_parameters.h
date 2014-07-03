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
 * \file onboard_parameters.h
 * 
 * Mav'ric Onboard parameters
 */


#ifndef ONBOARD_PARAMETERS_H_
#define ONBOARD_PARAMETERS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "mavlink_stream.h"

#define MAX_ONBOARD_PARAM_COUNT 120	// should be < 122 to fit on user page on AT32UC3C1512

#define MAVERIC_FLASHC_USER_PAGE_START_ADDRESS AVR32_FLASHC_USER_PAGE_ADDRESS + 0x04	// +4bytes for unknown reason
#define MAVERIC_FLASHC_USER_PAGE_FREE_SPACE 500	// 	512bytes user page, 
												//	-4bytes at the start, 
												//  -8bytes for the protected fuses at the end of the user page
												
/**
 * \brief	Structure of onboard parameter.
 */
typedef struct
{
	float* param;												///< Pointer to the parameter value
	char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];	///< Parameter name composed of 16 characters
	mavlink_message_type_t data_type;							///< Parameter type
	uint8_t param_name_length;									///< Length of the parameter name
	uint8_t param_id;											///< Parameter ID
	bool  schedule_for_transmission;							///< Boolean to activate the transmission of the parameter
}Onboard_Parameter_t;

/**
 * \brief	Set of parameter composed of onboard parameters and number of parameters.
 */
typedef struct 
{
	Onboard_Parameter_t parameters[MAX_ONBOARD_PARAM_COUNT];	///< Onboard parameters array
	uint16_t param_count;										///< Number of onboard parameter effectively in the array
}Parameter_Set_t;											

/**
 * \brief	TODO: Modify the name of this structure to make it sized as the free flash memory to store these parameters
 */															
typedef struct												
{
	float values[MAVERIC_FLASHC_USER_PAGE_FREE_SPACE];
}nvram_data_ttt;

nvram_data_ttt *nvram_array;

/**
* \brief	Initialisation of the Parameter_Set structure by setting the number of onboard parameter to 0
*/
void onboard_parameters_init(void);

/**
 * \brief				Register parameter in the internal parameter list that gets published to MAVlink
 *
 * \param val			Unsigned 8 - bits integer parameter value
 * \param param_name	Name of the parameter
 */
void onboard_parameters_add_parameter_uint8(uint8_t* val, const char* param_name);

/**
 * \brief				Register parameter in the internal parameter list that gets published to MAVlink
 *
 * \param val			Unsigned 32 - bits integer parameter value
 * \param param_name	Name of the parameter
 */
void onboard_parameters_add_parameter_uint32(uint32_t* val, const char* param_name);

/**
 * \brief				Register parameter in the internal parameter list that gets published to MAVlink
 *
 * \param val			Signed 32 - bits integer parameter value
 * \param param_name	Name of the parameter
 */
void onboard_parameters_add_parameter_int32(int32_t* val, const char* param_name);

/**
 * \brief				Registers parameter in the internal parameter list that gets published to MAVlink
 *
 * \param val			Floating point parameter value
 * \param param_name	Name of the parameter
 */
void onboard_parameters_add_parameter_float(float* val, const char* param_name);

/**
 * \brief				Updates linked memory location of a parameter with given value, with care of necessary size/type conversions
 *
 * \param param_index	Set index of the parameter to update
 * \param value			Value of the parameter to update
 */
void onboard_parameters_update_parameter(int param_index, float value);

/** 
 *  This method takes care of necessary size/type conversions.
 *  
*/
/**
 * \brief				Reads linked memory location and returns parameter value, with care of necessary size/type conversions
 *
 * \param param_index	Set index of the parameter to update
 *
 * \return				Value of the parameter read. Note that the parameter might not be a float, but float is the default data type for the MAVlink message.
 */
float onboard_parameters_read_parameter(int param_index);

/**
 * \brief	Immediately sends all parameters via MAVlink. This might block for a while.
 */
void onboard_parameters_send_all_parameters_now(void);

/**
 * \brief	Marks all parameters to be scheduled for transmission
 */
void onboard_parameters_send_all_parameters(void);

/**
 * \brief	Sends all parameters that have been scheduled via MAVlink
 */
void onboard_parameters_send_scheduled_parameters(void);

/**
 * \brief			Responds to a MAVlink parameter request
 *
 * \param request	Pointer to the request structure received by MAVlink
 */
void onboard_parameters_send_parameter(mavlink_param_request_read_t* request);

/**
 * \brief		Responds to a MAVlink parameter set
 *
 * \param rec	Pointer to the received parameter structure
 */
void onboard_parameters_receive_parameter(Mavlink_Received_t* rec);

/**
 * \brief		Read onboard parameters from the user page in the flash memory to the RAM memory
 */
void onboard_parameters_read_parameters_from_flashc(void);

/**
 * \brief		Write onboard parameters to the RAM memory from the user page in the flash memory
 */
void onboard_parameters_write_parameters_from_flashc(void);

#ifdef __cplusplus
}
#endif

#endif /* ONBOARD_PARAMETERS_H */