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
 * \file data_logging.h
 *
 *  Performs the data logging on the SD card
 */


#ifndef DATA_LOGGING_H__
#define DATA_LOGGING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "FatFs/ff.h"
#include "tasks.h"


#define MAX_DATA_LOGGING_COUNT 100								///< The max number of data logging parameters

/**
 * \brief	Structure of data logging parameter.
 */
typedef struct
{
	double* param;												///< Pointer to the parameter value
	char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];	///< Parameter name composed of 16 characters
	mavlink_message_type_t data_type;							///< Parameter type
	uint8_t param_name_length;									///< Length of the parameter name
	bool  schedule_for_logging;									///< Boolean to activate the transmission of the parameter
} data_logging_entry_t;


/**
 * \brief 		Set of data logging parameters
 * 
 * \details 	Uses C99's flexible member arrays: it is required to 
 * 				allocate memory for this structure
 */
typedef struct
{
	uint32_t data_logging_count;								///< Number of data logging parameter effectively in the array
	uint32_t max_data_logging_count; 							///< Maximum number of logged parameters
	data_logging_entry_t data_log[];							///< Data logging array, needs memory allocation
} data_logging_set_t;


/**
 * \brief 	Configuration for the module data logging
 */
typedef struct
{
	uint32_t max_data_logging_count;									///< Maximum number of parameters
	bool debug;													///< Indicates if debug messages should be printed for each param change
} data_logging_conf_t;


/**
 * \brief	The structure to log the data
 *
 * \details  	data_logging_set is implemented as pointer because its memory will be
 * 				allocated during initialisation
 */
typedef struct  
{	
	bool debug;													///< Indicates if debug messages should be printed for each param change
	data_logging_set_t* data_logging_set;						///< Pointer to a set of parameters, needs memory allocation
	
	FRESULT fr;													///< The result of the fatfs functions
	FATFS fs;													///< The fatfs handler
	FIL fil;													///< The fatfs file handler

	uint32_t time_ms;											///< The microcontroller time in ms

	bool file_init;												///< A flag to tell whether a file is init or not
	
	bool continue_writing;										///< A flag to stop/start writing to file	
}data_logging_t;


/**
 * \brief	Initialise the data logging module
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param	imu						The pointer to the imu structure
 * \param	gps						The pointer to the gps structure
 */
void data_logging_init(data_logging_t* data_logging, const data_logging_conf_t* config);

/**
 * \brief	The task to log the data to the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 *
 * \return	The result of the task execution
 */
task_return_t data_logging_run(data_logging_t* data_logging);

void data_logging_add_parameter_uint8(data_logging_t* data_logging, uint8_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_int8(data_logging_t* data_logging, int8_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_uint16(data_logging_t* data_logging, uint16_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_int16(data_logging_t* data_logging, int16_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_uint32(data_logging_t* data_logging, uint32_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_int32(data_logging_t* data_logging, int32_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_uint64(data_logging_t* data_logging, uint64_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_int64(data_logging_t* data_logging, int64_t* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_float(data_logging_t* data_logging, float* val, const char* param_name, bool schedule_for_logging);
void data_logging_add_parameter_double(data_logging_t* data_logging, double* val, const char* param_name, bool schedule_for_logging);


#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGING_H__ */