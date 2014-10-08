/*******************************************************************************
 * Copyright (c) 2009-2014, MAV'RIC Development Team
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
 
/*******************************************************************************
 * \file data_logging.h
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief Performs the data logging on the SD card
 *
 ******************************************************************************/


#ifndef DATA_LOGGING_H__
#define DATA_LOGGING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fat_fs/ff.h"
#include "tasks.h"


#define MAX_DATA_LOGGING_COUNT 50								///< The max number of data logging parameters

#define MAX_NUMBER_OF_LOGGED_FILE 500							///< The max number of logged files with the same name on the SD card

#define LOGGING_INTERVAL_SEC 10

/**
 * \brief	Structure of data logging parameter.
 */
typedef struct
{
	double* param_double;										///< Pointer to the parameter value
	double* param;
	char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];	///< Parameter name composed of 16 characters
	mavlink_message_type_t data_type;							///< Parameter type
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
	uint32_t max_data_logging_count;							///< Maximum number of parameters
	bool debug;													///< Indicates if debug messages should be printed for each param change
	uint32_t log_data;											///< The initial state of writing a file
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

	int buffer_name_size;										///< The buffer for the size of the file's name
	int buffer_add_size;										///< The buffer for the size of the file's extension char*

	char *file_name;											///< The file name
	char *name_n_extension;										///< Stores the name of the file

	bool file_init;												///< A flag to tell whether a file is init or not
	bool file_opened;											///< A flag to tell whether a file is opened or not
	bool file_name_init;										///< A flag to tell whether a valid name was proposed
	bool sys_mounted;											///< A flag to tell whether the file system is mounted
	
	uint32_t loop_count;										///< Counter to try to mount the SD card many times
	
	uint32_t logging_time;										///< The time that we've passed logging since the last f_close
	
	uint32_t log_data;											///< A flag to stop/start writing to file	
}data_logging_t;


/**
 * \brief	Initialise the data logging module
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param	config					The pointer to the configuration structure
 * \param	mavlink_communication	The pointer to the communication structure
 */
void data_logging_init(data_logging_t* data_logging, const data_logging_conf_t* config);

/**
 * \brief	Create and open a new file
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param	file_name				The name of the file to create
 */
void data_logging_create_new_log_file(data_logging_t* data_logging, const char* file_name);

/**
 * \brief	The task to log the data to the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 *
 * \return	The result of the task execution
 */
task_return_t data_logging_update(data_logging_t* data_logging);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_uint8(data_logging_t* data_logging, uint8_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_int8(data_logging_t* data_logging, int8_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_uint16(data_logging_t* data_logging, uint16_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_int16(data_logging_t* data_logging, int16_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_uint32(data_logging_t* data_logging, uint32_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_int32(data_logging_t* data_logging, int32_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_uint64(data_logging_t* data_logging, uint64_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_int64(data_logging_t* data_logging, int64_t* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_float(data_logging_t* data_logging, float* val, const char* param_name);

/**
 * \brief	Registers parameter to log on the SD card
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param 	val						The parameter value
 * \param 	param_name				Name of the parameter
 */
void data_logging_add_parameter_double(data_logging_t* data_logging, double* val, const char* param_name);


#ifdef __cplusplus
}
#endif

#endif /* DATA_LOGGING_H__ */