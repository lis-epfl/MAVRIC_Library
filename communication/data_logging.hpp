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
 * \file data_logging.hpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 * 
 * \brief Performs the data logging on a file
 *
 ******************************************************************************/


#ifndef DATA_LOGGING_H__
#define DATA_LOGGING_H__

#include "mavlink_communication.hpp"
#include "state.hpp"
#include "toggle_logging.hpp"
#include "file.hpp"
#include "console.hpp"

/**
 * \brief	Structure of data logging parameter.
 */
typedef struct
{
	double* param;												///< Pointer to the parameter value
	char param_name[MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];	///< Parameter name composed of 16 characters
	mavlink_message_type_t data_type;							///< Parameter type
	uint8_t precision;											///< Number of digit after the zero
} data_logging_entry_t;


/**
 * \brief 		Set of data logging parameters
 * 
 * \details 	Uses C99's flexible member arrays: it is required to 
 * 				allocate memory for this structure
 */
typedef struct
{
	uint32_t data_logging_count;				///< Number of data logging parameter effectively in the array
	uint32_t max_data_logging_count; 			///< Maximum number of logged parameters
	uint16_t max_logs;							///< The max number of logged files with the same name on the SD card
	data_logging_entry_t data_log[];			///< Data logging array, needs memory allocation
} data_logging_set_t;

/**
 * \brief	The class of the log of the data
 *
 * \details  	data_logging_set is implemented as pointer because its memory will be
 * 				allocated during initialisation
 */
class Data_logging
{
public:
	/**
	 * \brief	Data logging constructor
	 */
	Data_logging(File& file);


	/**
	 * \brief	Initialise the data logging module
	 *
	 * \param	file_name				The pointer to name of the file to create
	 * \param	continuous_write		Boolean to state whether writing should be continous or not
	 * \param	toggle_logging			The pointer to toggle logging structure
	 * \param 	state 					The pointer to the state structure
	 * \param	sysid					The system identification number of the MAV
	 *
	 * \return	True if the init succeed, false otherwise
	 */
	bool create_new_log_file(const char* file_name_, bool continuous_write_, const toggle_logging_t* toggle_logging_, const State* state_, uint32_t sysid);

	/**
	 * \brief	The task to log the data to the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 *
	 * \return	The result of the task execution
	 */
	bool update();

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(uint8_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(int8_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(uint16_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(int16_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(uint32_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(int32_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(uint64_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(int64_t* val, const char* param_name);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 * \param 	precision				The number of digit after the zero
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(float* val, const char* param_name, uint32_t precision);

	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 * \param 	precision				The number of digit after the zero
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(double* val, const char* param_name, uint32_t precision);


	/**
	 * \brief	Registers parameter to log on the SD card
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param 	val						The parameter value
	 * \param 	param_name				Name of the parameter
	 *
	 * \return	True if the parameter was added, false otherwise
	 */
	bool add_field(bool* val, const char* param_name);


private:
		/**
	 * \brief	Add in the file fp the first line with the name of the parameter
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 */
	void add_header_name(void);


	/**
	 * \brief	Function to put a \r or a \n after the data logging parameter value (\r between them and \n and the end)
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 * \param	param_num				The index of the data logging parameter
	 */
	void put_r_or_n(uint16_t param_num);


	/**
	 * \brief	Function to log a new line of values
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 */
	void log_parameters(void);


	/**
	 * \brief	Seek the end of an open file to append
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 */
	void seek(void);


	/**
	* \brief	Appends ".txt" to the end of a character string. If not enough 
	*			memory allocated in output, will write as many letters from
	*			filename as possible, will not include .txt\0 unless entire
	*			.txt\0 can fit. 
	*
	* \param	output		The output character string
	* \param	filename	The input string
	* \param	length		The maximum length of output
	*
	* \return	success		Bool stating if the entire output was written
	*/
	bool filename_append_extension(char* output, char* filename, uint32_t length);


	/**
	* \brief	Appends a uint32_t to a character string with an underscore between.
	*			If not enough memory allocated in output, will write as many letters
	*			from filename as possible, will not include num\0 unless entire number
	*			and null character can fit.
	*
	* \param	output		The output character string
	* \param	filename	The input string
	* \param	num			The uint32_t to be appended to filename
	* \param	length		The maximum length of the output string, must be positive
	*
	* \return	success		Bool stating if the entire output was written
	*/
	bool filename_append_int(char* output, char* filename, uint32_t num, uint32_t length);


	/**
	 * \brief	Create and open a new file
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 *
	 * \result	True if the file was open correctly, false otherwise
	 */
	bool open_new_log_file(void);


	/**
	 * \brief	Computes the checksum of data logging 
	 *
	 * \param	data_logging			The pointer to the data logging structure
	 *
	 * \result	True if the the checksum of the data didn't change, false otherwise
	 */
	bool checksum_control(void);

	bool debug;									///< Indicates if debug messages should be printed for each param change

	data_logging_set_t* data_logging_set;		///< Pointer to a set of parameters, needs memory allocation

	int buffer_name_size;						///< The buffer for the size of the file's name

	char *file_name;							///< The file name
	char *name_n_extension;						///< Stores the name of the file

	bool file_init;								///< A flag to tell whether a file is init or not
	bool file_opened;							///< A flag to tell whether a file is opened or not

	bool continuous_write;						///< A flag to tell whether we write continuously to the file or not

	uint32_t logging_time;						///< The time that we've passed logging since the last f_close
	
	double cksum_a;								///< Checksum to see if the onboard parameters have changed values
	double cksum_b;								///< Checksum to see if the onboard parameters have changed values
	
	uint32_t sys_id;							///< the system ID
	
	Console<File> console;						///< The console containing the file to write data to

	const State* state;							///< The pointer to the state structure	
	const toggle_logging_t* toggle_logging;		///< The pointer to the toggle logging structure
};

#endif /* DATA_LOGGING_H__ */
