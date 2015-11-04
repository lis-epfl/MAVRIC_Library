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
 * \file data_logging.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the data logging on the SD card
 *
 ******************************************************************************/


#include "data_logging.h"
#include "print_util.h"
#include "time_keeper.h"

#include <stdlib.h>
#include <math.h>

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Add in the file fp the first line with the name of the parameter
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_add_header_name(data_logging_t* data_logging);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The floating point number to be added
 * \param	after_digits			The number of digits after the floating point
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_float(FIL* fp, float c, uint32_t after_digits);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 * \param	after_digits			The number of digits after the floating point
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_double(FIL* fp, double c, uint32_t after_digits);

/**
 * \brief	Function to put a uint64_t number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_uint64_t(FIL* fp, uint64_t c);

/**
 * \brief	Function to put a int64_t number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_int64_t(FIL* fp, int64_t c);

/**
 * \brief	Function to put a \r or a \n after the data logging parameter value (\r between them and \n and the end)
 *
 * \param	data_logging			The pointer to the data logging structure
 * \param	param_num				The index of the data logging parameter
 */
static void data_logging_put_r_or_n(data_logging_t* data_logging, uint16_t param_num);

/**
 * \brief	Function to log a new line of values
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_log_parameters(data_logging_t* data_logging);

/**
 * \brief	Seek the end of an open file to append
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_f_seek(data_logging_t* data_logging);

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
static bool data_logging_filename_append_extension(char* output, char* filename, int length);

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
static bool data_logging_filename_append_int(char* output, const char* filename, uint32_t num, int length);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void data_logging_add_header_name(data_logging_t* data_logging)
{
	bool init = false;
	
	uint16_t i;
	data_logging_set_t* data_set = data_logging->data_logging_set;
	
	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		
		int32_t res = f_printf(&data_logging->fil,param->param_name);
		if (res == EOF)
		{
			if (data_logging->debug)
			{
				print_util_dbg_print("Error appending header!\r\n");
			}
			init = false;
		}
		else
		{
			init = true;
			data_logging_put_r_or_n(data_logging, i);
		}
	}
	data_logging->file_init = init;
}

static int32_t data_logging_put_float(FIL* fp, float c, uint32_t after_digits)
{
	uint32_t i;
	int32_t res = 0;
	float num = c;
	
	if (c<0)
	{
		res = f_puts("-",fp);
		if (res == EOF)
		{
			return res;
		}
		num=-c;
	}

	int32_t whole=(int32_t)(maths_f_abs(num));
	float after=(num-(float)whole);

	res = f_printf(fp,"%d", whole);
	if (res == EOF)
	{
		return res;
	}
	
	res = f_puts(".",fp);
	if (res == EOF)
	{
		return res;
	}
	
	for (i=0; i<after_digits; i++)
	{
		after*=10;
		res = f_printf(fp, "%d",(int32_t)after);
		if (res == EOF)
		{
			return res;
		}
		after=after-(int32_t)after;
	}
	
	return res;
}

static int32_t data_logging_put_double(FIL* fp, double c, uint32_t after_digits)
{
	uint32_t i;
	int32_t res = 0;
	double num = c;
	
	if (c<0)
	{
		res = f_puts("-",fp);
		if (res == EOF)
		{
			return res;
		}
		num=-c;
	}

	int64_t whole=(int64_t)floor(num);
	double after=(num-(double)whole);

	data_logging_put_uint64_t(fp,(uint64_t)whole);
	
	res = f_puts(".",fp);
	if (res == EOF)
	{
		return res;
	}
	
	for (i=0; i<after_digits; i++)
	{
		after*=10;
		res = f_printf(fp, "%ld",(int32_t)after);
		if (res == EOF)
		{
			return res;
		}
		after=after-(int64_t)after;
	}
	
	return res;
}

static int32_t data_logging_put_uint64_t(FIL* fp, uint64_t c)
{
	int32_t res = EOF;

	char storage[MAX_DIGITS_LONG];
	int32_t i = MAX_DIGITS_LONG;
	
	do
	{
		i--;
		storage[i] = c % 10;
		c = c / 10;
	} while((i >= 0) && (c > 0) );

	for( ; i<MAX_DIGITS_LONG; i++)
	{
		res = f_printf(fp,"%d",storage[i]);
		if (res == EOF)
		{
			return res;
		}
	}
	
	return res;
}

static int32_t data_logging_put_int64_t(FIL* fp, int64_t c)
{
	int32_t res = 0;
	int64_t num = c;
	
	if (c<0)
	{
		res = f_puts("-",fp);
		if (res == EOF)
		{
			return res;
		}
		num=-c;
	}
	
	res = data_logging_put_uint64_t(fp,(uint64_t)num);
	
	return res;
}

static void data_logging_put_r_or_n(data_logging_t* data_logging, uint16_t param_num)
{
	int32_t res = 0;
	
	if (param_num == (data_logging->data_logging_set->data_logging_count-1))
	{
		res = f_puts("\n",&data_logging->fil);
	}
	else
	{
		res = f_puts("\t",&data_logging->fil);
	}
	if (res == EOF)
	{
		if (data_logging->debug)
		{
			data_logging->fr = f_stat(data_logging->name_n_extension,NULL);
			print_util_dbg_print("Error putting tab or new line character!\r\n");
			fat_fs_mounting_print_error_signification(data_logging->fr);
			print_util_dbg_print("\r\n");
		}
	}
}

static void data_logging_log_parameters(data_logging_t* data_logging)
{
	uint16_t i;
	uint32_t res = 0;
	data_logging_set_t* data_set = data_logging->data_logging_set;
	
	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		switch(param->data_type)
		{
			case MAV_PARAM_TYPE_UINT8:
				res = f_printf(&data_logging->fil, "%d", *((uint8_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_INT8:
				res = f_printf(&data_logging->fil, "%d", *((int8_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_UINT16:
				res = f_printf(&data_logging->fil, "%d", *((uint16_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_INT16:
				res = f_printf(&data_logging->fil, "%d", *((int16_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
					
			case MAV_PARAM_TYPE_UINT32:
				res = f_printf(&data_logging->fil, "%d", *((uint32_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_INT32:
				res = f_printf(&data_logging->fil, "%d", *((int32_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_UINT64:
				res = data_logging_put_uint64_t(&data_logging->fil,*((uint64_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
				
			case MAV_PARAM_TYPE_INT64:
				res = data_logging_put_int64_t(&data_logging->fil,*((int64_t*)param->param));
				data_logging_put_r_or_n(data_logging,i);
				break;
					
			case MAV_PARAM_TYPE_REAL32:
				res = data_logging_put_float(&data_logging->fil,*((float*)param->param),param->precision);
				data_logging_put_r_or_n(data_logging,i);
				break;
					
			case MAV_PARAM_TYPE_REAL64:
				res = data_logging_put_double(&data_logging->fil,*((double*)param->param),param->precision);
				data_logging_put_r_or_n(data_logging,i);
				break;
			default:
				print_util_dbg_print("Data type not supported!\r\n");
		}
		if (res == EOF)
		{
			if (data_logging->debug)
			{
				data_logging->fr = f_stat(data_logging->name_n_extension,NULL);
				print_util_dbg_print("Error appending parameter! Error:");
				fat_fs_mounting_print_error_signification(data_logging->fr);
			}
			break;
		}
	}
}

static void data_logging_f_seek(data_logging_t* data_logging)
{
	/* Seek to end of the file to append data */
	data_logging->fr = f_lseek(&data_logging->fil, f_size(&data_logging->fil));
	if (data_logging->fr != FR_OK)
	{
		if (data_logging->debug)
		{
			print_util_dbg_print("lseek error:");
			fat_fs_mounting_print_error_signification(data_logging->fr);
		}
		f_close(&data_logging->fil);
	}
}

static bool data_logging_filename_append_extension(char* output, char* filename, int length)
{
	// Success flag
	bool is_success = true;

	// Declare counter for char location
	int i = 0;

	// Copy characters to output from filename until null character is reached
	while (filename[i] != '\0')
	{
		output[i] = filename[i];
		i++;

		// If i is one less than length
		if (i == (length - 1))
		{
			// Set last character of output to null character
			output[i] = '\0';

			// Return is_success as false
			is_success = false;
			return is_success;
		}
	}

	// If there is not enough room for .txt\0
	if ((i + 5) >= (length))
	{
		// Set last character of output to null, dont append
		// .txt
		output[i] = '\0';
		
		// Return is_success as false
		is_success = false;
		return is_success;
	}

	// Add ".txt"
	output[i] = '.';
	output[i + 1] = 't';
	output[i + 2] = 'x';
	output[i + 3] = 't';

	// Add null character
	output[i + 4] = '\0';

	// Return is_success as true;
	return is_success;
}

static bool data_logging_filename_append_int(char* output, const char* filename, uint32_t num, int length)
{
	// Success flag
	bool is_success = true;

	// Declare counter for char location
	int i = 0;

	// Copy characters to output from filename until null character is
	while (filename[i] != '\0')
	{
		output[i] = filename[i];
		i++;

		// If i is one less than length
		if (i == (length - 1))
		{
			// Set last character of output to null character
			output[i] = '\0';

			// Return is_success as false
			is_success = false;
			return is_success;
		}
	}

	// Add underscore
	output[i] = '_';

	// Count number of digits
	int num_digits = 0;
	uint32_t num_copy = num;
	do // Do while loop to have 0 written as 1 digit
	{
		num_digits++; // Add one to digits
		num_copy = num_copy / 10; // Remove digit from num_copy
	} while (num_copy != 0);

	// If the number of digits + i is greater than or equal to length - 1
	if ((num_digits + i) >= (length - 1))
	{
		// Not enough space is allocated
		// Set null character (overwrite _ since number wont be outputted)
		output[i] = '\0';

		// Return false
		is_success = false;
		return is_success;
	} // If not, then there is enough space and continue

	  // Add num_digits to i
	i += num_digits;
	do // Remove digits right to left adding them to output
	{
		output[i] = (num % 10) + '0';
		num = num / 10;
		i--; // Subtrack as we are moving right to left
	} while (num != 0); // Stop when rev_num has gone through all the digits

						// Add null character to i+num_digits+1
	output[i + num_digits + 1] = '\0';

	return is_success;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool data_logging_create_new_log_file(data_logging_t* data_logging, const char* file_name, bool continuous_write, fat_fs_mounting_t* fat_fs_mounting, uint32_t sysid)
{
	bool init_success = true;
	
	const data_logging_conf_t* config = &fat_fs_mounting->data_logging_conf;

	data_logging->debug = config->debug;

	data_logging->continuous_write = continuous_write;
	data_logging->data_write = true;

	data_logging->state = fat_fs_mounting->state;
	data_logging->fat_fs_mounting = fat_fs_mounting;

	// Allocate memory for the onboard data_log
	data_logging->data_logging_set = malloc( sizeof(data_logging_set_t) + sizeof(data_logging_entry_t[config->max_data_logging_count]) );
	
	if ( data_logging->data_logging_set != NULL )
	{
		data_logging->data_logging_set->max_data_logging_count = config->max_data_logging_count;
		data_logging->data_logging_set->max_logs = config->max_logs;
		data_logging->data_logging_set->data_logging_count = 0;
		
		init_success &= true;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r\n");
		data_logging->data_logging_set->max_data_logging_count = 0;
		data_logging->data_logging_set->max_logs = 0;
		data_logging->data_logging_set->data_logging_count = 0;
		
		init_success &= false;
	}

	// Automatically add the time as first logging parameter
	data_logging_add_parameter_uint32(data_logging,&data_logging->time_ms,"time");
	
	data_logging->file_init = false;
	data_logging->file_opened = false;
	
	#if _USE_LFN
	data_logging->buffer_name_size = _MAX_LFN;
	data_logging->buffer_add_size = _MAX_LFN;
	#else
	data_logging->buffer_name_size = 8;
	data_logging->buffer_add_size = 8;
	#endif
	
	data_logging->file_name = malloc(data_logging->buffer_name_size);
	data_logging->name_n_extension = malloc(data_logging->buffer_name_size);
	
	if (data_logging->file_name == NULL)
	{
		init_success &= false;
	}
	if (data_logging->name_n_extension == NULL)
	{
		init_success &= false;
	}
	
	data_logging->sys_id = sysid;

	// Append sysid to filename
	data_logging_filename_append_int(data_logging->file_name, file_name, sysid, data_logging->buffer_name_size);

	init_success &= data_logging_open_new_log_file(data_logging);

	data_logging->logging_time = time_keeper_get_millis();
	
	print_util_dbg_print("[DATA LOGGING] initialised.\r\n");

	return init_success;
}

bool data_logging_open_new_log_file(data_logging_t* data_logging)
{
	bool create_success = true;

	int32_t i = 0;
	
	char *file_add = malloc(data_logging->buffer_add_size);
	
	if (file_add == NULL)
	{
		create_success &= false;
		print_util_dbg_print("[DATA LOGGING] Error opening new file: cannot allocate memory.\r\n");
	}
	
	if (data_logging->fat_fs_mounting->log_data)
	{
		do 
		{
			// Create flag for successfully written file names
			bool successful_filename = true;

			// Add iteration number to name_n_extension (does not yet have extension)
			successful_filename &= data_logging_filename_append_int(data_logging->name_n_extension, data_logging->file_name, i, data_logging->name_n_extension);

			// Add extension (.txt) to name_n_extension
			successful_filename &= data_logging_filename_append_extension(data_logging->name_n_extension, data_logging->name_n_extension, data_logging->name_n_extension);

			// Check if there wasn't enough memory allocated to name_n_extension
			if (successful_filename)
			{
				print_util_dbg_print("Name error: The name is too long! It should be, with the extension, maximum ");
				print_util_dbg_print_num(data_logging->buffer_name_size,10);
				print_util_dbg_print(" and it is ");
				print_util_dbg_print_num(sizeof(data_logging->name_n_extension),10);
				print_util_dbg_print("\r\n");
				
				create_success &= false;
			}
		
			data_logging->fr = f_open(&data_logging->fil, data_logging->name_n_extension, FA_WRITE | FA_CREATE_NEW);
			
			if (data_logging->debug)
			{
				print_util_dbg_print("f_open result:");
				fat_fs_mounting_print_error_signification(data_logging->fr);
			}
		
			++i;
		
		//}while((i < data_logging->data_logging_set->max_data_logging_count)&&(data_logging->fr != FR_OK)&&(data_logging->fr != FR_NOT_READY));
		} while( (i < data_logging->data_logging_set->max_data_logging_count) && (data_logging->fr == FR_EXIST) );
	
		if (data_logging->fr == FR_OK)
		{
			data_logging_f_seek(data_logging);
		}
	
		if (data_logging->fr == FR_OK)
		{
			data_logging->file_opened = true;
			
			data_logging->fat_fs_mounting->num_file_opened++;

			if (data_logging->debug)
			{
				print_util_dbg_print("File ");
				print_util_dbg_print(data_logging->name_n_extension);
				print_util_dbg_print(" opened. \r\n");
				
				create_success &= true;
			}
		} //end of if data_logging->fr == FR_OK
	}//end of if (data_logging->fat_fs_mounting->log_data)
	
	free(file_add);
	
	return create_success;
}

task_return_t data_logging_update(data_logging_t* data_logging)
{
	if (data_logging->fat_fs_mounting->log_data == 1)
	{
		if (data_logging->fat_fs_mounting->sys_mounted)
		{
			if (data_logging->file_opened)
			{
				if (!data_logging->file_init)
				{
					data_logging_add_header_name(data_logging);
				}

				data_logging->time_ms = time_keeper_get_millis();
				
				if (data_logging->state->mav_mode.ARMED == ARMED_OFF)
				{
					if ( (data_logging->time_ms - data_logging->logging_time) > 5000)
					{
						data_logging->fr = f_sync(&data_logging->fil);
						data_logging->logging_time = data_logging->time_ms;
					}
				}
				
				if (data_logging->fr == FR_OK)
				{
					if (data_logging->data_write)
					{
						data_logging_log_parameters(data_logging);
					}
					if (!data_logging->continuous_write)
					{
						data_logging->data_write = false;
					}
					
				}
			} //end of if (data_logging->file_opened)
			else
			{
				data_logging_open_new_log_file(data_logging);
			}//end of else if (data_logging->file_opened)
		}//end of if (sys_mounted)
		else
		{
			fat_fs_mounting_mount(data_logging->fat_fs_mounting, data_logging->debug);
		}//end of else if (sys_mounted)
	}//end of if (data_logging->fat_fs_mounting->log_data == 1)
	else
	{
		if (data_logging->file_opened)
		{
			if (data_logging->fr != FR_NO_FILE)
			{
				bool succeed = false;
				for (uint8_t i = 0; i < 5; ++i)
				{
					if (data_logging->debug)
					{
						print_util_dbg_print("Attempt to close file ");
						print_util_dbg_print(data_logging->name_n_extension);
						print_util_dbg_print("\r\n");
					}

					data_logging->fr = f_close(&data_logging->fil);
					
					if ( data_logging->fr == FR_OK)
					{
						succeed = true;
						break;
					}
					if(data_logging->fr == FR_NO_FILE)
					{
						break;
					}
				} //end for loop
					
				data_logging->file_opened = false;
				data_logging->file_init = false;
				
				data_logging->fat_fs_mounting->num_file_opened--;

				if (data_logging->debug)
				{
					if ( succeed)
					{
						print_util_dbg_print("File closed\r\n");
					}
					else
					{
						print_util_dbg_print("Error closing file\r\n");	
					}
				}
			}//end of if (data_logging->fr != FR_NO_FILE)
			else
			{
				data_logging->file_opened = false;
				data_logging->file_init = false;
			}
		}//end of if (data_logging->file_opened)

		fat_fs_mounting_unmount(data_logging->fat_fs_mounting, data_logging->debug);

		if (!data_logging->fat_fs_mounting->sys_mounted)
		{
			data_logging->file_opened = false;
			//data_logging->sys_mounted = false;
			data_logging->file_init = false;
		}
	}//end of else (data_logging->fat_fs_mounting->log_data != 1)
	return TASK_RUN_SUCCESS;
}

bool data_logging_add_parameter_uint8(data_logging_t* data_logging, uint8_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_UINT8;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_int8(data_logging_t* data_logging, int8_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_INT8;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}

	return add_success;
}

bool data_logging_add_parameter_uint16(data_logging_t* data_logging, uint16_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_UINT16;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_int16(data_logging_t* data_logging, int16_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_INT16;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_uint32(data_logging_t* data_logging, uint32_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_UINT32;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_int32(data_logging_t* data_logging, int32_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_INT32;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_uint64(data_logging_t* data_logging, uint64_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_UINT64;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_int64(data_logging_t* data_logging, int64_t* val, const char* param_name)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_INT64;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_float(data_logging_t* data_logging, float* val, const char* param_name, uint32_t precision)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = (double*) val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_REAL32;
			new_param->precision				 = precision;
			
			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}

bool data_logging_add_parameter_double(data_logging_t* data_logging, double* val, const char* param_name, uint32_t precision)
{
	bool add_success = true;
	
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( (val == NULL)||(data_logging_set == NULL) )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
		
		add_success &= false;
	}
	else
	{
		if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
		{
			data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

			new_param->param					 = val;
			strcpy( new_param->param_name, 		 param_name );
			new_param->data_type                 = MAV_PARAM_TYPE_REAL64;
			new_param->precision				 = precision;

			data_logging_set->data_logging_count += 1;
			
			add_success &= true;
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}