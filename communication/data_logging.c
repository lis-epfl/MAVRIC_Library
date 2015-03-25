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

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Add in the file fp the first line with the name of the parameter
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_add_header_name(data_logging_t* data_logging, data_logging_set_t* data_set, data_logging_file_t* file);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The floating point number to be added
 * \param	after_digits			The number of digits after the floating point
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_float(FIL* fp, float c, int32_t after_digits);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 * \param	after_digits			The number of digits after the floating point
 *
 * \return	The result : 0 if it fails otherwise EOF (-1)
 */
static int32_t data_logging_put_double(FIL* fp, double c, int32_t after_digits);

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
static void data_logging_put_r_or_n(data_logging_t* data_logging, uint16_t param_num, data_logging_set_t* data_set, data_logging_file_t* file);

/**
 * \brief	Function to log a new line of values
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_log_parameters(data_logging_t* data_logging, data_logging_set_t* data_set, data_logging_file_t* file);

/**
 * \brief	Prints on debug port the result's value of the fatfs operation
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_print_error_signification(FRESULT fr);

/**
 * \brief	Seek the end of an open file to append
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_f_seek(data_logging_t* data_logging, data_logging_file_t* file);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void data_logging_add_header_name(data_logging_t* data_logging, data_logging_set_t* data_set, data_logging_file_t* file)
{
	bool init = false;
	
	uint16_t i;
	
	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		
		int32_t res = f_printf(&file->fil,param->param_name);
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
			data_logging_put_r_or_n(data_logging, i, data_set, file);
		}
	}
	file->file_init = init;
}

static int32_t data_logging_put_float(FIL* fp, float c, int32_t after_digits)
{
	int32_t i;
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

	int32_t whole=abs((int32_t)num);
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

static int32_t data_logging_put_double(FIL* fp, double c, int32_t after_digits)
{
	int32_t i;
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

static void data_logging_put_r_or_n(data_logging_t* data_logging, uint16_t param_num, data_logging_set_t* data_set, data_logging_file_t* file)
{
	int32_t res = 0;
	
	if (param_num == (data_set->data_logging_count-1))
	{
		res = f_puts("\n",&file->fil);
	}
	else
	{
		res = f_puts("\t",&file->fil);
	}
	if (res == EOF)
	{
		if (data_logging->debug)
		{
			file->fr = f_stat(file->name_n_extension,NULL);
			print_util_dbg_print("Error putting tab or new line character!\r\n");
			data_logging_print_error_signification(file->fr);
			print_util_dbg_print("\r\n");
		}
	}
}

static void data_logging_log_parameters(data_logging_t* data_logging, data_logging_set_t* data_set, data_logging_file_t* file)
{
	uint16_t i;
	uint32_t res = 0;
	
	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		switch(param->data_type)
		{
			case MAV_PARAM_TYPE_UINT8:
				res = f_printf(&file->fil, "%d", *((uint8_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_INT8:
				res = f_printf(&file->fil, "%d", *((int8_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_UINT16:
				res = f_printf(&file->fil, "%d", *((uint16_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_INT16:
				res = f_printf(&file->fil, "%d", *((int16_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
					
			case MAV_PARAM_TYPE_UINT32:
				res = f_printf(&file->fil, "%d", *((uint32_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_INT32:
				res = f_printf(&file->fil, "%d", *((int32_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_UINT64:
				res = data_logging_put_uint64_t(&file->fil,*((uint64_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
				
			case MAV_PARAM_TYPE_INT64:
				res = data_logging_put_int64_t(&file->fil,*((int64_t*)param->param));
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
					
			case MAV_PARAM_TYPE_REAL32:
				res = data_logging_put_float(&file->fil,*((float*)param->param),10);
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
					
			case MAV_PARAM_TYPE_REAL64:
				res = data_logging_put_double(&file->fil,*((double*)param->param),10);
				data_logging_put_r_or_n(data_logging,i,data_set,file);
				break;
			default:
				print_util_dbg_print("Data type not supported!\r\n");
		}
		if (res == EOF)
		{
			if (data_logging->debug)
			{
				file->fr = f_stat(file->name_n_extension,NULL);
				print_util_dbg_print("Error appending parameter! Error:");
				data_logging_print_error_signification(file->fr);
			}
			break;
		}
	}
}

static void data_logging_print_error_signification(FRESULT fr)
{
	switch(fr)
	{
		case FR_OK:
			print_util_dbg_print("FR_OK\r\n");
			break;
			
		case FR_DISK_ERR:
			print_util_dbg_print("FR_DISK_ERR\r\n");
			break;
			
		case FR_INT_ERR:
			print_util_dbg_print("FR_INT_ERR\r\n");
			break;
			
		case FR_NOT_READY:
			print_util_dbg_print("FR_NOT_READY\r\n");
			break;
			
		case FR_NO_FILE:
			print_util_dbg_print("FR_NO_FILE\r\n");
			break;
			
		case FR_NO_PATH:
			print_util_dbg_print("FR_NO_PATH\r\n");
			break;
			
		case FR_INVALID_NAME:
			print_util_dbg_print("FR_INVALID_NAME\r\n");
			break;
			
		case FR_DENIED:
			print_util_dbg_print("FR_DENIED\r\n");
			break;
			
		case FR_EXIST:
			print_util_dbg_print("FR_EXIST\r\n");
			break;
			
		case FR_INVALID_OBJECT:
			print_util_dbg_print("FR_INVALID_OBJECT\r\n");
			break;
			
		case FR_WRITE_PROTECTED:
			print_util_dbg_print("FR_WRITE_PROTECTED\r\n");
			break;
			
		case FR_INVALID_DRIVE:
			print_util_dbg_print("FR_INVALID_DRIVE\r\n");
			break;
			
		case FR_NOT_ENABLED:
			print_util_dbg_print("FR_NOT_ENABLED\r\n");
			break;
			
		case FR_NO_FILESYSTEM:
			print_util_dbg_print("FR_NO_FILESYSTEM\r\n");
			break;
			
		case FR_MKFS_ABORTED:
			print_util_dbg_print("FR_MKFS_ABORTED\r\n");
			break;
			
		case FR_TIMEOUT:
			print_util_dbg_print("FR_TIMEOUT\r\n");
			break;
			
		case FR_LOCKED:
			print_util_dbg_print("FR_LOCKED\r\n");
			break;
			
		case FR_NOT_ENOUGH_CORE:
			print_util_dbg_print("FR_NOT_ENOUGH_CORE\r\n");
			break;
			
		case FR_TOO_MANY_OPEN_FILES:
			print_util_dbg_print("FR_TOO_MANY_OPEN_FILES\r\n");
			break;
			
		case FR_INVALID_PARAMETER:
			print_util_dbg_print("FR_INVALID_PARAMETER\r\n");
			break;
			
		default:
			print_util_dbg_print("Error unknown\r\n");
			break;
	}
}

static void data_logging_f_seek(data_logging_t* data_logging, data_logging_file_t* file)
{
	/* Seek to end of the file to append data */
	file->fr = f_lseek(&file->fil, f_size(&file->fil));
	if (file->fr != FR_OK)
	{
		if (data_logging->debug)
		{
			print_util_dbg_print("lseek error:");
			data_logging_print_error_signification(file->fr);
		}
		f_close(&file->fil);
	}
}

static void data_logging_mount_sys(data_logging_t* data_logging)
{
	if ( (data_logging->fr != FR_OK)&&(data_logging->loop_count < 10) )
	{
		data_logging->loop_count += 1;
	}

	if (data_logging->loop_count < 10)
	{	
		data_logging->fr = f_mount(&data_logging->fs,"",1);
		
		if (data_logging->fr == FR_OK)
		{
			data_logging->sys_mounted = true;
		}
		else
		{
			data_logging->sys_mounted = false;
		}
	}
}

static void data_logging_unmount_sys(data_logging_t* data_logging)
{
	data_logging->fr = f_mount(&data_logging->fs,"",0);
	if (data_logging->debug)
	{
		print_util_dbg_print("f_(un)mount result:");
		data_logging_print_error_signification(data_logging->fr);
	}
	if (data_logging->fr == FR_OK)
	{
		data_logging->loop_count = 0;
		data_logging->sys_mounted = false;

		data_logging->data_all.file_opened = false;
		data_logging->data_all.file_init = false;
		data_logging->data_stat.file_opened = false;
		data_logging->data_stat.file_init = false;
	}
}

static void data_logging_close_file(data_logging_t* data_logging, data_logging_file_t* file)
{
	if (file->fr != FR_NO_FILE)
	{
		bool succeed = false;
		for (uint8_t i = 0; i < 5; ++i)
		{
			if (data_logging->debug)
			{
				print_util_dbg_print("Attempt to close file\r\n");
			}

			file->fr = f_close(&file->fil);
			
			if ( file->fr == FR_OK)
			{
				succeed = true;
				break;
			}

			if(file->fr == FR_NO_FILE)
			{
				break;
			}
		} //end for loop
		
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
	}//end of if (file->fr != FR_NO_FILE)
	
	file->file_opened = false;
	file->file_init = false;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool data_logging_init(data_logging_t* data_logging, const data_logging_conf_t* config, const state_t* state)
{
	bool init_success = true;
	
	// Init debug mode
	data_logging->debug = config->debug;
	
	data_logging->state = state;

	// Allocate memory for the onboard data_log
	data_logging->data_logging_set = malloc( sizeof(data_logging_set_t) + sizeof(data_logging_entry_t[config->max_data_logging_count]) );
	
	if ( data_logging->data_logging_set != NULL )
	{
		data_logging->data_logging_set->max_data_logging_count = config->max_data_logging_count;
		data_logging->data_logging_set->max_logs = config->max_logs;
		data_logging->data_logging_set->log_interval = config->log_interval;
		data_logging->data_logging_set->data_logging_count = 0;
		
		init_success &= true;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r\n");
		data_logging->data_logging_set->max_data_logging_count = 0;
		data_logging->data_logging_set->max_logs = 0;
		data_logging->data_logging_set->log_interval = 0;
		data_logging->data_logging_set->data_logging_count = 0;
		
		init_success &= false;
	}
	
	data_logging->data_logging_set_stat = malloc( sizeof(data_logging_set_t) + sizeof(data_logging_entry_t[config->max_data_logging_count]) );
	if ( data_logging->data_logging_set_stat != NULL )
	{
		data_logging->data_logging_set_stat->max_data_logging_count = config->max_data_logging_count;
		data_logging->data_logging_set_stat->max_logs = config->max_logs;
		data_logging->data_logging_set_stat->log_interval = config->log_interval;
		data_logging->data_logging_set_stat->data_logging_count = 0;
		
		init_success &= true;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r\n");
		data_logging->data_logging_set_stat->max_data_logging_count = 0;
		data_logging->data_logging_set_stat->max_logs = 0;
		data_logging->data_logging_set_stat->log_interval = 0;
		data_logging->data_logging_set_stat->data_logging_count = 0;
		
		init_success &= false;
	}

	// Automaticly add the time as first logging parameter
	data_logging_add_parameter_uint32(data_logging->data_logging_set,&data_logging->time_ms,"time");
	data_logging_add_parameter_uint32(data_logging->data_logging_set_stat,&data_logging->time_ms,"time");
	
	data_logging->data_all.file_init = false;
	data_logging->data_all.file_opened = false;
	data_logging->data_all.file_name_init = false;
	
	data_logging->data_stat.file_init = false;
	data_logging->data_stat.file_opened = false;
	data_logging->data_stat.file_name_init = false;

	data_logging->log_data = config->log_data;
	
	data_logging->loop_count = 0;
	
	#if _USE_LFN
	data_logging->buffer_name_size = _MAX_LFN;
	data_logging->buffer_add_size = _MAX_LFN;
	#else
	data_logging->buffer_name_size = 8;
	data_logging->buffer_add_size = 8;
	#endif
	
	// Deliberately initialized out of normal bound, to tell whether it is initialized or not
	// The value will then be set by the sys_id of the vehicle coming from the flash memory or from the hardcoded value
	data_logging->sys_id = 256;
	
	data_logging->data_all.file_name = malloc(data_logging->buffer_name_size);
	data_logging->data_all.name_n_extension = malloc(data_logging->buffer_name_size);

	data_logging->data_stat.file_name = malloc(data_logging->buffer_name_size);
	data_logging->data_stat.name_n_extension = malloc(data_logging->buffer_name_size);
	
	if ( (data_logging->data_all.file_name == NULL)||(data_logging->data_stat.file_name == NULL) )
	{
		init_success &= false;
	}
	if ( (data_logging->data_all.name_n_extension == NULL)||(data_logging->data_stat.name_n_extension == NULL) )
	{
		init_success &= false;
	}
	
	data_logging->sys_mounted = false;
	data_logging_mount_sys(data_logging);
	
	data_logging->write_stat = false;
	
	if (data_logging->debug)
	{
		if (data_logging->fr == FR_OK)
		{
			print_util_dbg_print("SD card mounted\r\n");
		}
		else
		{
			print_util_dbg_print("Mounting error:");
			data_logging_print_error_signification(data_logging->fr);
		}
	}
	data_logging->logging_time = time_keeper_get_millis();
	
	print_util_dbg_print("[DATA LOGGING] initialised.\r\n");
	
	return init_success;
}

bool data_logging_create_new_file(data_logging_t* data_logging, data_logging_file_t* file, const char* file_name, uint32_t sysid)
{
	bool create_success = true;
	
	int32_t i = 0;
	
	char *file_add = malloc(data_logging->buffer_add_size);
	
	if (file_add == NULL)
	{
		create_success &= false;
		print_util_dbg_print("[DATA LOGGING] Error: cannot allocate memory.\r\n");
	}
	
	data_logging->sys_id = sysid;
	
	if (!file->file_name_init)
	{
		snprintf(file->file_name, data_logging->buffer_name_size, "%s_%ld", file_name, sysid);
	}
	file->file_name_init = true;
	
	if (data_logging->log_data)
	{
		do 
		{
			if (i > 0)
			{
				if (snprintf(file->name_n_extension, data_logging->buffer_name_size, "%s%s.txt", file->file_name, file_add) >= data_logging->buffer_name_size)
				{
					print_util_dbg_print("Name error: The name is too long! It should be, with the extension, maximum ");
					print_util_dbg_print_num(data_logging->buffer_name_size,10);
					print_util_dbg_print(" and it is ");
					print_util_dbg_print_num(sizeof(file_name),10);
					print_util_dbg_print("\r\n");
					
					create_success &= false;
				}
			}
			else
			{
				if (snprintf(file->name_n_extension, data_logging->buffer_name_size, "%s.txt", file->file_name) >= data_logging->buffer_name_size)
				{
					print_util_dbg_print("Name error: The name is too long! It should be maximum ");
					print_util_dbg_print_num(data_logging->buffer_name_size,10);
					print_util_dbg_print(" characters and it is ");
					print_util_dbg_print_num(sizeof(file_name),10);
					print_util_dbg_print(" characters.\r\n");
					
					create_success &= false;
				}
			}
		
			file->fr = f_open(&file->fil, file->name_n_extension, FA_WRITE | FA_CREATE_NEW);
			
			if (data_logging->debug)
			{
				print_util_dbg_print("f_open result:");
				data_logging_print_error_signification(file->fr);
			}
		
			++i;
		
			if (file->fr == FR_EXIST)
			{

				if(snprintf(file_add,data_logging->buffer_add_size,"_%ld",i) >= data_logging->buffer_add_size)
				{
					print_util_dbg_print("Error file extension! Extension too long.\r\n");
					
					create_success &= false;
				}
			}
		
		//}while((i < data_logging->data_logging_set->max_data_logging_count)&&(data_logging->fr != FR_OK)&&(data_logging->fr != FR_NOT_READY));
		} while( (i < data_logging->data_logging_set->max_data_logging_count) && (file->fr == FR_EXIST) );
	
		if (file->fr == FR_OK)
		{
			data_logging_f_seek(data_logging,file);
		}
	
		if (file->fr == FR_OK)
		{
			file->file_opened = true;
		
			if (data_logging->debug)
			{
				print_util_dbg_print("File ");
				print_util_dbg_print(file->name_n_extension);
				print_util_dbg_print(" opened. \r\n");
				
				create_success &= true;
			}
		} //end of if data_logging->fr == FR_OK
	}//end of if (data_logging->log_data)
	
	return create_success;
}

task_return_t data_logging_update(data_logging_t* data_logging)
{
	if (data_logging->log_data == 1)
	{
		if (data_logging->data_all.file_opened)
		{
			if (data_logging->data_all.file_init)
			{
				data_logging->time_ms = time_keeper_get_millis();
				
				if (data_logging->state->mav_mode.ARMED == ARMED_OFF)
				{
					if ( (data_logging->time_ms - data_logging->logging_time) > 5000)
					{
						data_logging->data_all.fr = f_sync(&data_logging->data_all.fil);
						data_logging->logging_time = data_logging->time_ms;
					}
				}
				
				if (data_logging->data_all.fr == FR_OK)
				{
					data_logging_log_parameters(data_logging, data_logging->data_logging_set, &data_logging->data_all);
				}
			}
			else
			{
				if (data_logging->data_all.fr == FR_OK)
				{
					data_logging_add_header_name(data_logging,data_logging->data_logging_set,&data_logging->data_all);
				}
			}
		} //end of if (data_logging->file_opened)
		else
		{
			if (!data_logging->sys_mounted)
			{
				data_logging_mount_sys(data_logging);
			}

			if (!data_logging->data_all.file_name_init)
			{
				data_logging->data_all.file_name = "Default";
			}

			if (data_logging->sys_mounted)
			{
				data_logging_create_new_file(data_logging,&data_logging->data_all,data_logging->data_all.file_name,data_logging->sys_id);
			}
		}//end of else !data_logging->data_all.file_opened
		if (data_logging->data_stat.file_opened)
		{
			if (data_logging->data_stat.file_init)
			{
				if (data_logging->write_stat && (data_logging->data_stat.fr == FR_OK))
				{
					data_logging->write_stat = false;
					data_logging_log_parameters(data_logging, data_logging->data_logging_set_stat, &data_logging->data_stat);
					data_logging->data_stat.fr = f_sync(&data_logging->data_stat.fil);
				}
			}
			else
			{
				if (data_logging->data_stat.fr == FR_OK)
				{
					data_logging_add_header_name(data_logging,data_logging->data_logging_set_stat,&data_logging->data_stat);
				}
			}
		}
		else
		{
			if (!data_logging->sys_mounted)
			{
				data_logging_mount_sys(data_logging);
			}

			if (!data_logging->data_stat.file_name_init)
			{
				data_logging->data_stat.file_name = "Default";
			}

			if (data_logging->sys_mounted) 
			{
				data_logging_create_new_file(data_logging,&data_logging->data_stat,data_logging->data_stat.file_name,data_logging->sys_id);
			}
		}
	}//end of if (data_logging->log_data == 1)
	else
	{
		data_logging->loop_count = 0;

		if (data_logging->data_all.file_opened)
		{
			data_logging_close_file(data_logging,&data_logging->data_all);
		}

		if (data_logging->data_stat.file_opened)
		{
			data_logging_close_file(data_logging,&data_logging->data_stat);
		}

		if (data_logging->sys_mounted)
		{
			data_logging_unmount_sys(data_logging);
		}
	}//end of else (data_logging->log_data != 1)
	return TASK_RUN_SUCCESS;
}

bool data_logging_add_parameter_uint8(data_logging_set_t* data_logging_set, uint8_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_int8(data_logging_set_t* data_logging_set, int8_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_uint16(data_logging_set_t* data_logging_set, uint16_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_int16(data_logging_set_t* data_logging_set, int16_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_uint32(data_logging_set_t* data_logging_set, uint32_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_int32(data_logging_set_t* data_logging_set, int32_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_uint64(data_logging_set_t* data_logging_set, uint64_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_int64(data_logging_set_t* data_logging_set, int64_t* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_float(data_logging_set_t* data_logging_set, float* val, const char* param_name)
{
	bool add_success = true;
	
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

bool data_logging_add_parameter_double(data_logging_set_t* data_logging_set, double* val, const char* param_name)
{
	bool add_success = true;
	
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