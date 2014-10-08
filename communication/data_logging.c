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
static void data_logging_add_header_name(data_logging_t* data_logging);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The floating point number to be added
 * \param	after_digits			The number of digits after the floating point
 */
static int32_t data_logging_put_float(FIL* fp, float c, int32_t after_digits);

/**
 * \brief	Function to put a floating point number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 * \param	after_digits			The number of digits after the floating point
 */
static int32_t data_logging_put_double(FIL* fp, double c, int32_t after_digits);

/**
 * \brief	Function to put a uint64_t number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
 */
static int32_t data_logging_put_uint64_t(FIL* fp, uint64_t c);

/**
 * \brief	Function to put a int64_t number in the file fp
 *
 * \param	fp						The pointer to the file
 * \param	c						The double number to be added
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
 * \brief	Prints on debug port the result's value of the fatfs operation
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_print_error_signification(data_logging_t* data_logging);

/**
 * \brief	Seek the end of an open file to append
 *
 * \param	data_logging			The pointer to the data logging structure
 */
static void data_logging_f_seek(data_logging_t* data_logging);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void data_logging_add_header_name(data_logging_t* data_logging)
{
	bool init;
	
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

static int32_t data_logging_put_float(FIL* fp, float c, int32_t after_digits)
{
	int32_t i;
	int32_t res;
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
	int32_t res;
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
	int32_t res;
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
	int32_t res;
	
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
			data_logging_print_error_signification(data_logging);
			print_util_dbg_print("\r\n");
		}
	}
}

static void data_logging_log_parameters(data_logging_t* data_logging)
{
	uint16_t i;
	uint32_t res;
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
				res = data_logging_put_float(&data_logging->fil,*((float*)param->param),10);
				data_logging_put_r_or_n(data_logging,i);
				break;
					
			case MAV_PARAM_TYPE_REAL64:
				res = data_logging_put_double(&data_logging->fil,*((double*)param->param),10);
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
				data_logging_print_error_signification(data_logging);
			}
			break;
		}
	}
}

static void data_logging_print_error_signification(data_logging_t* data_logging)
{
	switch(data_logging->fr)
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

static void data_logging_f_seek(data_logging_t* data_logging)
{
	/* Seek to end of the file to append data */
	data_logging->fr = f_lseek(&data_logging->fil, f_size(&data_logging->fil));
	if (data_logging->fr != FR_OK)
	{
		if (data_logging->debug)
		{
			print_util_dbg_print("lseek error:");
			data_logging_print_error_signification(data_logging);
		}
		f_close(&data_logging->fil);
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void data_logging_init(data_logging_t* data_logging, const data_logging_conf_t* config)
{
	// Init debug mode
	data_logging->debug = config->debug;

	// Allocate memory for the onboard data_log
	data_logging->data_logging_set = malloc( sizeof(data_logging_set_t) + sizeof(data_logging_entry_t[config->max_data_logging_count]) );
	
	if ( data_logging->data_logging_set != NULL )
	{
		data_logging->data_logging_set->max_data_logging_count = config->max_data_logging_count;
		data_logging->data_logging_set->data_logging_count = 0;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r\n");
		data_logging->data_logging_set->max_data_logging_count = 0;
		data_logging->data_logging_set->data_logging_count = 0;
	}
	
	// Automaticly add the time as first logging parameter
	data_logging_add_parameter_uint32(data_logging,&data_logging->time_ms,"time");
	
	data_logging->file_init = false;
	data_logging->file_opened = false;
	data_logging->file_name_init = false;
	data_logging->log_data = config->log_data;
	
	data_logging->loop_count = 0;
	
	#if _USE_LFN
	data_logging->buffer_name_size = _MAX_LFN;
	data_logging->buffer_add_size = _MAX_LFN;
	#else
	data_logging->buffer_name_size = 8;
	data_logging->buffer_add_size = 8;
	#endif
	
	data_logging->file_name = malloc(data_logging->buffer_name_size);
	data_logging->name_n_extension = malloc(data_logging->buffer_name_size);
	
	data_logging->fr = f_mount(&data_logging->fs, "", 1);
	
	if (data_logging->fr == FR_OK)
	{
		data_logging->sys_mounted = true;
	}
	else
	{
		data_logging->sys_mounted = false;
	}
	
	if (data_logging->debug)
	{
		if (data_logging->fr == FR_OK)
		{
			print_util_dbg_print("SD card mounted\r\n");
		}
		else
		{
			print_util_dbg_print("Mounting error:");
			data_logging_print_error_signification(data_logging);
		}
	}
	data_logging->logging_time = time_keeper_get_millis();
	

	
	print_util_dbg_print("[Data logging] Data logging initialised.\r\n");
}

void data_logging_create_new_log_file(data_logging_t* data_logging, const char* file_name)
{
	int32_t i = 0;
	
	char *file_add = malloc(data_logging->buffer_add_size);
	
	snprintf(data_logging->file_name, data_logging->buffer_name_size, "%s", file_name);
	data_logging->file_name_init = true;
	
	if (data_logging->log_data)
	{
		do 
		{
			if (i > 0)
			{
				if (snprintf(data_logging->name_n_extension, data_logging->buffer_name_size, "%s%s.txt", file_name, file_add) >= data_logging->buffer_name_size)
				{
					print_util_dbg_print("Name error: The name is too long! It should be, with the extension, maximum ");
					print_util_dbg_print_num(data_logging->buffer_name_size,10);
					print_util_dbg_print(" and it is ");
					print_util_dbg_print_num(sizeof(file_name),10);
					print_util_dbg_print("\r\n");
				}
			}
			else
			{
				if (snprintf(data_logging->name_n_extension, data_logging->buffer_name_size, "%s.txt", file_name) >= data_logging->buffer_name_size)
				{
					print_util_dbg_print("Name error: The name is too long! It should be maximum ");
					print_util_dbg_print_num(data_logging->buffer_name_size,10);
					print_util_dbg_print(" characters and it is ");
					print_util_dbg_print_num(sizeof(file_name),10);
					print_util_dbg_print(" characters.\r\n");
				}
			}
		
			data_logging->fr = f_open(&data_logging->fil, data_logging->name_n_extension, FA_WRITE | FA_CREATE_NEW);
			
			if (data_logging->debug)
			{
				print_util_dbg_print("f_open result:");
				data_logging_print_error_signification(data_logging);
			}
		
			++i;
		
			if (data_logging->fr == FR_EXIST)
			{

				if(snprintf(file_add,data_logging->buffer_add_size,"_%ld",i) >= data_logging->buffer_add_size)
				{
					print_util_dbg_print("Error file extension! Extension too long.\r\n");
				}
			}
		
		//}while((i < MAX_NUMBER_OF_LOGGED_FILE)&&(data_logging->fr != FR_OK)&&(data_logging->fr != FR_NOT_READY));
		} while( (i < MAX_NUMBER_OF_LOGGED_FILE) && (data_logging->fr == FR_EXIST) );
	
		if (data_logging->fr == FR_OK)
		{
			data_logging_f_seek(data_logging);
		}
	
		if (data_logging->fr == FR_OK)
		{
			data_logging->file_opened = true;
		
			if (data_logging->debug)
			{
				print_util_dbg_print("File ");
				print_util_dbg_print(data_logging->name_n_extension);
				print_util_dbg_print(" opened. \r\n");
			}
		}
	}
}

task_return_t data_logging_update(data_logging_t* data_logging)
{
	if (data_logging->log_data == 1)
	{
		if (data_logging->file_opened)
		{
			if (data_logging->file_init)
			{
				data_logging->time_ms = time_keeper_get_millis();
				
				if (data_logging->fr == FR_OK)
				{
					data_logging_log_parameters(data_logging);
				}
			}
			else
			{
				if (data_logging->fr == FR_OK)
				{
					data_logging_add_header_name(data_logging);
				}
			}
		}
		else
		{
			if (!data_logging->file_name_init)
			{
				data_logging->file_name = "Default";
			}
			
			if ((data_logging->fr != FR_OK)&&(data_logging->loop_count < 10))
			{
				data_logging->loop_count += 1;
			}
			
			if (data_logging->loop_count < 10)
			{	
				data_logging->fr = f_mount(&data_logging->fs,"",1);
				
				if (data_logging->fr == FR_OK)
				{
					data_logging->sys_mounted = true;
					data_logging_create_new_log_file(data_logging,data_logging->file_name);
				}
				else
				{
					data_logging->sys_mounted = false;
				}
			}
		}
	}
	else
	{
		data_logging->loop_count = 0;
		if (data_logging->file_opened)
		{
			if (data_logging->fr != FR_NO_FILE)
			{
				bool succeed = false;
				for (uint8_t i = 0; i < 5; ++i)
				{
					if (data_logging->debug)
					{
						print_util_dbg_print("Attempt to close file\r\n");
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
				}
					
				data_logging->file_opened = false;

				
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
			}
			else
			{
				data_logging->file_opened = false;
			}
		}
		if (data_logging->sys_mounted)
		{
			data_logging->fr = f_mount(&data_logging->fs,"",0);
			if (data_logging->debug)
			{
				print_util_dbg_print("f_(un)mount result:");
				data_logging_print_error_signification(data_logging);
			}
			if (data_logging->fr == FR_OK)
			{
				data_logging->file_opened = false;
				data_logging->sys_mounted = false;
			}
		}
		
	}
	return TASK_RUN_SUCCESS;
}

void data_logging_add_parameter_uint8(data_logging_t* data_logging, uint8_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_int8(data_logging_t* data_logging, int8_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_uint16(data_logging_t* data_logging, uint16_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_int16(data_logging_t* data_logging, int16_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_uint32(data_logging_t* data_logging, uint32_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_int32(data_logging_t* data_logging, int32_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_uint64(data_logging_t* data_logging, uint64_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_int64(data_logging_t* data_logging, int64_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_float(data_logging_t* data_logging, float* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}

void data_logging_add_parameter_double(data_logging_t* data_logging, double* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;
	
	if( val == NULL )
	{
		print_util_dbg_print("[DATA LOGGING] Error: Null pointer!");
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
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
		}
	}
}