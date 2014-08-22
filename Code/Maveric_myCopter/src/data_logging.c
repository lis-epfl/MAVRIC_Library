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
 * \file data_logging.c
 *
 *  Performs the data logging on the SD card
 */


#include "data_logging.h"
#include "FatFs/diskio.h"
#include "print_util.h"
#include "time_keeper.h"
#include "delay.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

uint32_t loop_count;

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

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void data_logging_add_header_name(data_logging_t* data_logging)
{
	bool init;
	
	uint16_t i;
	data_logging_set_t* data_set = data_logging->data_logging_set;
	
	if (data_logging->debug)
	{
		print_util_dbg_print("header: returns:");
	}
	
	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		
		int32_t res = f_printf(&data_logging->fil,param->param_name);
		if (res == EOF)
		{
			if (data_logging->debug)
			{
				print_util_dbg_print("Error appending header!\r");
			}
			init = false;
		}
		else
		{
			init = true;
			data_logging_put_r_or_n(data_logging, i);
		}
	}
	
	if (data_logging->debug)
	{
		print_util_dbg_print_num(init,10);
		print_util_dbg_print("\r");
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
			print_util_dbg_print("Error putting tab or new line character!\r");
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
		
		if (param->schedule_for_logging)
		{
			switch(param->data_type)
			{
				case MAV_PARAM_TYPE_UINT8:
					res = f_printf(&data_logging->fil, "%d", *param->param_uint8);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_INT8:
					res = f_printf(&data_logging->fil, "%d", *param->param_int8);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_UINT16:
					res = f_printf(&data_logging->fil, "%d", *param->param_uint16);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_INT16:
					res = f_printf(&data_logging->fil, "%d", *param->param_int16);
					data_logging_put_r_or_n(data_logging,i);
					break;
					
				case MAV_PARAM_TYPE_UINT32:
					res = f_printf(&data_logging->fil, "%ld", *param->param_uint32);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_INT32:
					res = f_printf(&data_logging->fil, "%ld", *param->param_int32);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_UINT64:
					res = data_logging_put_uint64_t(&data_logging->fil,*param->param_uint64);
					data_logging_put_r_or_n(data_logging,i);
					break;
				case MAV_PARAM_TYPE_INT64:
					res = data_logging_put_int64_t(&data_logging->fil,*param->param_int64);
					data_logging_put_r_or_n(data_logging,i);
					break;
					
				case MAV_PARAM_TYPE_REAL32:
					res = data_logging_put_float(&data_logging->fil,*param->param_float,10);
					data_logging_put_r_or_n(data_logging,i);
					break;
					
				case MAV_PARAM_TYPE_REAL64:
					res = data_logging_put_double(&data_logging->fil,*param->param_double,10);
					data_logging_put_r_or_n(data_logging,i);
					break;
				default:
					print_util_dbg_print("Data type not supported!\r");
			}
			if (res == EOF)
			{
				if (data_logging->debug)
				{
					print_util_dbg_print("Error appending parameter!\r");
				}
			}
		}
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
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r");
		data_logging->data_logging_set->max_data_logging_count = 0;
		data_logging->data_logging_set->data_logging_count = 0;
	}
	
	// Automaticly add the time as first logging parameter
	data_logging_add_parameter_uint32(data_logging,&data_logging->time_ms,"time");
	
	loop_count = 0;
	
	data_logging->file_init = false;
	data_logging->continue_writing = true;
	
	data_logging->fr = f_mount(&data_logging->fs, "", 1);
	
	if (data_logging->fr == FR_OK)
	{
		if (data_logging->debug)
		{
			print_util_dbg_print("SD card mounted\r");
		}
	}
}

void data_logging_create_new_log_file(data_logging_t* data_logging, const char* file_name)
{
	int32_t i = 0;
	
	char *file_add = "";
	char *name_n_extension = "";
	
	do 
	{
		sprintf(name_n_extension, "%s%s.txt", file_name,file_add);
		print_util_dbg_print(name_n_extension);
		print_util_dbg_print("\r");
		data_logging->fr = f_open(&data_logging->fil, name_n_extension, FA_WRITE | FA_CREATE_NEW);
		if (data_logging->fr == FR_EXIST)
		{
			//file_add = "";
			sprintf(file_add,"_%ld",i);
		}
		i++;
	}while((i<50)&&(data_logging->fr != FR_OK));
	
	
	if (data_logging->fr == FR_OK)
	{
		/* Seek to end of the file to append data */
		data_logging->fr = f_lseek(&data_logging->fil, f_size(&data_logging->fil));
		if (data_logging->fr != FR_OK)
		{
			f_close(&data_logging->fil);
		}
	}
	
	//data_logging->fr = diskio_open_append(&data_logging->fil,"test.txt",FA_WRITE | FA_CREATE_NEW);
	
	if (data_logging->fr == FR_OK)
	{
		if (data_logging->debug)
		{
			print_util_dbg_print("File opened\r");
		}
	}
}

task_return_t data_logging_run(data_logging_t* data_logging)
{
	if (data_logging->continue_writing)
	{
		if (data_logging->file_init)
		{
			loop_count++;
			if (loop_count == 5)
			{
				data_logging->continue_writing = false;
				loop_count = 0;
			}
			
			data_logging->time_ms = time_keeper_get_millis();
			
			data_logging_log_parameters(data_logging);
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
		if (loop_count < 10)
		{
			loop_count++;
			print_util_dbg_print("File closed\r");
		}
		
		data_logging->fr = f_close(&data_logging->fil);
	}
	return TASK_RUN_SUCCESS;
}

void data_logging_add_parameter_uint8(data_logging_t* data_logging, uint8_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_uint8               = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_UINT8;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_int8(data_logging_t* data_logging, int8_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_int8                = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_INT8;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_uint16(data_logging_t* data_logging, uint16_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_uint16              = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_UINT16;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_int16(data_logging_t* data_logging, int16_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_int16               = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_INT16;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_uint32(data_logging_t* data_logging, uint32_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_uint32              = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_UINT32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_int32(data_logging_t* data_logging, int32_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_int32               = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_INT32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_uint64(data_logging_t* data_logging, uint64_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_uint64              = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_UINT64;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_int64(data_logging_t* data_logging, int64_t* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_int64               = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_INT64;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_float(data_logging_t* data_logging, float* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_float               = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_REAL32;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}

void data_logging_add_parameter_double(data_logging_t* data_logging, double* val, const char* param_name)
{
	data_logging_set_t* data_logging_set = data_logging->data_logging_set;

	if( data_logging_set->data_logging_count < data_logging_set->max_data_logging_count )
	{
		data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

		new_param->param_double              = val;
		strcpy( new_param->param_name, 		 param_name );
		new_param->data_type                 = MAV_PARAM_TYPE_REAL64;
		new_param->param_name_length         = strlen(param_name);
		new_param->schedule_for_logging      = true;
		
		data_logging_set->data_logging_count += 1;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r");
	}
}