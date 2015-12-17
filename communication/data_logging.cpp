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
 * \file data_logging.cpp
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the data logging on a file
 *
 ******************************************************************************/


#include "data_logging.hpp"

#include <string>

extern "C"
{
	#include "print_util.h"
	#include "time_keeper.hpp"
	#include <stdbool.h>
	#include <stdlib.h>
	#include <math.h>
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void Data_logging::add_header_name(void)
{
	bool init = true;

	uint16_t i;
	data_logging_set_t* data_set = data_logging_set;
	
	init &= console.write("time");
	put_r_or_n(0);

	for (i = 0; i < data_set->data_logging_count; i++)
	{
		data_logging_entry_t* param = &data_set->data_log[i];

		init &= console.write(reinterpret_cast<uint8_t*>(param->param_name),strlen(param->param_name));

		if (!init)
		{
			break;
			if (debug)
			{
				print_util_dbg_print("Error appending header!\r\n");
			}
		}
		else
		{
			put_r_or_n(i);
		}
	}
	file_init = init;
}


void Data_logging::put_r_or_n(uint16_t param_num)
{
	bool success = true;

	// Writes a tab character or a end of line character to the file depending on the parameter current number
	if (param_num == (data_logging_set->data_logging_count-1))
	{
		success &= console.write("\n");
	}
	else
	{
		success &= console.write("\t");
	}
	if (!success)
	{
		if (debug)
		{
			print_util_dbg_print("Error putting tab or new line character!\r\n");
		}
	}
}


void Data_logging::log_parameters(void)
{
	uint32_t i;
	bool success = true;
	data_logging_set_t* data_set = data_logging_set;

	// First parameter is always time
	uint32_t time_ms = time_keeper_get_ms();
	success &=  console.write(time_ms);
	put_r_or_n(0);

	for (i = 0; i < data_set->data_logging_count; i++)
	{
		// Writing the value of the parameter to the file, separate values by tab character
		data_logging_entry_t* param = &data_set->data_log[i];
		switch(param->data_type)
		{
			case MAV_PARAM_TYPE_UINT8:
				success &= console.write(*((uint8_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_INT8:
				success &= console.write(*((int8_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_UINT16:
				success &= console.write(*((uint16_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_INT16:
				success &= console.write(*((int16_t*)param->param));
				put_r_or_n(i);
				break;
					
			case MAV_PARAM_TYPE_UINT32:
				success &= console.write(*((uint32_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_INT32:
				success &= console.write(*((int32_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_UINT64:
				success &= console.write(*((uint64_t*)param->param));
				put_r_or_n(i);
				break;
				
			case MAV_PARAM_TYPE_INT64:
				success &= console.write(*((int64_t*)param->param));
				put_r_or_n(i);
				break;
					
			case MAV_PARAM_TYPE_REAL32:
				success &= console.write(*(float*)param->param,param->precision);
				put_r_or_n(i);
				break;
					
			case MAV_PARAM_TYPE_REAL64:
				success &= console.write(*((double*)param->param),param->precision);
				put_r_or_n(i);
				break;
			default:
				success &= false;
				print_util_dbg_print("Data type not supported!\r\n");
				break;
		}

		if (!success)
		{
			if (debug)
			{
				print_util_dbg_print("Error appending parameter! Error:");
			}
			break;
		}
	}
}


void Data_logging::seek(void)
{
	bool success = true;

	/* Seek to end of the file to append data */
	success &= console.get_stream()->seek(0,FILE_SEEK_END);

	if (!success)
	{
		if (debug)
		{
			print_util_dbg_print("lseek error:");
		}
		// Closing the file if we could not seek the end of the file
		console.get_stream()->close();
	}
}


bool Data_logging::filename_append_extension(char* output, char* filename, uint32_t length)
{
	// Success flag
	bool is_success = true;

	// Declare counter for char location
	uint32_t i = 0;

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


bool Data_logging::filename_append_int(char* output, char* filename, uint32_t num, uint32_t length)
{
	// Success flag
	bool is_success = true;

	// Declare counter for char location
	uint32_t i = 0;

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
	uint32_t num_digits = 0;
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

bool Data_logging::checksum_control(void)
{
	bool new_values = false;
	
	double cksum_a_current = 0.0;
	double cksum_b_current = 0.0;

	float approx = 1.0f;

	uint32_t j = 0;

	data_logging_set_t* data_set = data_logging_set;
	for (uint32_t i = 0; i < data_set->data_logging_count; ++i)
	{
		data_logging_entry_t* param = &data_set->data_log[i];
		switch(param->data_type)
		{
			case MAV_PARAM_TYPE_UINT8:
				cksum_a_current += *((uint8_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_INT8:
				cksum_a_current += *((int8_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_UINT16:
				cksum_a_current += *((uint16_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_INT16:
				cksum_a_current += *((int16_t*)param->param);
				break;
					
			case MAV_PARAM_TYPE_UINT32:
				cksum_a_current += *((uint32_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_INT32:
				cksum_a_current += *((int32_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_UINT64:
				cksum_a_current += *((uint64_t*)param->param);
				break;
				
			case MAV_PARAM_TYPE_INT64:
				cksum_a_current += *((int64_t*)param->param);
				break;
					
			case MAV_PARAM_TYPE_REAL32:
				for (j = 0; j < param->precision; ++j)
				{
					approx *= 10;
				}
				approx = round((*((float*)param->param))*approx)/approx;
				cksum_a_current += approx;
				break;
					
			case MAV_PARAM_TYPE_REAL64:
				for (j = 0; j < param->precision; ++j)
				{
					approx *= 10;
				}
				approx = round((*((double*)param->param))*approx)/approx;
				cksum_a_current += approx;
				break;
			default:
				cksum_a_current = 0.0;
				cksum_b_current = 0.0;
				print_util_dbg_print("Data type not supported!\r\n");
				break;
		}
		cksum_b_current += cksum_a_current;
	}

	// 	print_util_dbg_print("cksum: (");
	// 	print_util_dbg_print_num(cksum_a_current*100,10);
	// 	print_util_dbg_print("==");
	// 	print_util_dbg_print_num(cksum_a*100,10);
	// 	print_util_dbg_print(")&&(");
	// 	print_util_dbg_print_num(cksum_b_current*100,10);
	// 	print_util_dbg_print("==");
	// 	print_util_dbg_print_num(cksum_b*100,10);
	// 	print_util_dbg_print(")\r\n");
	

	if ( (cksum_a_current == cksum_a)&&(cksum_b_current == cksum_b) )
	{
		new_values = false;
	}
	else
	{
		new_values = true;
		cksum_a = cksum_a_current;
		cksum_b = cksum_b_current;
	}

	return new_values;
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Data_logging::Data_logging(File& file):
console(file)
{
	;
}

bool Data_logging::create_new_log_file(const char* file_name_, bool continuous_write_, const toggle_logging_t* toggle_logging_, const State* state_, uint32_t sysid)
{
	bool init_success = true;
	
	const toggle_logging_conf_t* config = &toggle_logging_->toggle_logging_conf;

	toggle_logging = toggle_logging_;

	debug = config->debug;

	continuous_write = continuous_write_;

	state = state_;

	// Allocate memory for the onboard data_log
	data_logging_set = (data_logging_set_t*)malloc( sizeof(data_logging_set_t) + sizeof(data_logging_entry_t[config->max_data_logging_count]) );

	if ( data_logging_set != NULL )
	{
		data_logging_set->max_data_logging_count = config->max_data_logging_count;
		data_logging_set->max_logs = config->max_logs;
		data_logging_set->data_logging_count = 0;
		
		init_success &= true;
	}
	else
	{
		print_util_dbg_print("[DATA LOGGING] ERROR ! Bad memory allocation.\r\n");
		data_logging_set->max_data_logging_count = 0;
		data_logging_set->max_logs = 0;
		data_logging_set->data_logging_count = 0;
		
		init_success &= false;
	}
	
	file_init = false;
	file_opened = false;

	// Setting the maximal size of the name string
	#if _USE_LFN
	buffer_name_size = _MAX_LFN;
	#else
	#ifdef _MAX_LFN
	buffer_name_size = 8;
	#else
	buffer_name_size = 255;
	#endif
	#endif

	// Allocating memory for the file name string
	file_name = (char*)malloc(buffer_name_size);
	name_n_extension = (char*)malloc(buffer_name_size);
	
	if (file_name == NULL)
	{
		init_success &= false;
	}
	if (name_n_extension == NULL)
	{
		init_success &= false;
	}
	
	sys_id = sysid;

	// Append sysid to filename
	filename_append_int(file_name, (char*)file_name_, sysid, buffer_name_size);

	init_success &= open_new_log_file();

	logging_time = time_keeper_get_ms();

	cksum_a = 0.0;
	cksum_b = 0.0;

	return init_success;
}


bool Data_logging::open_new_log_file(void)
{
	bool create_success = true;

	uint32_t i = 0;
	
	if (toggle_logging->log_data)
	{
		do 
		{
			// Create flag for successfully written file names
			bool successful_filename = true;

			// Add iteration number to name_n_extension (does not yet have extension)
			successful_filename &= filename_append_int(name_n_extension, file_name, i, buffer_name_size);

			// Add extension (.txt) to name_n_extension
			successful_filename &= filename_append_extension(name_n_extension, name_n_extension, buffer_name_size);

			// Check if there wasn't enough memory allocated to name_n_extension
			if (!successful_filename)
			{
				print_util_dbg_print("Name error: The name is too long! It should be, with the extension, maximum ");
				print_util_dbg_print_num(buffer_name_size,10);
				print_util_dbg_print(" and it is ");
				print_util_dbg_print_num(sizeof(name_n_extension),10);
				print_util_dbg_print("\r\n");
				
				create_success = false;
			}

			// If the filename was successfully created, try to open a file
			if (successful_filename)
			{
				if (!console.get_stream()->exists(name_n_extension))
				{
					create_success = console.get_stream()->open(name_n_extension);
				}
				else
				{
					create_success = false;
				}
				
			}
			
			if (debug)
			{
				print_util_dbg_print("Open result:");
				print_util_dbg_print_num(create_success,10);
				print_util_dbg_print("\r\n");
			}

			++i;
		} while( (i < data_logging_set->max_data_logging_count) && (!create_success) );

		if (create_success)
		{
			seek();

			file_opened = true;

			if (debug)
			{
				print_util_dbg_print("File ");
				print_util_dbg_print(name_n_extension);
				print_util_dbg_print(" opened. \r\n");
			}
		} //end of if fr == FR_OK
	}//end of if (toggle_logging->log_data)

	return create_success;
}

bool Data_logging::update(void)
{
	uint32_t time_ms = 0;
	if (toggle_logging->log_data == 1)
	{
		if (file_opened)
		{
			if (!file_init)
			{
				add_header_name();
			}
			
			if ( !mav_modes_is_armed(state->mav_mode) )
			{
				time_ms = time_keeper_get_ms();
				if ( (time_ms - logging_time) > 5000)
				{
					console.get_stream()->flush();
					logging_time = time_ms;
				}
			}
			
			if (continuous_write)
			{
				log_parameters();
			}
			else
			{
				if (checksum_control())
				{
					log_parameters();
				}
			}

		} //end of if (file_opened)
		else
		{
			open_new_log_file();
			cksum_a = 0.0;
			cksum_b = 0.0;
		}//end of else if (file_opened)
	} //end of if (toggle_logging->log_data == 1)
	else
	{
		if (file_opened)
		{
			bool succeed = false;
			for (uint8_t i = 0; i < 5; ++i)
			{
				if (debug)
				{
					print_util_dbg_print("Attempt to close file ");
					print_util_dbg_print(name_n_extension);
					print_util_dbg_print("\r\n");
				}

				succeed = console.get_stream()->close();
				
				if (succeed)
				{
					break;
				}
			} //end for loop
			
			cksum_a = 0.0;
			cksum_b = 0.0;

			file_opened = false;
			file_init = false;

			if (debug)
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
		} //end of if (file_opened)

	} //end of else (toggle_logging->log_data != 1)
	
	return true;
}


bool Data_logging::add_field(uint8_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_UINT8_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(int8_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_INT8_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}

	return add_success;
}


bool Data_logging::add_field(uint16_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_UINT16_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(int16_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_INT16_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(uint32_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_UINT32_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(int32_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_INT32_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(uint64_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_UINT64_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(int64_t* val, const char* param_name)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_INT64_T;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(float* val, const char* param_name, uint32_t precision)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = (double*) val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_FLOAT;
				new_param->precision				 = precision;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}


bool Data_logging::add_field(double* val, const char* param_name, uint32_t precision)
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
			if (strlen(param_name) < MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN)
			{
				data_logging_entry_t* new_param = &data_logging_set->data_log[data_logging_set->data_logging_count];

				new_param->param					 = val;
				strcpy( new_param->param_name, 		 param_name );
				new_param->data_type                 = MAVLINK_TYPE_DOUBLE;
				new_param->precision				 = precision;

				data_logging_set->data_logging_count += 1;

				add_success &= true;
			}
			else
			{
				print_util_dbg_print("[DATA LOGGING] Error: Param name too long.\r\n");
			
				add_success &= false;
			}
		}
		else
		{
			print_util_dbg_print("[DATA LOGGING] Error: Cannot add more logging param.\r\n");
			
			add_success &= false;
		}
	}
	
	return add_success;
}
