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
 * \file sd_mounting.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the mounting/unmounting of the SD card
 *
 ******************************************************************************/


#include "sd_mounting.h"
#include "print_util.h"
#include "time_keeper.h"

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Prints on debug port the result's value of the fatfs operation
 *
 * \param	sd_mounting			The pointer to the SD card mounting structure
 */
static void sd_mounting_print_error_signification(sd_mounting_t* sd_mounting);

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

static void sd_mounting_print_error_signification(sd_mounting_t* sd_mounting)
{
	switch(sd_mounting->fr)
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

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool sd_mounting_init(sd_mounting_t* sd_mounting, const data_logging_conf_t* data_logging_conf, const state_t* state)
{
	bool init_success = true;

	sd_mounting->data_logging_conf = *data_logging_conf;

	sd_mounting->log_data = data_logging_conf->log_data;
	
	sd_mounting->sys_mounted = false;
	sd_mounting->loop_count = 0;

	sd_mounting->state = state;

	sd_mounting->fr = FR_NO_FILE;

	sd_mounting_mount(sd_mounting, data_logging_conf->debug);

	sd_mounting->num_file_opened = 0;

	return init_success;
}

void sd_mounting_mount(sd_mounting_t* sd_mounting, bool debug)
{
	if (!sd_mounting->sys_mounted)
	{
		if ((sd_mounting->fr != FR_OK)&&(sd_mounting->loop_count < 10))
		{
			sd_mounting->loop_count += 1;
		}

		if (sd_mounting->loop_count < 10)
		{
			sd_mounting->fr = f_mount(&sd_mounting->fs, "", 1);
			
			if (sd_mounting->fr == FR_OK)
			{
				sd_mounting->sys_mounted = true;
			}
			else
			{
				sd_mounting->sys_mounted = false;
			}
			
			if (debug)
			{
				if (sd_mounting->fr == FR_OK)
				{
					print_util_dbg_print("SD card mounted\r\n");
				}
				else
				{
					print_util_dbg_print("Mounting error:");
					sd_mounting_print_error_signification(sd_mounting);
				}
			}
		}
	}
}

void sd_mounting_unmount(sd_mounting_t* sd_mounting, bool debug)
{
	if ( (sd_mounting->num_file_opened == 0) && sd_mounting->sys_mounted )
	{
		sd_mounting->loop_count = 0;

		sd_mounting->fr = f_mount(&sd_mounting->fs,"",0);

		if (sd_mounting->fr == FR_OK)
		{
			sd_mounting->sys_mounted = false;
		}

		if (debug)
		{
			if (sd_mounting->fr == FR_OK)
			{
				print_util_dbg_print("SD card unmounted. \r\n");
			}
			else
			{
				print_util_dbg_print("Unmounting error:");
				sd_mounting_print_error_signification(sd_mounting);
			}
		}
	}
}
