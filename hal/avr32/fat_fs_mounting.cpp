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
 * \file fat_fs_mounting.c
 *
 * \author MAV'RIC Team
 * \author Nicolas Dousse
 *   
 * \brief Performs the mounting/unmounting of the fat_fs file system
 *
 ******************************************************************************/


#include "fat_fs_mounting.hpp"

extern "C"
{
	#include "print_util.h"
	#include "time_keeper.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool fat_fs_mounting_init(fat_fs_mounting_t* fat_fs_mounting, bool debug, FATFS* fs)
{
	bool init_success = true;

	fat_fs_mounting->fr = FR_NO_FILE;

	fat_fs_mounting->loop_count = 0;

	fat_fs_mounting->sys_mounted = false;

	fat_fs_mounting->fs = fs;

	fat_fs_mounting_mount(fat_fs_mounting, debug);

	return init_success;
}

void fat_fs_mounting_mount(fat_fs_mounting_t* fat_fs_mounting, bool debug)
{
	if (!fat_fs_mounting->sys_mounted)
	{
		print_util_dbg_print("Trying to mount SD card\r\n");
		if ((fat_fs_mounting->fr != FR_OK)&&(fat_fs_mounting->loop_count < 10))
		{
			fat_fs_mounting->loop_count += 1;
		}

		if (fat_fs_mounting->loop_count < 10)
		{
			fat_fs_mounting->fr = f_mount(fat_fs_mounting->fs, "1:", 1);
			//fat_fs_mounting->fr = f_mount(&fat_fs_mounting->fs, 0, 1);
			
			if (fat_fs_mounting->fr == FR_OK)
			{
				fat_fs_mounting->sys_mounted = true;
			}
			else
			{
				fat_fs_mounting->sys_mounted = false;
			}
			
			if (debug)
			{
				if (fat_fs_mounting->fr == FR_OK)
				{
					print_util_dbg_print("[FAT] SD card mounted\r\n");
				}
				else
				{
					print_util_dbg_print("[FAT] [ERROR] Mounting");
					fat_fs_mounting_print_error_signification(fat_fs_mounting->fr);
				}
			}
		}
	}
	else
	{
		print_util_dbg_print("System already mounted \r\n");
	}
}

void fat_fs_mounting_unmount(fat_fs_mounting_t* fat_fs_mounting, bool debug)
{
	if ( (fat_fs_mounting->num_file_opened == 0) && fat_fs_mounting->sys_mounted )
	{
		fat_fs_mounting->loop_count = 0;

		fat_fs_mounting->fr = f_mount(fat_fs_mounting->fs,"",0);

		if (fat_fs_mounting->fr == FR_OK)
		{
			fat_fs_mounting->sys_mounted = false;
		}

		if (debug)
		{
			if (fat_fs_mounting->fr == FR_OK)
			{
				print_util_dbg_print("[FAT] SD card unmounted. \r\n");
			}
			else
			{
				print_util_dbg_print("[FAT] [ERROR] Unmounting");
				fat_fs_mounting_print_error_signification(fat_fs_mounting->fr);
			}
		}
	}
}

void fat_fs_mounting_print_error_signification(FRESULT fr)
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
