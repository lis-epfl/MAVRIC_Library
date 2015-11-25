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
 * \file  	file_fat_fs.cpp
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Class for files on avr32 platforms
 *
 ******************************************************************************/

#include "file_fat_fs.hpp"

extern "C"
{
	#include <stdlib.h>
	#include "print_util.h"
}

//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

<<<<<<< HEAD:hal/avr32/file_fat_fs.cpp
void File_fat_fs::mount_system()
=======
//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

bool fat_fs_mounting_init(fat_fs_mounting_t* fat_fs_mounting, data_logging_conf_t data_logging_conf, const State* state)
>>>>>>> dev_cpp:hal/common/fat_fs_mounting.cpp
{
	if (!sys_mounted)
	{
		print_util_dbg_print("Trying to mount SD card\r\n");

		if ((fr != FR_OK)&&(loop_count < 10))
		{
			loop_count += 1;
		}

		if (loop_count < 10)
		{
			fr = f_mount(&fs, "1:", 1);
			
			if (fr == FR_OK)
			{
				sys_mounted = true;
			}
			else
			{
				sys_mounted = false;
			}
			
			if (debug)
			{
				if (fr == FR_OK)
				{
					print_util_dbg_print("[FAT] SD card mounted\r\n");
				}
				else
				{
					print_util_dbg_print("[FAT] [ERROR] Mounting");
					print_error_signification(fr);
				}
			}
		}
	}
	else
	{
		print_util_dbg_print("System already mounted \r\n");
	}
}

bool File_fat_fs::unmount_system()
{
	bool success = false;

	if ( (num_file_opened == 0) && sys_mounted )
	{
		loop_count = 0;

		fr = f_mount(&fs,"1:",0);

		if (fr == FR_OK)
		{
			sys_mounted = false;
			success = true;
		}
		else
		{
			success = false;
		}

		if (debug)
		{
			if (fr == FR_OK)
			{
				print_util_dbg_print("[FAT] SD card unmounted. \r\n");
			}
			else
			{
				print_util_dbg_print("[FAT] [ERROR] Unmounting");
				print_error_signification(fr);
			}
		}
	}

	return success;
}

void File_fat_fs::print_error_signification(FRESULT fr)
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

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

File_fat_fs::File_fat_fs(bool debug_)
{
	fr = FR_NO_FILE;

	loop_count = 0;

	sys_mounted = false;

	num_file_opened = 0;

	debug = debug_;
}

bool File_fat_fs::open(const char* path, bool new_file)
{
	bool success = true;
	FRESULT fr;

	file_name = (char*)malloc(sizeof(path)+2);
	strcpy(file_name, "1:");
	strcat(file_name, path);

	if (debug)
	{
		print_util_dbg_print("Opening file:");
		print_util_dbg_print(file_name);
		print_util_dbg_print("\r\n");
	}

	mount_system();

	if (new_file)
	{
		fr = f_open(&file_, file_name, FA_WRITE | FA_CREATE_NEW);
	}
	else
	{
		fr = f_open(&file_, file_name, FA_WRITE | FA_OPEN_ALWAYS);
	}

	if (fr == FR_OK)
	{
		success = true;

		num_file_opened++;

	}
	else
	{
		if (debug)
		{
			print_util_dbg_print("Opening error:");
			print_error_signification(fr);
		}
		
		success = false;
	}

	return success;
}


bool File_fat_fs::is_open()
{
	bool success;

	if ( file_.fs != NULL )
	{
		success = true;
	}
	else
	{
		success = false;
	}

	return success;
}



bool File_fat_fs::close()
{
	bool success = true;
	FRESULT fr;

	fr = f_close(&file_);

	if (fr == FR_OK)
	{
		success = true;

		num_file_opened--;
	}
	else
	{
		if (debug)
		{
			print_util_dbg_print("Closing error:");
			print_error_signification(fr);
		}
		
		success = false;
	}

	success &= unmount_system();

	return success;
}



bool File_fat_fs::read(uint8_t* data, uint32_t size)
{
	bool success;
	FRESULT fr;

	uint32_t num_bytes_read;

	fr = f_read(&file_, data, size, (UINT*)&num_bytes_read);

	if (fr == FR_OK)
	{
		success = true;
	}
	else
	{
		if (debug)
		{
			print_util_dbg_print("Reading error:");
			print_error_signification(fr);
		}
		
		success = false;
	}

	return success;
}



bool File_fat_fs::write(const uint8_t* data, uint32_t size)
{
	bool success;
	int32_t fr = 1;
	FRESULT fr_stat;

	for (uint32_t i = 0; i < size; ++i)
	{
		fr = f_putc((char)*(data++),&file_);
		if (fr == EOF)
		{
			break;
		}
	}

	if (fr == EOF)
	{
		fr_stat = f_stat(file_name,NULL);
		if (debug)
		{
			print_util_dbg_print("Writing error:");
			print_error_signification(fr_stat);
		}

		success = false;
	}
	else
	{
		success = true;
	}

	return success;
}



bool File_fat_fs::seek(int32_t offset, file_seekfrom_t origin)
{
	bool success;
	FRESULT fr;
	
	switch( origin )
	{
		case FILE_SEEK_START:
			fr = f_lseek(&file_,offset);
		break;

  		case FILE_SEEK_CURRENT:
  			fr = f_lseek(&file_,file_.fptr+offset);
  		break;

  		case FILE_SEEK_END:
  			fr = f_lseek(&file_,f_size(&file_)+offset);
  		break;

  		default:
  			fr = f_lseek(&file_,f_size(&file_));
  		break;
	}

	if (fr == FR_OK)
	{
		success = true;
	}
	else
	{
		if (debug)
		{
			print_util_dbg_print("Seeking error:");
			print_error_signification(fr);
		}

		success = false;
	}

	return success;
}


uint32_t File_fat_fs::offset()
{
	DWORD fsize;

	fsize = f_tell(&file_);

	return fsize;
}


uint32_t File_fat_fs::length()
{
	DWORD fsize;

	fsize = f_size(&file_);

	return fsize;
}

bool File_fat_fs::flush()
{
	bool success = true;
	FRESULT fr;

	fr = f_sync(&file_);

	if (fr == FR_OK)
	{
		success = true;
	}
	else
	{
		if (debug)
		{
			print_util_dbg_print("Syncing error:");
			print_error_signification(fr);
		}
		
		success = false;	
	}

	return success;
}
