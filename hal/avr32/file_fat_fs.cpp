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
 * \brief   Class for files on linux platforms
 *
 ******************************************************************************/

#include "file_fat_fs.hpp"

extern "C"
{
	#include <stdlib.h>
	#include "print_util.h"
}

//using namespace fa;

File_fat_fs::File_fat_fs()
{
	
}

void File_fat_fs::init(fat_fs_mounting_t* fat_fs_mounting)
{
	fat_fs_mounting_ = fat_fs_mounting;

	fat_fs_mounting_init(fat_fs_mounting_,true);
}

bool File_fat_fs::open(const char* path, bool new_file)
{
	bool success = true;
	FRESULT fr;

	file_name = (char*)malloc(sizeof(path)+2);
	strcpy(file_name, "1:");
	strcat(file_name, path);

	// file_name = (char*)malloc(sizeof(path));
	// strcpy(file_name,path);

	print_util_dbg_print("Opening file:");
	print_util_dbg_print(file_name);
	print_util_dbg_print("\r\n");
	

	fat_fs_mounting_mount(fat_fs_mounting_,true);

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

		fat_fs_mounting_->num_file_opened++;

	}
	else
	{
		print_util_dbg_print("Opening error:");
		fat_fs_mounting_print_error_signification(fr);
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

		fat_fs_mounting_->num_file_opened--;
	}
	else
	{
		fat_fs_mounting_print_error_signification(fr);
		success = false;
	}

	success &= fat_fs_mounting_unmount(fat_fs_mounting_,true);

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
		fat_fs_mounting_print_error_signification(fr);
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
		success = false;
		fr_stat = f_stat(file_name,NULL);
		fat_fs_mounting_print_error_signification(fr_stat);
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
		success = false;
		fat_fs_mounting_print_error_signification(fr);
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

bool File_fat_fs::sync()
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
		success = false;
		fat_fs_mounting_print_error_signification(fr);
	}

	return success;
}
