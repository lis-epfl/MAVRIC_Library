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
 * \file  	file_flash_avr32.cpp
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Class to read and write to flash memory on avr32
 *
 ******************************************************************************/

#include "file_flash_avr32.hpp"


extern "C"
{
	#include "flashc.h"
}

#define MAVERIC_FLASHC_USER_PAGE_START_ADDRESS (AVR32_FLASHC_USER_PAGE_ADDRESS + 0x04)	// +4bytes for unknown reason
#define MAVERIC_FLASHC_USER_PAGE_FREE_SPACE 500	// 	512bytes user page, 
												//	-4bytes at the start, 
												//  -8bytes for the protected fuses at the end of the user page

/**
 * \brief	TODO: Modify the name of this structure to make it sized as the free flash memory to store these parameters
 */															
typedef struct												
{
	uint8_t values[MAVERIC_FLASHC_USER_PAGE_FREE_SPACE];
} nvram_data_t;


File_flash_avr32::File_flash_avr32(const char* path)
{
	offset_ = 0;
}


bool File_flash_avr32::open(const char* path, bool new_file)
{
	offset_ = 0;

	return true;
}


bool File_flash_avr32::is_open()
{
	return true;
}


bool File_flash_avr32::exists(const char* path)
{
	return true;
}


bool File_flash_avr32::close()
{
	offset_ = 0;

	return true;
}



bool File_flash_avr32::read(uint8_t* data, uint32_t size)
{
	bool success = true;

	// Get pointer to flash memory
	nvram_data_t* nvram_array = (nvram_data_t *) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;

	// Check if read will not overflow
	if( offset_ + size > MAVERIC_FLASHC_USER_PAGE_FREE_SPACE )
	{
		size = MAVERIC_FLASHC_USER_PAGE_FREE_SPACE - offset_;
		success = false;
	}

	// Copy data
	for( uint32_t i = 0; i < size; ++i)
	{
		data[i] = nvram_array->values[offset_ + i];
	}

	// Update offset
	offset_ += size; 

	return success;
}



bool File_flash_avr32::write(const uint8_t* data, uint32_t size)
{
	bool success = true;

	// Get pointer to flash memory
	nvram_data_t* nvram_array = (nvram_data_t *) MAVERIC_FLASHC_USER_PAGE_START_ADDRESS;

	// Check if read will not overflow
	if( offset_ + size > MAVERIC_FLASHC_USER_PAGE_FREE_SPACE )
	{
		size = MAVERIC_FLASHC_USER_PAGE_FREE_SPACE - offset_;
		success = false;
	}

	// Copy data
	flashc_memcpy((void*)&nvram_array->values[offset_], data, size, true);

	// Update offset
	offset_ += size; 

	return success;
}



bool File_flash_avr32::seek(int32_t offset, file_seekfrom_t origin)
{
	bool success;

	switch( origin )
	{
		case FILE_SEEK_START:
			offset_ = offset;
		break;

  		case FILE_SEEK_CURRENT:
			offset_ += offset;
  		break;

  		case FILE_SEEK_END:
  			offset_ = MAVERIC_FLASHC_USER_PAGE_FREE_SPACE + offset;
  		break;

  		default:
  			offset_ = offset;
  		break;
	}

	if( offset_ < 0 )
	{
		offset_ = 0;
		success = false;
	}
	else if( offset_ > MAVERIC_FLASHC_USER_PAGE_FREE_SPACE )
	{
		offset_ = MAVERIC_FLASHC_USER_PAGE_FREE_SPACE;
		success = false;
	}
	else
	{
		success = true;
	}

	return success;
}


uint32_t File_flash_avr32::offset()
{
	return offset_;
}


uint32_t File_flash_avr32::length()
{
	return MAVERIC_FLASHC_USER_PAGE_FREE_SPACE;
}

bool File_flash_avr32::flush()
{
	return true;
}