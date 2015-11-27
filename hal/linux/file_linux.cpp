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
 * \file  	file_linux.cpp
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Class for files on linux platforms
 *
 ******************************************************************************/

#include "file_linux.hpp"

extern "C"
{
	#include "print_util.h"
}

using namespace std;

File_linux::File_linux()
{
	;
}

bool File_linux::open(const char* path)
{
	bool success = true;

	// Close if another file open
	if( file_.is_open() )
	{
		file_.close();
	}

	// Try opening file in input/output mode
	//file_.open(path, ios::in | ios::out | ios::binary | ios::ate );
	file_.open(path, ios::in | ios::out | ios::ate );

	// If it fails, create the file
	if( ~file_.is_open() )
	{
		// open in output mode
		file_.open(path, ios::out | ios::trunc);

		file_.close();

		// and reopen in input/output mode
		file_.open(path, ios::in | ios::out | ios::binary | ios::ate );
	}

	success = file_.is_open();

	return success;
}


bool File_linux::is_open()
{
	return file_.is_open();
}


bool File_linux::exists(const char* path)
{
	bool success = true;

	if( !file_.is_open() )
	{
		file_.open(path, ios::in | ios::out | ios::ate );
		
		if (file_.is_open())
		{
			success = true;
			file_.close();
		}
		else
		{
			success = false;
		}
	}
	else
	{
		success = true;
	}

	return success;
}


bool File_linux::close()
{
	bool success;

	file_.close();

	if (file_.is_open())
	{
		success = false;
	}
	else
	{
		success = true;
	}

	return success;
}



bool File_linux::read(uint8_t* data, uint32_t size)
{
	file_.read((char*)data, size);
	return true;
}



bool File_linux::write(const uint8_t* data, uint32_t size)
{
	file_.write((const char*)data, size);
	return true;
}



bool File_linux::seek(int32_t offset, file_seekfrom_t origin)
{
	ios::seekdir dir;
	
	switch( origin )
	{
		case FILE_SEEK_START:
			dir = ios::beg;
		break;

  		case FILE_SEEK_CURRENT:
  			dir = ios::cur;
  		break;

  		case FILE_SEEK_END:
  			dir = ios::end;
  		break;

  		default:
  			dir = ios::cur;
  		break;
	}

	file_.seekg(offset, dir);
	
	return true;
}


uint32_t File_linux::offset()
{
	streampos current;

	current = file_.tellg();

	return current;
}


uint32_t File_linux::length()
{
	streampos begin, end, current;
	current = file_.tellg();

	file_.seekg(0, ios::beg);
	begin = file_.tellg();

	file_.seekg(0, ios::end);
	end = file_.tellg();

	file_.seekg(current, ios::beg);

	return end - begin;
}

bool File_linux::flush()
{
	return true;
}
