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
 * \file  	console.hxx
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Write to any write-able module in human-readable format
 *
 ******************************************************************************/

#ifndef CONSOLE_HXX_
#define CONSOLE_HXX_

#include "maths.h"
#include "string_util.hpp"



template <typename Writeable>
Console<Writeable>::Console(Writeable& stream):
	stream_(stream)
{
	;
}


/**
 * \brief 	Write buffer to the console
 *
 * \param 	data 	The buffer to write.
 * \param 	size 	The number of bytes to write.
 * 
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::write(const uint8_t* data, uint32_t size)
{
 	return stream_.write(data, size);
}

/**
 * \brief 	Write integer number to the console
 *
 * \param 	number 	integer number (uintX_t/intX_t)
 * 
 * \return 	success
 */
template <typename Writeable>
template <typename T>
bool Console<Writeable>::write(T number)
{
	uint8_t data_tmp[str::MAX_DIGITS10_LONG+1];
	uint8_t length;
	uint8_t* data = str::format_integer(number, data_tmp, &length);

	return stream_.write(data, length);
}

/**
 * \brief 	Write floating point to the console
 *
 * \param 	number 	floating point number (float/double)
 * \param 	after_digits 	digits after decimal point
 * 
 * \return 	success
 */
template <typename Writeable>
template <typename T>
bool Console<Writeable>::write_floating(T num, uint8_t after_digits)
{

	bool is_negativ = false;
	if(num < 0)
	{
		is_negativ = true;
		num = num * -1;
	}

	int32_t whole = floor(num);
	uint8_t i,j;
	uint8_t data_tmp[str::MAX_DIGITS10_LONG + 1 + after_digits + 1];
	uint8_t* data = str::format_integer(whole, data_tmp+1, &i);

	if(is_negativ){
		data = data - 1;
		data[0] = '-';
		i++;
	}
	
	float after = (num-(float)whole);

	data[i++] = '.';
	for (j = 0; j < after_digits; j++) 
	{
		after *= 10;
		char digit = (char)after;
		data[i++] = digit + '0';
		after=after-digit;
	}

	return write(data, i);
}


/**
 * \brief 	Write floating point to the console (wrapper for write_floating(..))
 *
 * \param 	number 	
 * 
 * \param 	after_digits 	digits after decimal point
 *
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::write(float number, uint8_t after_digits)
{
	return write_floating<float>(number, after_digits);
}

/**
 * \brief 	Write floating point to the console (wrapper for write_floating(..))
 *
 * \param 	number 	
 *
 * \param 	after_digits 	digits after decimal point
 * 
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::write(double number, uint8_t after_digits)
{
	return write_floating(number, after_digits);
}


/**
 * \brief 	Write bool to the console ("true"/"false")
 *
 * \param 	value 	boolean value to be evaluated
 * 
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::write(bool value)
{
	if(value)
	{
		const char* answer = "true";
		return write((uint8_t*)answer, 4);
	}else
	{
		const char* answer = "false";
		return write((uint8_t*)answer, 5);
	}
}

/**
 * \brief 	Write text to the console
 *
 * \param 	text 	Text to write to console
 * 
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::write(const char* text)
{
	uint8_t* data = (uint8_t*)text;
 	return stream_.write(data, str::strlen(text));
}

/**
 * \brief 	Write text to the console, append newline ('\n\r') and flush the stream
 *
 * \param 	text 	Text to write to console
 *
 * \return 	success
 */
template <typename Writeable>
bool Console<Writeable>::writeln(const char* text)
{
	bool result = write(text);
	endl(*this);
	return result;
}

/**
 * \brief 	Flushes the buffer of the console
 * 
 * \return 	success
 */
template <typename Writeable>
void Console<Writeable>::flush()
{
	stream_.flush();
}

/**
 * \brief operator to print like cout: console << "hello";
 *			calls write(data)
 *
 * \param data 	data to be printed
 *
 * \return 	success
 *
 */
template <typename Writeable>
template <typename T>
Console<Writeable>& Console<Writeable>::operator<<(const T &data)
{
	write(data);
	return *this;
}

/**
 * \brief applies the function pointed to by the argument and executes it with this as argument
 *			operator overload to work with endl function pointer
 *
 * \param	pointer to the function to be executed
 *
 * \return 	return itself (*this)
 */
template <typename Writeable>
Console<Writeable>& Console<Writeable>::operator<<(ConsoleManipulator manip)
{
	return manip(*this);
}

/**
 * \brief Writes line end ('\n\r') to stream and flushes the stream
 * 			used like 'console << "hello" << endl'
 *
 * \param console 	console to be written to
 *
 * \return console
 */
template <typename Writeable>
static Console<Writeable>& endl(Console<Writeable>& console)
{
	console.write("\n\r");
	console.flush();
	return console;
}

#endif /* CONSOLE_HXX_ */