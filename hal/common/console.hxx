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

#define MAX_DIGITS10_LONG 20




namespace{
	uint8_t myy_strlen(const char* text)
	{
		uint8_t i = 0;
		while(text[i] != '\0')
		{
			i++;
		}
		return i;
	}

	/**
	 * \brief 	returns an array of ascii characters representing an integer
	 *
	 * \param 	number 	Number to be put into the array
	 * \param	dest	Adress of the array to put it to (should be at least max_digits+1 long)
	 * \param	length 	Adress where the length of the array is written to (length including the sign)
	 * \param	max_digits 	maximal number of digits allowed (the rest is truncated)
	 * 
	 * \return 	new_dest 	new Address of the array (new_dest is a subarray of dest)
	 */
	template<typename T>
	uint8_t* format_integer(T number, uint8_t* dest, uint8_t* length, uint8_t max_digits = MAX_DIGITS10_LONG)
	{
		uint8_t i = max_digits+1;

		/* Take Care of the sign */
		bool is_negativ = false;
		if(number < 0)
		{
			is_negativ = true;
			number = number*-1;
		}

		do
		{
			dest[--i] = (number % 10) + '0';
			number = number / 10;
		} while((i >= 1) && (number > 0) );

		/* add sign to char* */
		if(is_negativ)
		{
			dest[--i] = '-';
		}

		*length = max_digits+1 -i;

		return dest + i;
	}
};

template <typename Writeable>
Console<Writeable>::Console(Writeable& stream):
	stream_(stream)
{
	;
}


template <typename Writeable>
bool Console<Writeable>::write(const uint8_t* data, uint32_t size)
{
 	return stream_.write(data, size);
}

template <typename Writeable>
template <typename T>
bool Console<Writeable>::write_integer(T number)
{
	uint8_t data_tmp[MAX_DIGITS10_LONG+1];
	uint8_t length;
	uint8_t* data = format_integer(number, data_tmp, &length);

	return stream_.write(data, length);
}

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
	uint8_t data_tmp[MAX_DIGITS10_LONG + 1 + after_digits + 1];
	uint8_t* data = format_integer(whole, data_tmp+1, &i);

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


template <typename Writeable>
bool Console<Writeable>::write(float number, uint8_t after_digits)
{
	return write_floating<float>(number, after_digits);
}

template <typename Writeable>
bool Console<Writeable>::write(double number, uint8_t after_digits)
{
	return write_floating(number, after_digits);
}

template <typename Writeable>
bool Console<Writeable>::write(uint32_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(int32_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(uint16_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(int16_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(uint8_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(int8_t number)
{
	return write_integer(number);
}

template <typename Writeable>
bool Console<Writeable>::write(int number)
{
	return write_integer(number);
}

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


template <typename Writeable>
bool Console<Writeable>::write(const char* text)
{
	uint8_t* data = (uint8_t*)text;
 	return stream_.write(data, myy_strlen(text));
}


// template <typename Writeable>
// void Console<Writeable>::flush()
// {}

// template<>
// void Console<Serial>::flush()
// {
// 	write("fl");
// 	stream_.flush();
// }

template <typename Writeable>
void Console<Writeable>::flush()
{
	stream_.flush();
}

template <typename Writeable>
template <typename T>
Console<Writeable>& Console<Writeable>::operator<<(const T &a)
{
	write(a);
	return *this;
}

template <typename Writeable>
Console<Writeable>& Console<Writeable>::operator<<(ConsoleManipulator manip)
{
	return manip(*this);
}

template <typename Writeable>
static Console<Writeable>& endl(Console<Writeable>& console)
{
	console.write("\n\r");
	console.flush();
	return console;
}

#endif /* CONSOLE_HXX_ */