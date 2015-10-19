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
 * \file  	string_utils.hxx
 * 
 * \author  MAV'RIC Team
 *   
 * \brief   Implementation of template functions for string_util
 * 			(see string_util.cpp for non-template implementations)
 *
 ******************************************************************************/

namespace str{
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
	uint8_t* format_integer(T number, uint8_t* dest, uint8_t* length, uint8_t max_digits)
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


	/**
	 * \brief 	returns an array of ascii characters representing a floating number
	 *
	 * \param 	number 	Number to be put into the array
	 * \param	dest	Adress of the array to put it to (should be at least max_digits+1 long)
	 * \param	length 	Adress where the length of the array is written to
	 					(length including the sign and decimal point)
	 * \param	after_digits number of digits after the decimal point
	 * \param 	max_int_digits 	maximal number of digits before decimal point
	 * 
	 * \return 	new_dest 	new Address of the array (new_dest is a subarray of dest)
	 */
	template <typename T>
	uint8_t* format_floating(T num, uint8_t* dest, uint8_t* length, uint8_t after_digits, uint8_t max_int_digits)
	{
		bool is_negativ = false;
		if(num < 0)
		{
			is_negativ = true;
			num = num * -1;
		}

		/* write the integer part to char array 'dest' / 'data' */
		int32_t whole = floor(num);
		uint8_t i,j;
		uint8_t* data = format_integer(whole, dest+1, &i, max_int_digits);

		/* add minus sign to array if needed */
		if(is_negativ){
			data = data - 1;
			data[0] = '-';
			i++;
		}
		
		/* get floating point part */
		T after = (num-(T)whole);

		data[i++] = '.';
		for (j = 0; j < after_digits; j++) 
		{
			after *= 10;
			char digit = (char)after;
			data[i++] = digit + '0';
			after=after-digit;
		}

		/* calc length */
		*length = i;

		return data;
	}
};