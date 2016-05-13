/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file    string_utils.hpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Collection of static functions to work with const char*
 *
 ******************************************************************************/

#ifndef STRING_UTIL_HPP_
#define STRING_UTIL_HPP_

#include <stdint.h>
#include <stddef.h>
 
namespace str
{
const uint8_t MAX_DIGITS10_LONG = 20;
const uint8_t MAX_AFTERDIGITS = 255;
const uint8_t FLOAT_DIGIT_COUNT = 3;

/**
 * \brief   returns the length of a const char* excluding the termination character '\0'
 *
 * \details checks for NULL pointer: returns 0 if text == NULL
 *
 * \param   max_digits  maximal number of digits allowed (the rest is truncated)
 *
 * \return  length  number of characters excluding the termination character '\0'
 */
uint64_t strlen(const char* text);


/**
 * \brief   compares two null terminated strings
 *
 * \detail  reproduces the behaviour of the function strcmp of cstring
 *
 * \param   str1:   string to compare
 * \param   str2:   string to compare
 *
 * \return  0 if the strings are equal
 *          <0 the first character that does not match has a lower value in ptr1 than in ptr2;
 *          >0 the first character that does not match has a greater value in ptr1 than in ptr2
 */
int16_t strcmp(const char* str1, const char* str2);


/**
 * \brief   copys a string with maximal length of max_len (including '\0')
 *          if src is longer, dst is truncated
 *
 * \detail  reproduces behavior of FreeBSD strlcpy except for return
 *
 * \param   dst:    pointer to destination
 * \param   src:    pointer to source
 * \param   max_len: maximal length to be copied (INCLUDING '\0')
 * \return  true dst == src, false if truncated
 */
bool strlcpy(char* dst, const char* src, uint16_t max_len);


/**
 * \brief   returns an array of ascii characters representing an integer
 *
 * \param   number  Number to be put into the array
 * \param   dest    Adress of the array to put it to (should be at least max_digits+1 long)
 * \param   length  Adress where the length of the array is written to (length including the sign)
 * \param   max_digits  maximal number of digits allowed (the rest is truncated)
 *
 * \return  new_dest    new Address of the array (new_dest is a subarray of dest)
 */
template<typename T>
uint8_t* format_integer(T number, uint8_t* dest, uint8_t* length, uint8_t max_digits = MAX_DIGITS10_LONG);


/**
 * \brief   returns an array of ascii characters representing a floating number
 *
 * \param   number  Number to be put into the array
 * \param   dest    Adress of the array to put it to (should be at least max_digits+1 long)
 * \param   length  Adress where the length of the array is written to
                    (length including the sign and decimal point)
 * \param   after_digits number of digits after the decimal point
 * \param   max_int_digits  maximal number of digits before decimal point
 *
 * \return  new_dest    new Address of the array (new_dest is a subarray of dest)
 */
template <typename T>
uint8_t* format_floating(T num, uint8_t* dest, uint8_t* length, uint8_t after_digits = FLOAT_DIGIT_COUNT, uint8_t max_int_digits = MAX_DIGITS10_LONG);

/**
* \brief    returns an array of ascii characters representing a number in scientific notation
*
* \param    num     Number to be put into the array
* \param    dest    Adress of the array to put it to (should be at least max_digits+3+x long,
*                   where x is the number of digits of the exponent)
* \param    length  Adress where the length of the array is written to
*                   (length including the sign and decimal point)
* \param    after_digits number of digits after the decimal point but before E
*
* \return   new_dest    new Address of the array (new_dest is a subarray of dest)
*/
template <typename T>
uint8_t* format_scientific(T num, uint8_t* dest, uint8_t* length, uint8_t after_digits);

};

#include "string_util.hxx"

#endif /* STRING_UTIL_HPP_ */
