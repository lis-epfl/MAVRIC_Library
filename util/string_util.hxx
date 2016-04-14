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
 * \file    string_utils.hxx
 *
 * \author  MAV'RIC Team
 *
 * \brief   Implementation of template functions for string_util
 *          (see string_util.cpp for non-template implementations)
 *
 ******************************************************************************/

#include "util/maths.h"

namespace str
{

template<typename T>
uint8_t* format_integer(T number, uint8_t* dest, uint8_t* length, uint8_t max_digits)
{
    uint8_t i = max_digits + 1;

    /* Take Care of the sign */
    bool is_negativ = false;
    if (number < 0)
    {
        is_negativ = true;
        number = number * -1;
    }

    do
    {
        dest[--i] = (number % 10) + '0';
        number = number / 10;
    }
    while ((i >= 1) && (number > 0));

    /* add sign to char* */
    if (is_negativ)
    {
        dest[--i] = '-';
    }

    *length = max_digits + 1 - i;

    return dest + i;
}


template <typename T>
uint8_t* format_floating(T num, uint8_t* dest, uint8_t* length, uint8_t after_digits, uint8_t max_int_digits)
{
    bool is_negativ = false;
    if (num < 0)
    {
        is_negativ = true;
        num = num * -1;
    }

    /* write the integer part to char array 'dest' / 'data' */
    int32_t whole = floor(num);
    uint8_t i, j;
    uint8_t* data = format_integer(whole, dest + 1, &i, max_int_digits);

    /* add minus sign to array if needed */
    if (is_negativ)
    {
        data = data - 1;
        data[0] = '-';
        i++;
    }

    /* get floating point part */
    T after = (num - (T)whole);

    data[i++] = '.';
    for (j = 0; j < after_digits; j++)
    {
        after *= 10;
        char digit = (char)after;
        data[i++] = digit + '0';
        after = after - digit;
    }

    /* calc length */
    *length = i;

    return data;
}


template <typename T>
uint8_t* format_scientific(T num, uint8_t* dest, uint8_t* length, uint8_t after_digits)
{
    // Determine if the number is less than or equal to 1 or
    // greater than or equal to 10
    bool is_greater_10 = false;
    bool is_less_1 = false;

    // Multiply factor, for bringing the number to scientific
    // Notation
    float multiply_factor = 1.0f;

    // Count for number of decimal moves
    int decimal_moves = 0;

    // If number is exactly 0, do nothing
    if (num == 0)
    {

    }
    else if (num >= 10 || num <= -10) // If the number is greater than 10
    {
        is_greater_10 = true;
        multiply_factor = 0.1f;
    }
    else if (num < 1 && num > -1) // If the number is less than 1
    {
        is_less_1 = false;
        multiply_factor = 10.0f;
    } // If the number is between 10 and 1, do nothing

    // Bring number to one digit before decimal, if not already
    while (num >= 10 || num <= -10 || (num < 1 && num > -1 && num != 0))
    {
        // Multiply
        num = num * multiply_factor;

        // Update counter
        is_greater_10 ? decimal_moves++ : decimal_moves--;
    }

    // Get floating point string without the E and the rest
    uint8_t* data = format_floating(num, dest, length, after_digits, 1);

    // Append E and the exponent
    data[*length] = 'E';
    int decimal_moves_original = decimal_moves; // Create copy as decimal_moves will go to 0
    uint8_t exp_length = 0; // Determine how long the exponent should be
    if (decimal_moves_original < 0) // If there is a negative exponent, we need to add 1 for the length
    {
        exp_length++;
    }
    while (decimal_moves != 0) // Add one to length for each digit
    {
        exp_length++;
        decimal_moves = decimal_moves / 10;
    }
    if (exp_length == 0) // If decimal moves is still 0
    {
        exp_length++; // Make exponent length one
    }
    uint8_t* exp_length_ptr = &exp_length; // Point to the number of digits in exponent
    format_integer(decimal_moves_original, data + *length, exp_length_ptr, exp_length);

    // Update length and dest
    *length = *length + 1 + exp_length; // Add the E and the exponent to length

    return data; // Return data
}
};