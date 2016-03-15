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
 * \file    string_utils.cpp
 *
 * \author  MAV'RIC Team
 *
 * \brief   Collection of static functions to work with const char*
 *          (see string_util.hxx for implementation of templated functions)
 *
 ******************************************************************************/

#include "util/string_util.hpp"

namespace str
{

uint8_t strlen(const char* text)
{
    uint8_t i = 0;
    while (text[i] != '\0')
    {
        i++;
    }
    return i;
}

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
int8_t strcmp(const char* str1, const char* str2)
{
    int8_t d = 0;
    int i=0;
    do
    {
        d = str1[i] - str2[i];
        if(d != 0)
        {
            return d;
        }
    }while(str1[i++] != '\0');
    return 0;
}
};