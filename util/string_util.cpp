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

uint64_t strlen(const char* text)
{
    if(text == NULL)
    {
        return 0;
    }
    char* t = const_cast<char*>(text);
    while(*(t++) != '\0');
    return t - text - 1;
}


int16_t strcmp(const char* str1, const char* str2)
{
    int16_t d = 0;
    char* s1 = const_cast<char*>(str1);
    char* s2 = const_cast<char*>(str2);
    do
    {
        d = *(s1++) - *(s2++);
        if(d != 0)
        {
            return d;
        }
    }while(*s1 != '\0');
    return 0;
}

bool strlcpy(char* dst, const char* src, uint16_t max_len)
{
    char* s = const_cast<char*>(src);
    while(*s != '\0')
    {
        if(--max_len <= 0)
        {
            break;
        }
        *(dst++) = *(s++);
    }
    *dst = '\0';
    return max_len > 0;
}
};