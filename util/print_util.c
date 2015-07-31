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
 * \file print_util.c
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Some utilities for printing strings and numbers
 *
 ******************************************************************************/


#include "print_util.h"
#include <stdlib.h>
#include <stdbool.h>

byte_stream_t* deb_stream;

bool blocking;
static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	
 *
 *	\param	out_stream		Pointer to output stream
 *	\param	c				Number between 0 and 35
 *	\param	base			Base in which the number should be printed
 */
void putnum_tight(byte_stream_t *out_stream, int32_t c, char base);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


void putnum_tight(byte_stream_t *out_stream, int32_t c, char base)
{
	char storage[MAX_DIGITS];
	int32_t i = MAX_DIGITS;
	
	if ((out_stream==NULL) || (out_stream->put==NULL))
	{
		return;
	}

	/* Take Care of the sign */
	if(c < 0)
	{
		out_stream->put(out_stream->data, '-');
		c = c*-1;
	}

	do
	{
		i--;
		storage[i] = c % base;
		c = c / base;
	} while((i >= 0) && (c > 0) );

	/* i is the index of the last digit calculated */

	/* Hence, there is no need to initialize i */
	for( ; i<MAX_DIGITS; i++)
	{
		print_util_putdigit(out_stream, storage[i]);
	}
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


byte_stream_t* print_util_get_debug_stream()
{
	return deb_stream;
}


void print_util_dbg_print_init(byte_stream_t* debug_stream)
{
	deb_stream = debug_stream;
}


void print_util_putstring(byte_stream_t *out_stream, const char* s) 
{
	if ((out_stream == NULL) || (out_stream->put == NULL)) 
	{
		return;
	}
	while (*s != 0) 
	{
		out_stream->put(out_stream->data, *s);
		s++;
	}
}

void print_util_putstring_length(byte_stream_t *out_stream, const char* s, unsigned long len) 
{
	if ((out_stream == NULL) || (out_stream->put == NULL)) 
	{
		return;
	}
	while (len--) 
	{
		out_stream->put(out_stream->data, *s);
		s++;
	}
}


void print_util_putdigit(byte_stream_t *out_stream, uint32_t  c)
{
	if ((out_stream == NULL) || (out_stream->put == NULL)) 
	{
		return;
	}
		
	if (c > 35)
	{
	    return;
	}

	out_stream->put(out_stream->data,  alphabet[c]);
}


void print_util_putnum(byte_stream_t *out_stream, int32_t c, char base)
{
	char storage[MAX_DIGITS];
	int32_t i = MAX_DIGITS;

	if ((out_stream == NULL) || (out_stream->put == NULL))
	{
		return;
	}

	/* Take Care of the sign */
	if(c < 0)
	{
		out_stream->put(out_stream->data,   '-');
		c = c*-1;
	} 
	else 
	{
	  out_stream->put(out_stream->data,  ' ');
	}

	do
	{
		i--;
		storage[i] = c % base;
		c = c / base;
	} while((i >= 0) && (c > 0) );

	/* i is the index of the last digit calculated */

	/* Hence, there is no need to initialize i */
	for( ; i<MAX_DIGITS; i++)
	{
		print_util_putdigit(out_stream, storage[i]);
	}
}

void print_util_putlong(byte_stream_t *out_stream, int64_t c, char base)
{
	char storage[MAX_DIGITS_LONG];
	int32_t i = MAX_DIGITS_LONG;

	if ((out_stream == NULL) || (out_stream->put == NULL))
	{
		return;
	}

	/* Take Care of the sign */
	if(c < 0)
	{
		out_stream->put(out_stream->data,   '-');
		c = c*-1;
	}
	else
	{
		out_stream->put(out_stream->data,  ' ');
	}

	do
	{
		i--;
		storage[i] = c % base;
		c = c / base;
	} while((i >= 0) && (c > 0) );

	/* i is the index of the last digit calculated */

	/* Hence, there is no need to initialize i */
	for( ; i<MAX_DIGITS_LONG; i++)
	{
		print_util_putdigit(out_stream, storage[i]);
	}
}


void print_util_putfloat(byte_stream_t *out_stream, float c, int32_t after_digits)
{
	int32_t i;
	float num = c;
	
	if ( c < 0 ) 
	{
		print_util_putstring(out_stream, "-");
		num = -c;
	} 
	else 
	{
		print_util_putstring(out_stream, "");
	}

	int32_t whole = abs((int32_t)num);
	float after = (num-(float)whole);

	putnum_tight(out_stream, whole, 10);
	print_util_putstring(out_stream, "."); 
	
	for (i = 0; i < after_digits; i++) 
	{
		after *= 10;
		print_util_putdigit(out_stream, (int32_t)after);
		after=after-(int32_t)after;
	}
}


void print_util_print_vector(byte_stream_t *out_stream, float const v[], int32_t after_digits)
{
	print_util_putstring(out_stream, "(");

	for (int32_t i = 0; i < 3; i++) 
	{
		print_util_putfloat(out_stream, v[i], after_digits);
		
		if (i < 2) 
		{
			print_util_putstring(out_stream, ", ");
		}
	}

	print_util_putstring(out_stream, ") ");

}


void print_util_print_quaternion(byte_stream_t *out_stream, quat_t const *quat, int32_t after_digits) 
{
	print_util_putstring(out_stream, "(");
	print_util_putfloat(out_stream, quat->s, after_digits);
	print_util_putstring(out_stream, ", ");
	print_util_print_vector(out_stream, quat->v, after_digits);
	print_util_putstring(out_stream, ") ");
}


void print_util_print_matrix(byte_stream_t *out_stream, float const v[], int32_t rows, int32_t columns, int32_t after_digits) 
{
	int32_t i, j;
	
	for (i = 0; i < rows; i++) 
	{
		print_util_putstring(out_stream, "| ");
		
		for (j = 0; j < columns; j++) 
		{
			print_util_putfloat(out_stream, v[i*rows+j], after_digits);
			if (j < columns-1) 
			{
				print_util_putstring(out_stream, ", ");
			}
		}

		print_util_putstring(out_stream, " |\n");
	}
}


void print_util_dbg_print(const char* s) 
{
	print_util_putstring(deb_stream, s);
}


void print_util_dbg_print_num(int32_t c, char base) 
{
	print_util_putnum(deb_stream, c, base);
}


void print_util_dbg_putfloat(float c, int32_t after_digits) 
{
	print_util_putfloat(deb_stream, c, after_digits);
}


void print_util_dbg_print_vector(float const v[], int32_t after_digits) 
{
	print_util_print_vector(deb_stream, v, after_digits);
}


void print_util_dbg_print_quaternion(quat_t const *quat, int32_t after_digits) 
{
	print_util_print_quaternion(deb_stream, quat, after_digits);
}


void print_util_dbg_log_value(const char* msg, int32_t value, char base) 
{
	print_util_dbg_print(msg);
	
	if (base > 1) 
	{
		print_util_dbg_print_num(value, base);
	}
	
	print_util_dbg_print("\n");
}

void print_util_dbg_print_long(int64_t c, char base)
{
	print_util_putlong(deb_stream, c, base);
}