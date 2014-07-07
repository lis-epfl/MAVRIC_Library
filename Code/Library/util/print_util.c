/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file print_util.c
 * 
 * Some utilities for printing strings and numbers 
 */


#include "print_util.h"

byte_stream_t* deb_stream;

Bool blocking;
static const char alphabet[36] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

/**
 * \brief	
 *
 *	\param	out_stream		Pointer to output stream
 *	\param	c				Number between 0 and 35
 *	\param	base			Base in which the number should be printed
 */
void putnum_tight(byte_stream_t *out_stream, int32_t c, char base);

byte_stream_t* print_util_get_debug_stream()
{
	return deb_stream;
}


void print_util_dbg_print_init(byte_stream_t* debug_stream)
{
	deb_stream=debug_stream;
}


void print_util_putstring(byte_stream_t *out_stream, const char* s) 
{
	if ((out_stream==NULL) || (out_stream->put==NULL)) 
	{
		return;
	}
	while (*s != 0) 
	{
		out_stream->put(out_stream->data, *s);
		s++;
	}
}


void print_util_putdigit(byte_stream_t *out_stream, uint32_t  c)
{
	if ((out_stream==NULL) || (out_stream->put==NULL)) 
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

	if ((out_stream==NULL) || (out_stream->put==NULL))
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


void print_util_putfloat(byte_stream_t *out_stream, float c, int32_t after_digits)
{
	int32_t i;
	float num = c;
	
	if (c<0) 
	{
		print_util_putstring(out_stream, "-");
		num=-c;
	} 
	else 
	{
		print_util_putstring(out_stream, "");
	}

	int32_t whole=abs((int32_t)num);
	float after=(num-(float)whole);

	putnum_tight(out_stream, whole, 10);
	print_util_putstring(out_stream, "."); 
	
	for (i=0; i<after_digits; i++) 
	{
		after*=10;
		print_util_putdigit(out_stream, (int32_t)after);
		after=after-(int32_t)after;
	}
}


void print_util_print_vector(byte_stream_t *out_stream, float v[], int32_t after_digits) 
{
	int32_t i;
	print_util_putstring(out_stream, "(");

	for (i=0; i<3; i++) 
	{
		print_util_putfloat(out_stream, v[i], after_digits);
		
		if (i<2) 
		{
			print_util_putstring(out_stream, ", ");
		}
	}

	print_util_putstring(out_stream, ") ");

}


void print_util_print_quaternion(byte_stream_t *out_stream, UQuat_t *quat, int32_t after_digits) 
{
	print_util_putstring(out_stream, "(");
	print_util_putfloat(out_stream, quat->s, after_digits);
	print_util_putstring(out_stream, ", ");
	print_util_print_vector(out_stream, quat->v, after_digits);
	print_util_putstring(out_stream, ") ");
}


void print_util_print_matrix(byte_stream_t *out_stream, float v[], int32_t rows, int32_t columns, int32_t after_digits) 
{
	int32_t i, j;
	
	for (i=0; i<rows; i++) 
	{
		print_util_putstring(out_stream, "| ");
		
		for (j=0; j<columns; j++) 
		{
			print_util_putfloat(out_stream, v[i*rows+j], after_digits);
			if (j<columns-1) 
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


void print_util_dbg_print_vector(float v[], int32_t after_digits) 
{
	print_util_print_vector(deb_stream, v, after_digits);
}


void print_util_dbg_print_quaternion(UQuat_t *quat, int32_t after_digits) 
{
	print_util_print_quaternion(deb_stream, quat, after_digits);
}


void print_util_dbg_log_value(const char* msg, int32_t value, char base) 
{
	print_util_dbg_print(msg);
	
	if (base>1) 
	{
		print_util_dbg_print_num(value, base);
	}
	
	print_util_dbg_print("\n");
}