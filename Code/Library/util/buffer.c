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
 * \file buffer.c
 * 
 * Buffer
 */ 


#include "buffer.h"
#include <stdbool.h>
#include "compiler.h"

uint8_t buffer_full(Buffer_t * buffer) 
{
	return (((buffer->BufferHead + 1)&BUFFER_MASK) == buffer->BufferTail);
}


uint8_t buffer_put_lossy(Buffer_t * buffer, uint8_t byte) 
{
	uint8_t tmp;
	tmp = (buffer->BufferHead + 1)&BUFFER_MASK;

	if (tmp==buffer->BufferTail) 
	{
		// error: receive buffer overflow!!
		// lose old incoming data at the end of the buffer
		buffer->BufferTail=(buffer->BufferTail + 1)&BUFFER_MASK;
	}

	// store incoming data in buffer
	buffer->Buffer[buffer->BufferHead] = byte;
	buffer->BufferHead=tmp;
	
	if (buffer_full(buffer)) 
	{
		buffer->full = 1;
	}
	else 
	{
		buffer->full = 0;
	}
	
	return 0;
}


uint8_t buffer_put(Buffer_t * buffer, uint8_t byte) 
{
	uint8_t tmp;
	tmp = (buffer->BufferHead + 1)&BUFFER_MASK;

	if (tmp==buffer->BufferTail) 
	{
		//error: buffer full! return 1
		return 1;
	}
	else
	{
		// store incoming data in buffer
		buffer->Buffer[buffer->BufferHead] = byte;
		buffer->BufferHead=tmp;
	
		if (buffer_full(buffer)) 
		{
			buffer->full = 1;
		}	 
		else 
		{
			buffer->full = 0;
		}
		
		return 0;
	}
}


uint8_t buffer_get(Buffer_t * buffer) 
{
	uint8_t ret=0;
	
	if (buffer->BufferHead!=buffer->BufferTail)
	{
		ret = buffer->Buffer[buffer->BufferTail];
		buffer->BufferTail =  (buffer->BufferTail + 1)&BUFFER_MASK;
		buffer->full = 0;
	}

	return ret;
}


int8_t buffer_empty(Buffer_t * buffer) 
{
	return (buffer->BufferHead==buffer->BufferTail);
}


uint32_t buffer_bytes_available(Buffer_t * buffer) 
{
	return (BUFFER_SIZE + buffer->BufferHead - buffer->BufferTail)&BUFFER_MASK;
}


void buffer_init(Buffer_t * buffer) 
{
	buffer->BufferHead = 0;
	buffer->BufferTail = 0;
	buffer->full = 0;
}


void buffer_clear(Buffer_t * buffer) 
{
	buffer->BufferHead = 0;
	buffer->BufferTail = 0;
	buffer->full = 0;
}


void buffer_make_buffered_stream(Buffer_t *buffer, byte_stream_t *stream) 
{
	stream->get = ( uint8_t(*)(stream_data_t*) ) &buffer_get;				// Here we need to explicitely cast the function to match the prototype  
	stream->put = ( uint8_t(*)(stream_data_t*, uint8_t) ) &buffer_put;		// stream->get and stream->put expect stream_data_t* as first argument
	stream->flush = NULL;													// but buffer_get and buffer_put take Buffer_t* as first argument
	stream->data = buffer;
	stream->bytes_available = ( uint32_t(*)(stream_data_t*) ) &buffer_bytes_available;
}


void buffer_make_buffered_stream_lossy(Buffer_t *buffer, byte_stream_t *stream) 
{
	stream->get = (uint8_t(*)(stream_data_t*)) &buffer_get;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t)) &buffer_put_lossy;
	stream->flush = NULL;
	stream->data = buffer;
	stream->bytes_available = (uint32_t(*)(stream_data_t*)) &buffer_bytes_available;
}
