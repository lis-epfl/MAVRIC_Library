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

uint8_t buffer_full(buffer_t * buffer) 
{
	return (((buffer->buffer_head + 1)&BUFFER_MASK) == buffer->buffer_tail);
}


uint8_t buffer_put_lossy(buffer_t * buffer, uint8_t byte) 
{
	uint16_t tmp;
	tmp = (buffer->buffer_head + 1)&BUFFER_MASK;

	if (tmp==buffer->buffer_tail) 
	{
		// error: receive buffer overflow!!
		// lose old incoming data at the end of the buffer
		buffer->buffer_tail=(buffer->buffer_tail + 1)&BUFFER_MASK;
	}

	// store incoming data in buffer
	buffer->Buffer[buffer->buffer_head] = byte;
	buffer->buffer_head=tmp;
	
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


uint8_t buffer_put(buffer_t * buffer, uint8_t byte) 
{
	uint16_t tmp;
	tmp = (buffer->buffer_head + 1)&BUFFER_MASK;

	if (tmp==buffer->buffer_tail) 
	{
		//error: buffer full! return 1
		return 1;
	}
	else
	{
		// store incoming data in buffer
		buffer->Buffer[buffer->buffer_head] = byte;
		buffer->buffer_head=tmp;
	
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


uint8_t buffer_get(buffer_t * buffer) 
{
	uint8_t ret=0;
	
	if (buffer->buffer_head!=buffer->buffer_tail)
	{
		ret = buffer->Buffer[buffer->buffer_tail];
		buffer->buffer_tail =  (buffer->buffer_tail + 1)&BUFFER_MASK;
		buffer->full = 0;
	}

	return ret;
}


int8_t buffer_empty(buffer_t * buffer) 
{
	return (buffer->buffer_head==buffer->buffer_tail);
}


uint32_t buffer_bytes_available(buffer_t * buffer) 
{
	return (BUFFER_SIZE + buffer->buffer_head - buffer->buffer_tail)&BUFFER_MASK;
}


void buffer_init(buffer_t * buffer) 
{
	buffer->buffer_head = 0;
	buffer->buffer_tail = 0;
	buffer->full = 0;
}


void buffer_clear(buffer_t * buffer) 
{
	buffer->buffer_head = 0;
	buffer->buffer_tail = 0;
	buffer->full = 0;
}


void buffer_make_buffered_stream(buffer_t *buffer, byte_stream_t *stream) 
{
	stream->get = ( uint8_t(*)(stream_data_t*) ) &buffer_get;				// Here we need to explicitely cast the function to match the prototype  
	stream->put = ( uint8_t(*)(stream_data_t*, uint8_t) ) &buffer_put;		// stream->get and stream->put expect stream_data_t* as first argument
	stream->flush = NULL;													// but buffer_get and buffer_put take buffer_t* as first argument
	stream->data = buffer;
	stream->bytes_available = ( uint32_t(*)(stream_data_t*) ) &buffer_bytes_available;
}


void buffer_make_buffered_stream_lossy(buffer_t *buffer, byte_stream_t *stream) 
{
	stream->get = (uint8_t(*)(stream_data_t*)) &buffer_get;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t)) &buffer_put_lossy;
	stream->flush = NULL;
	stream->data = buffer;
	stream->bytes_available = (uint32_t(*)(stream_data_t*)) &buffer_bytes_available;
}
