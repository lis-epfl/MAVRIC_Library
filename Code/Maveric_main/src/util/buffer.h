/*
 * buffer.h
 *
 * Created: 11/06/2012 17:43:46
 *  Author: sfx
 */ 


#ifndef BUFFER_H_
#define BUFFER_H_

#include "compiler.h"
#include "streams.h"

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE-1)


typedef struct 
{
	uint8_t Buffer[BUFFER_SIZE];
	uint8_t BufferHead, BufferTail;
	uint8_t full;
} Buffer_t;

void buffer_init(Buffer_t * buffer);

void buffer_put_lossy(Buffer_t * buffer, uint8_t byte);

int8_t buffer_put(Buffer_t * buffer, uint8_t byte);
uint8_t buffer_get(Buffer_t * buffer);

void buffer_clear(Buffer_t * buffer);

int buffer_bytes_available(Buffer_t * buffer);
int8_t buffer_full(Buffer_t * buffer);
int8_t buffer_empty(Buffer_t * buffer);

void make_buffered_stream(Buffer_t *buffer, byte_stream_t *stream);
void make_buffered_stream_lossy(Buffer_t *buffer, byte_stream_t *stream);


#endif /* BUFFER_H_ */