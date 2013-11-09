/*
 * streams.h
 *
 * Created: 07/06/2012 21:07:26
 *  Author: sfx
 */


#ifndef STREAMS_H_
#define STREAMS_H_
#include "compiler.h"

typedef void* stream_data_t;


typedef struct {
	uint8_t (*get)(stream_data_t *data);
	int8_t  (*put)(stream_data_t *data, uint8_t element);
	void    (*flush)(stream_data_t *data);
	int     (*buffer_empty)(stream_data_t *data);
	int     (*bytes_available)(stream_data_t *data);
	volatile stream_data_t data;
} byte_stream_t;





#endif
