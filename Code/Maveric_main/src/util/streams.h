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
	void    (*flush)(uint8_t element);
	volatile stream_data_t data;
} byte_stream_t;





#endif
