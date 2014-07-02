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
 * \file streams.h
 *
 * Stream
 */


#ifndef STREAMS_H_
#define STREAMS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

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


#ifdef __cplusplus
}
#endif

#endif /* STREAMS_H_ */
