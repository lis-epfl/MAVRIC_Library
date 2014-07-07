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

/**
 * @brief Stream data
 */
typedef void* stream_data_t;


/**
 * @brief Byte stream
 */
typedef struct {
	uint8_t (*get)(stream_data_t *data);						///<	Pointer to get function
	uint8_t  (*put)(stream_data_t *data, uint8_t element);		///<	Pointer to put function
	void    (*flush)(stream_data_t *data);						///<	Pointer to flush function
	int32_t     (*buffer_empty)(stream_data_t *data);				///<	Pointer to buffer_empty function
	uint32_t     (*bytes_available)(stream_data_t *data);			///<	Pointer to bytes_available function
	volatile stream_data_t data;								///<	Data
} byte_stream_t;


#ifdef __cplusplus
}
#endif

#endif /* STREAMS_H_ */
