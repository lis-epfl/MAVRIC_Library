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
 * \file streams.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Stream
 *
 ******************************************************************************/


#ifndef STREAMS_H_
#define STREAMS_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>

/**
 * \brief Stream data
 */
typedef void* stream_data_t;


/**
 * \brief Byte stream
 */
typedef struct 
{
	uint8_t 	(*get)(stream_data_t *data);					///<	Pointer to get function
	uint8_t  	(*put)(stream_data_t *data, uint8_t element);	///<	Pointer to put function
	void    	(*flush)(stream_data_t *data);					///<	Pointer to flush function
	int32_t     (*buffer_empty)(stream_data_t *data);			///<	Pointer to buffer_empty function
	uint32_t 	(*bytes_available)(stream_data_t *data);		///<	Pointer to bytes_available function
	volatile stream_data_t data;								///<	Data
} byte_stream_t;


#ifdef __cplusplus
}
#endif

#endif /* STREAMS_H_ */
