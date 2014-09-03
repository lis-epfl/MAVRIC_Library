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
 * \file buffer.h
 * 
 * \author MAV'RIC Team
 * \author Felix Schill
 *   
 * \brief Buffer
 *
 ******************************************************************************/


#ifndef BUFFER_H_
#define BUFFER_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>
#include "streams.h"

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE-1)


/**
 * @brief 		Buffer structure
 */
typedef struct 
{
	uint8_t Buffer[BUFFER_SIZE];		///<	Array of bytes containing the data
	uint8_t buffer_head;					///<	Head of the buffer (newest byte)
	uint8_t buffer_tail;					///<	Tail of the buffer (oldest byte)
	uint8_t full;						///<	Boolean, 1 if full, 0 if not
} buffer_t;


/**
 * @brief        	Buffer initialisation
 * 
 * @param buffer 	Pointer to buffer
 */
void buffer_init(buffer_t * buffer);


/**
 * @brief        	Stores data in the buffer even of it overwrites existing data
 * 
 * @param buffer 	Pointer to buffer
 * @param byte   	Byte to write
 */
uint8_t buffer_put_lossy(buffer_t * buffer, uint8_t byte);


/**
 * @brief        	Stores data in the buffer, until the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param byte   	Byte to write
 * 
 * @return       	Boolean, 0 if successfully added, 1 if not
 */
uint8_t buffer_put(buffer_t * buffer, uint8_t byte);


/**
 * @brief        	Get the oldest element in the buffer
 * 
 * @param buffer 	Pointer to buffer
 * 
 * @return       	Oldest byte in buffer
 */
uint8_t buffer_get(buffer_t * buffer);


/**
 * @brief        	Clear the buffer
 * @details      	This function erases the buffer, note that it does not erase all the bytes one by one so the function call is fast
 * 
 * @param buffer 	Pointer to buffer
 */
void buffer_clear(buffer_t * buffer);


/**
 * @brief        	Returns the number of available bytes in the buffer
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Number of available bytes
 */
uint32_t buffer_bytes_available(buffer_t * buffer);


/**
 * @brief        	Tests whether the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Boolean, 1 if full, 0 if not
 */
uint8_t buffer_full(buffer_t * buffer);


/**
 * @brief        	Tests whether the buffer is empty
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Boolean, 1 if empty, 0 if not
 */
int8_t buffer_empty(buffer_t * buffer);


/**
 * @brief        	Creates a buffered stream
 * @details      	As soon as a new byte arrives on the stream, it is stored in the buffer, unless the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param stream 	Pointer to stream
 */
void buffer_make_buffered_stream(buffer_t *buffer, byte_stream_t *stream);


/**
 * @brief        	Creates a buffered stream
 * @details      	As soon as a new byte arrives on the stream, it is stored in the buffer, even if the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param stream 	Pointer to stream
 */
void buffer_make_buffered_stream_lossy(buffer_t *buffer, byte_stream_t *stream);


#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H_ */