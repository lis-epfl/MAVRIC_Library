/**
 * Buffer
 *
 * The MAV'RIC Framework
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 *
 * This file is part of the MAV'RIC Framework.
 */


#ifndef BUFFER_H_
#define BUFFER_H_

#ifdef __cplusplus
extern "C" 
{
#endif

#include "compiler.h"
#include "streams.h"

#define BUFFER_SIZE 256
#define BUFFER_MASK (BUFFER_SIZE-1)


/**
 * @brief 		Buffer structure
 */
typedef struct 
{
	uint8_t Buffer[BUFFER_SIZE];		///<	Array of bytes containing the data
	uint8_t BufferHead;					///<	Head of the buffer (newest byte)
	uint8_t BufferTail;					///<	Tail of the buffer (oldest byte)
	uint8_t full;						///<	Boolean, 1 if full, 0 if not
} Buffer_t;


/**
 * @brief        	Buffer initialisation
 * 
 * @param buffer 	Pointer to buffer
 */
void buffer_init(Buffer_t * buffer);


/**
 * @brief        	Stores data in the buffer even of it overwrites existing data
 * 
 * @param buffer 	Pointer to buffer
 * @param byte   	Byte to write
 */
void buffer_put_lossy(Buffer_t * buffer, uint8_t byte);


/**
 * @brief        	Stores data in the buffer, until the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param byte   	Byte to write
 * 
 * @return       	Boolean, 1 if successfully added, 0 if not
 */
int8_t buffer_put(Buffer_t * buffer, uint8_t byte);


/**
 * @brief        	Get the oldest element in the buffer
 * 
 * @param buffer 	Pointer to buffer
 * 
 * @return       	Oldest byte in buffer
 */
uint8_t buffer_get(Buffer_t * buffer);


/**
 * @brief        	Clear the buffer
 * @details      	This function erases the buffer, note that it does not erase all the bytes one by one so the function call is fast
 * 
 * @param buffer 	Pointer to buffer
 */
void buffer_clear(Buffer_t * buffer);


/**
 * @brief        	Returns the number of available bytes in the buffer
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Number of available bytes
 */
int buffer_bytes_available(Buffer_t * buffer);


/**
 * @brief        	Tests whether the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Boolean, 1 if full, 0 if not
 */
int8_t buffer_full(Buffer_t * buffer);


/**
 * @brief        	Tests whether the buffer is empty
 * 
 * @param buffer 	Pointer to buffer
 * @return       	Boolean, 1 if empty, 0 if not
 */
int8_t buffer_empty(Buffer_t * buffer);


/**
 * @brief        	Creates a buffered stream
 * @details      	As soon as a new byte arrives on the stream, it is stored in the buffer, unless the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param stream 	Pointer to stream
 */
void make_buffered_stream(Buffer_t *buffer, byte_stream_t *stream);


/**
 * @brief        	Creates a buffered stream
 * @details      	As soon as a new byte arrives on the stream, it is stored in the buffer, even if the buffer is full
 * 
 * @param buffer 	Pointer to buffer
 * @param stream 	Pointer to stream
 */
void make_buffered_stream_lossy(Buffer_t *buffer, byte_stream_t *stream);


#ifdef __cplusplus
}
#endif

#endif /* BUFFER_H_ */