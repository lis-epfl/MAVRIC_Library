/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file console.h
 *
 * This file configures the console UART communication
 */


#ifndef CONSOLE_H_
#define CONSOLE_H_

#include "streams.h"
#include "buffer.h"

/**
 * \brief Initialize the console module
 *
 * \param UID uart device number
 */
void console_init(int32_t UID);

/**
 * \brief Return the console in stream
 *
 * \return the pointer to the console in stream
 */
byte_stream_t* console_get_in_stream(void);

/**
 * \brief Return the console out stream
 *
 * \return the pointer to the console out stream
 */
byte_stream_t* console_get_out_stream(void);

#endif /* CONSOLE_H_ */