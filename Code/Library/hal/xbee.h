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
* \file xbee.h
*
* This file is for the xbee settings
*/


#ifndef XBEE_H_
#define XBEE_H_

#include "streams.h"
#include "buffer.h"

/**
 * \brief Initialize the xbee module
 *
 * \param UID uart device number
 */
void xbee_init(int32_t UID);

/**
 * \brief Return the xbee in stream
 *
 * \return the pointer to the xbee in stream
 */
byte_stream_t* xbee_get_in_stream(void);

/**
 * \brief Return the xbee out stream
 *
 * \return the pointer to the xbee out stream
 */
byte_stream_t* xbee_get_out_stream(void);

#endif /* XBEE_H_ */