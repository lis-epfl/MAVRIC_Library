/*
 * ishtar_stream.h
 *
 * a wrapper for ishtar to use the stream interface
 *
 * Created: 29/08/2012 15:15:14
 *  Author: schill
 */ 


#ifndef ISHTAR_STREAM_H_
#define ISHTAR_STREAM_H_

#include "streams.h"


#include "ishtar.h"
#include "ishtar_def.h"


/**
 * Initialises ishtar, and the stream interfaces to ishtar. 
 * @param trasmit_stream: A byte stream that is registered with a transmitting hardware interface (e.g. UART)
 */
void init_ishtar_streams(byte_stream_t *transmit_stream);

/**
 * returns the internal receive stream, which maps received bytes to the ishtar receive function.
 * This stream has to be registered with a receiving hardware interface. 
 */
byte_stream_t* ishtar_get_inbound_stream(void);


#endif /* ISHTAR_STREAM_H_ */