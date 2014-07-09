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
 * \file mavlink_stream.h
 * 
 * A wrapper for mavlink to use the stream interface
 */


#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "streams.h"
#include "mavlink_bridge.h"
#include "mavlink/include/maveric/mavlink.h"
// #include "scheduler.h"


// TODO update documentation


/**
 * \brief	Mavlink structures for the receive message and its status
 */
typedef struct 
{
	mavlink_message_t msg;
	mavlink_status_t status;
} mavlink_received_t;

typedef struct
{
	byte_stream_t* out_stream;
	byte_stream_t* in_stream;
	mavlink_received_t rec;
	bool message_available;
} mavlink_stream_t;




/**
 * \brief					Initialization of mavlink sysid, compid and scheduler to send messages
 *
 * \param transmit_stream	Pointer to the mavlink transmit stream structure
 * \param receive_stream	Pointer to the mavlink receive stream structure
 * \param sysid				System ID (1-255)
 */
void mavlink_stream_init(mavlink_stream_t* mavlink_stream, byte_stream_t *transmit_stream, byte_stream_t *receive_stream, int32_t sysid, int32_t compid);


/**
 * \brief			Mavlink parsing of message
 *
 * \param stream	Pointer to the mavlink receive stream structure
 * \param rec		Pointer to the mavlink receive message structure 
 *
 * \return			Error code, 0 if no message decoded, 1 else
 */
void mavlink_stream_receive(mavlink_stream_t* mavlink_stream);


/**
 * \brief	Flushing mavlink stream
 */
void mavlink_stream_flush(mavlink_stream_t* mavlink_stream);





#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_STREAM_H */