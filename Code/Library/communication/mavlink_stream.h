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
#include "conf_platform.h"
#include "mavlink/include/maveric/mavlink.h"

/**
 * \brief	Mavlink structures for the receive message and its status
 */
typedef struct 
{
	mavlink_message_t msg;			///< Mavlink message
	mavlink_status_t status;		///< Status on the message
} mavlink_received_t;


/**
 * \brief 	Main structure for the mavlink stream module
 */
typedef struct
{
	byte_stream_t* tx;		///< Output stream
	byte_stream_t* rx;		///< Input stream
	uint32_t sysid;			///< System ID
	uint32_t compid;		///< System Component ID
	mavlink_received_t rec;	///< Last received message
	bool msg_available;		///< Indicates if a new message is available and not handled yet
	bool use_dma;			///< Indicates whether tx transfer should use dma
} mavlink_stream_t;


/**
 * \brief 	Configuration structure for the module mavlink stream
 */
typedef struct 
{
	byte_stream_t* rx_stream;		///< Output stream
	byte_stream_t* tx_stream;		///< Input stream
	uint32_t sysid;					///< System ID
	uint32_t compid;					///< System Component ID
	bool use_dma;
} mavlink_stream_conf_t;


/**
 * \brief					Initialization of mavlink sysid, compid and scheduler to send messages
 *
 * \param 	mavlink_stream		Pointer to the mavlink stream structure
 * \param 	config				Configuration
 */
void mavlink_stream_init(mavlink_stream_t* mavlink_stream, const mavlink_stream_conf_t* config);


void mavlink_stream_send(const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);


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