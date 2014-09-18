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
 * \file mavlink_stream.h
 * 
 * \author MAV'RIC Team
 * \author Julien Lecoeur
 *   
 * \brief A wrapper for MAVLink to use the stream interface
 *
 ******************************************************************************/


#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "streams.h"
#include "conf_platform.h"
#include "mavlink/include/mavric/mavlink.h"

/**
 * \brief	Mavlink structures for the receive message and its status
 */
typedef struct 
{
	mavlink_message_t msg;			///< Mavlink message
	mavlink_status_t status;		///< Status on the message
} mavlink_received_t;


/**
 * \brief 	Main structure for the MAVLink stream module
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
 * \brief 	Configuration structure for the module MAVLink stream
 */
typedef struct 
{
	byte_stream_t* rx_stream;		///< Output stream
	byte_stream_t* tx_stream;		///< Input stream
	uint32_t sysid;					///< System ID
	uint32_t compid;				///< System Component ID
	bool use_dma;
} mavlink_stream_conf_t;


/**
 * \brief					Initialization of MAVLink sysid, compid and scheduler to send messages
 *
 * \param 	mavlink_stream		Pointer to the MAVLink stream structure
 * \param 	config				Configuration
 */
void mavlink_stream_init(mavlink_stream_t* mavlink_stream, const mavlink_stream_conf_t* config);


void mavlink_stream_send(const mavlink_stream_t* mavlink_stream, mavlink_message_t* msg);


/**
 * \brief			Mavlink parsing of message
 *
 * \param stream	Pointer to the MAVLink receive stream structure
 * \param rec		Pointer to the MAVLink receive message structure 
 *
 * \return			Error code, 0 if no message decoded, 1 else
 */
void mavlink_stream_receive(mavlink_stream_t* mavlink_stream);


/**
 * \brief	Flushing MAVLink stream
 */
void mavlink_stream_flush(mavlink_stream_t* mavlink_stream);


#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_STREAM_H */