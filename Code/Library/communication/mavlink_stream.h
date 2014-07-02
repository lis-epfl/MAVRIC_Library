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

#include <stdint.h>
#include "streams.h"
#include "mavlink_bridge.h"
#include "mavlink/include/maveric/mavlink.h"
#include "scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * \brief	Mavlink structures for the receive message and its status.
 */
typedef struct {
	mavlink_message_t msg;
	mavlink_status_t status;
} Mavlink_Received_t;

/**
 * \brief					Initialization of mavlink sysid, compid and scheduler to send messages
 *
 * \param transmit_stream	Pointer to the mavlink transmit stream structure
 * \param receive_stream	Pointer to the mavlink receive stream structure
 * \param sysid				System ID (1-255)
 */
void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream, int sysid);

/**
 * \brief	Run task scheduler update if the buffer is empty 
 *
 * \return	Task status return
 */
task_return_t mavlink_protocol_update(void);

/**
 * \brief	Obtain the address of the mavlink task set
 *
 * \return	A pointer to the task set
 */
task_set* get_mavlink_taskset(void);

/**
 * \brief	Receive mavlink message
 */
void mavlink_receive_handler(void);

/**
 * \brief			Mavlink parsing of message
 *
 * \param stream	Pointer to the mavlink receive stream structure
 * \param rec		Pointer to the mavlink receive message structure 
 *
 * \return			Error code, 0 if no message decoded, 1 else
 */
uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec);

/**
 * \brief		handling specific mavlink message
 *
 * \param rec	Pointer to the mavlink receive message structure
 */
void handle_mavlink_message(Mavlink_Received_t* rec);

/**
 * \brief	Flushing mavlink stream
 */
void flush_mavlink(void);

/**
 * \brief		Suspending sending of messages
 *
 * \param delay	Delay of suspension in microsecond
 */
void suspend_downstream(uint32_t delay);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_STREAM_H */