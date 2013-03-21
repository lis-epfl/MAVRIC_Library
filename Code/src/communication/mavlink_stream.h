/*
 * mavlink_stream.h
 *
 * a wrapper for mavlink to use the stream interface
 *
 * Created: 14/02/2013 17:10:05
 *  Author: julien
 */ 


#ifndef MAVLINK_STREAM_H_
#define MAVLINK_STREAM_H_

#include "streams.h"
#include "mavlink_bridge.h"
#include "mavlink/include/common/mavlink.h"
#include "scheduler.h"
typedef struct {
	mavlink_message_t msg;
	mavlink_status_t status;
} Mavlink_Received_t;

void init_mavlink(byte_stream_t *transmit_stream, byte_stream_t *receive_stream);

task_return_t mavlink_protocol_update();


void mavlink_receive_handler();

uint8_t mavlink_receive(byte_stream_t* stream, Mavlink_Received_t* rec);
void handle_mavlink_message(Mavlink_Received_t* rec);

#endif /* MAVLINK_STREAM_H */