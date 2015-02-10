#ifndef UDP_STREAM_H
#define UDP_STREAM_H

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <stdbool.h>

#include "streams.h"
#include "buffer.h"

typedef struct udp_connection_t 
{
	buffer_t udp_buffer;
	struct sockaddr_in Addr; 
	int32_t sock;
} udp_connection_t;



void register_write_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, const char* target_ip, int32_t port);

void register_read_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, int32_t port);


#endif