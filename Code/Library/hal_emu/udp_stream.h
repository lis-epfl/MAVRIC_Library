#ifndef UDP_STREAM_H
#define UDP_STREAM_H

#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include "streams.h"
#include "buffer.h"
typedef struct udp_connection_t {
	Buffer_t udp_buffer;
	struct sockaddr_in Addr; 
	int sock;
} udp_connection_t;



void register_write_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, const char* target_ip, int port);

void register_read_stream_udp(byte_stream_t *stream, udp_connection_t *udpconn, int port);


#endif