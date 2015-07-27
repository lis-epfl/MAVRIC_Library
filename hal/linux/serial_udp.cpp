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
 * \file 	serial_udp.cpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Linux implementation of serial peripherals using UDP
 *
 ******************************************************************************/

#include "serial_udp.hpp"


extern "C"
{
	#include <unistd.h>
	// #include <stdlib.h>
	#include <stdio.h>
	#include <errno.h>
	#include <string.h>
	#include <sys/socket.h>
	#include <sys/types.h>
	#include <netinet/in.h>
	#include <fcntl.h>
	#include <time.h>
	#include <sys/time.h>
	#include <time.h>
	#include <arpa/inet.h>
	// #include "print_util.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_udp::Serial_udp(serial_udp_conf_t config):
	tx_udp_(udp_client(config.ip, config.tx_port)),
	rx_udp_(udp_server(config.ip, config.rx_port))
{
	config = config;

	// Init buffers
	buffer_init(&tx_buffer_);
	buffer_init(&rx_buffer_);

	// Set rx as non blocking
	int sock = rx_udp_.get_socket();
	int flags = fcntl (sock, F_GETFL, 0 );
	fcntl( sock, F_SETFL, flags | O_NONBLOCK );
}


bool Serial_udp::init(void)
{
	return true;
}

	
uint32_t Serial_udp::readable(void)
{
	int32_t recsize;
	char 	buf[BUFFER_SIZE];
	
	recsize = rx_udp_.recv(buf, BUFFER_SIZE);

	for( int32_t i=0; i<recsize; i++ ) 
	{
		buffer_put( &rx_buffer_, buf[i] );
	}

	return buffer_bytes_available(&rx_buffer_);
}



uint32_t Serial_udp::writeable(void)
{
	return buffer_bytes_free( &tx_buffer_ );
}


void Serial_udp::flush(void)
{
	int32_t bytes_sent; 

	bytes_sent = tx_udp_.send((const char*)tx_buffer_.Buffer, buffer_bytes_available(&tx_buffer_));
	
	if( bytes_sent == (int32_t)buffer_bytes_available( &tx_buffer_ ) ) 
	{
		buffer_clear( &tx_buffer_ );
	}
	else 
	{
		for(int32_t i=0; i<bytes_sent; i++ ) 
		{
			buffer_get( &tx_buffer_ );
		}
	}
}


bool Serial_udp::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = true;

	// Queue byte
	for (uint32_t i = 0; i < size; ++i)
	{
		ret &= buffer_put( &tx_buffer_, bytes[i] );
	}

	// Start transmission
	if( buffer_bytes_available( &tx_buffer_ ) >= 1 )
	{ 
		flush();
	}

	return ret;
}


bool Serial_udp::read(uint8_t bytes[], const uint32_t size)
{
	bool ret = false;

	if( readable() >= size )
	{
		for (uint32_t i = 0; i < size; ++i)
		{
			bytes[i] = buffer_get( &rx_buffer_ );
		}
		ret = true;
	}

	return ret;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------