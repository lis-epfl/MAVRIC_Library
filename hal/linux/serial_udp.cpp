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
	#include <fcntl.h>
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_udp::Serial_udp(serial_udp_conf_t config):
	tx_udp_(udp_client(config.ip, config.tx_port)),
	rx_udp_(udp_server(config.ip, config.rx_port))
{
	config = config;

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
	int32_t  recsize;
	uint32_t n_bytes_to_read = rx_buffer_.writeable();
	char 	 buf[n_bytes_to_read];
	
	recsize = rx_udp_.recv(buf, n_bytes_to_read);

	for( int32_t i=0; i<recsize; i++ ) 
	{
		rx_buffer_.put(buf[i]);
	}

	return rx_buffer_.available();
}



uint32_t Serial_udp::writeable(void)
{
	return tx_buffer_.writeable();
}


void Serial_udp::flush(void)
{
	uint32_t n_bytes_to_send = tx_buffer_.available(); 
	uint8_t to_send[n_bytes_to_send];
	uint32_t n_sent = 0;

	for( uint32_t i=0; i<n_bytes_to_send; ++i )
	{
		tx_buffer_.get(to_send[i]);
	} 

	while( n_bytes_to_send > 0 )
	{
		int32_t ret = tx_udp_.send((const char*)&to_send[n_sent], n_bytes_to_send);
		if( ret != -1 )
		{
			n_sent += ret;
			n_bytes_to_send -= ret; 
		}
	}
}


bool Serial_udp::attach(serial_interrupt_callback_t func)
{
	// Not implemented
	return false;
}


bool Serial_udp::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = true;

	// Queue byte
	for (uint32_t i = 0; i < size; ++i)
	{
		ret &= tx_buffer_.put(bytes[i]);
	}

	// Start transmission
	if( tx_buffer_.available() >= 1 )
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
		ret = true;
		for( uint32_t i = 0; i < size; ++i )
		{
			ret &= rx_buffer_.get(bytes[i]);
		}
	}

	return ret;
}
