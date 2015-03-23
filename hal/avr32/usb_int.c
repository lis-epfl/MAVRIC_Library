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
 * \file usb_int.c
 * 
 * \author MAV'RIC Team
 * \author Geraud L'Eplattenier
 *   
 * \brief This file implements the USB communication protocol
 * 
 ******************************************************************************/


#include "usb_int.h"
#include "streams.h"


static usb_config_t usb_conf;			///< Declare an object to store USB configuration


void usb_int_set_usb_conf(usb_config_t* usb_config)
{
	usb_conf.mode						= usb_config->mode;
	usb_conf.usb_device.IRQ				= usb_config->usb_device.IRQ;
	usb_conf.usb_device.receive_stream	= usb_config->usb_device.receive_stream;
}

void usb_int_init(void)
{
	stdio_usb_init(NULL);
	// stdio_usb_init();
	stdio_usb_enable();
} 


usb_config_t *usb_int_get_usb_handle(void)
{
	return &usb_conf;
}

void usb_int_send_byte(usb_config_t *usb_conf, uint8_t data) 
{
	for(uint8_t i=0;i<3;i++)
	{
		if (udi_cdc_is_tx_ready())
		{
			stdio_usb_putchar(NULL, (int)data);
			return;
		}	
	}
	
}


void usb_int_register_write_stream(usb_config_t *usb_conf, byte_stream_t *stream) 
{
	stream->get = NULL;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t))&usb_int_send_byte;			// Here we need to explicitly cast the function to match the prototype
	stream->flush = NULL;															// stream->get and stream->put expect stream_data_t* as first argument
	stream->buffer_empty = NULL;													// but buffer_get and buffer_put take Buffer_t* as first argument
	stream->data = usb_conf;
}

void  usb_int_register_read_stream(usb_config_t *usb_conf, byte_stream_t *stream)
{
	usb_conf->usb_device.receive_stream = stream;
}

