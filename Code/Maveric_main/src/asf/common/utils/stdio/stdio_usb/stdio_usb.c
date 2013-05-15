/**
 * \file
 *
 * \brief USB CDC Standard I/O Serial Management.
 *
 * This module defines support routines for a stdio serial interface to the
 * AVR Software Framework (ASF) common USB CDC service.
 *
 * Copyright (C) 2011 Atmel Corporation. All rights reserved.
 *
 * \page License
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
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 * Atmel AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */


#include "stdio_usb.h"

static bool stdio_usb_interface_enable = false;

int stdio_usb_putchar (volatile void * usart, int data)
{
	/* A negative return value should be used to indicate that data
	 * was not written, but this doesn't seem to work with GCC libc.
	 */
	if (!stdio_usb_interface_enable) {
		return 0;  // -1
	}

	return udi_cdc_putc (data) ? 0 : -1;
}

void stdio_usb_getchar (void volatile * usart, int * data)
{
	/* A negative return value should be used to indicate that data
	 * was not read, but this doesn't seem to work with GCC libc.
	 */
	if (!stdio_usb_interface_enable) {
		*data = 0;  // -1
		return;
	}
	
	*data = udi_cdc_getc ();
}

void stdio_usb_vbus_event(bool b_high)
{
	if (b_high) {
		// Attach USB Device
		udc_attach ();
	} else {
		// VBUS not present
		udc_detach ();
	}
}

bool stdio_usb_enable(void)
{
	stdio_usb_interface_enable = true;
	return true;
}

void stdio_usb_disable(void)
{
	stdio_usb_interface_enable = false;
}

void stdio_usb_init (volatile void * usart)
{
	stdio_base = usart;
	ptr_put = stdio_usb_putchar;
	ptr_get = stdio_usb_getchar;

	/*
	 * Start and attach USB CDC device interface for devices with
	 * integrated USB interfaces.  Assume the VBUS is present if
	 * VBUS monitoring is not available.
	 */
	udc_start ();

	if (! udc_include_vbus_monitoring ()) {
		stdio_usb_vbus_event (true);
	}

	// For AVR GCC libc print redirection uses fdevopen.

	#if defined(XMEGA) && defined(__GNUC__)
		fdevopen((int (*)(char, FILE*))(_write),(int (*)(FILE*))(_read));
	#endif
}

