/*This file is prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Usart Serial driver for AVR UC3.
 *
 * This file defines a useful set of functions for the Serial interface on AVR UC3
 * devices.
 *
 * - Compiler:           IAR EWAVR32 and GNU GCC for AVR UC3
 * - Supported devices:  All AVR UC3 devices with an Usart module can be used.
 * - AppNote:
 *
 * \author               Atmel Corporation: http://www.atmel.com \n
 *                       Support and FAQ: http://support.atmel.no/
 *
 ******************************************************************************/

/* Copyright (c) 2009 Atmel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an Atmel
 * AVR product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _USART_SERIAL_H_
#define _USART_SERIAL_H_

#include "compiler.h"
#include "sysclk.h"
#include "status_codes.h"
#include "usart.h"

/*! \name Serial Management Configuration
 */
//! @{
#include "conf_usart_serial.h"

//! Default Usart Mode
#ifndef CONFIG_USART_SERIAL_MODE
#define CONFIG_USART_SERIAL_MODE             USART_NORMAL_CHMODE
#endif

//! @}

typedef usart_options_t usart_serial_options_t; 

/*! \brief Initializes the Usart in master mode.
 *
 * \param usart       Base address of the USART instance.
 * \param opt         Options needed to set up RS232 communication (see \ref usart_options_t).
 *
 */
static inline void usart_serial_init(volatile avr32_usart_t *usart, usart_serial_options_t *opt)
{
	// USART options.
	opt->channelmode = CONFIG_USART_SERIAL_MODE;
#if (defined(AVR32_USART0_ADDRESS))
	if ((uint32_t)usart == AVR32_USART0_ADDRESS)
	{
		sysclk_enable_pba_module(SYSCLK_USART0);
	}
#endif
#if (defined(AVR32_USART1_ADDRESS))
	if ((uint32_t)usart == AVR32_USART1_ADDRESS)
	{
#if UC3C          
		sysclk_enable_pbc_module(SYSCLK_USART1);
#else
		sysclk_enable_pba_module(SYSCLK_USART1);
#endif
	}
#endif
#if (defined(AVR32_USART2_ADDRESS))
	if ((uint32_t)usart == AVR32_USART2_ADDRESS)
	{
		sysclk_enable_pba_module(SYSCLK_USART2);
	}
#endif
#if (defined(AVR32_USART3_ADDRESS))
	if ((uint32_t)usart == AVR32_USART3_ADDRESS)
	{
		sysclk_enable_pba_module(SYSCLK_USART3);
	}
#endif
	usart_init_rs232(usart, opt, sysclk_get_pba_hz());
}

/*! \brief Sends a character with the USART.
 *
 * \param usart   Base address of the USART instance.
 * \param c       Character to write.
 *
 * \return Status.
 *   \retval 1  The character was written.
 *   \retval 0  The function timed out before the USART transmitter became ready to send.
 */
static inline int usart_serial_putchar(volatile avr32_usart_t *usart, int32_t c)
{
	while (usart_write_char(usart, c)!=USART_SUCCESS);
	return 1;
}
/*! \brief Waits until a character is received, and returns it.
 *
 * \param usart   Base address of the USART instance.
 * \param data   Data to read
 *
 */
static inline void usart_serial_getchar(volatile avr32_usart_t *usart, int32_t *data)
{
	*data = usart_getchar(usart);
}


/**
 * \brief Send a sequence of bytes to a USART device
 *
 * \param usart Base address of the USART instance.
 * \param data   data buffer to write
 * \param len    Length of data
 *
 */
extern status_code_t usart_serial_write_packet(volatile avr32_usart_t *usart,const uint8_t *data, size_t len);

/**
 * \brief Reveive a sequence of bytes to a USART device
 *
 * \param usart Base address of the USART instance.
 * \param data   data buffer to write
 * \param len    Length of data
 *
 */
extern status_code_t usart_serial_read_packet(volatile avr32_usart_t *usart,const uint8_t *data, size_t len);


#endif  // _USART_SERIAL_H_
