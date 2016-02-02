/*******************************************************************************
 * Copyright (c) 2009-2016, MAV'RIC Development Team
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
 * \file 	serial_avr32.cpp
 * 
 * \author 	MAV'RIC Team
 *   
 * \brief 	Implementation of serial peripheral for avr32
 *
 ******************************************************************************/


#include "hal/avr32/serial_avr32.hpp"

extern "C"
{
	#include "libs/asf/avr32/drivers/gpio/gpio.h"
	#include "libs/asf/common/services/clock/sysclk.h"
}

//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_avr32::Serial_avr32(serial_avr32_conf_t config)
{
	config_			= config;
	irq_callback 	= NULL;
}


bool Serial_avr32::init(void)
{	
	// Init gpios
	if( config_.mode==AVR32_SERIAL_IN || config_.mode==AVR32_SERIAL_IN_OUT )
	{
		gpio_enable_module_pin( config_.rx_pin_map.pin,
								config_.rx_pin_map.function );
	}

	if( config_.mode==AVR32_SERIAL_OUT || config_.mode==AVR32_SERIAL_IN_OUT )
	{
		gpio_enable_module_pin( config_.tx_pin_map.pin,
								config_.tx_pin_map.function );
	}

	// Add pointer to this instance to the static handler array (for interrupt dispatch)
	handlers_[config_.serial_device] = this;

	// Init pointer to device, and Register interrupt
	switch( config_.serial_device )
	{
		case AVR32_SERIAL_0:
			uart_ = &AVR32_USART0;
			INTC_register_interrupt( &irq0, AVR32_USART0_IRQ, AVR32_INTC_INT1);		
		break;
		case AVR32_SERIAL_1:
			uart_ = &AVR32_USART1;
			INTC_register_interrupt( &irq1, AVR32_USART1_IRQ, AVR32_INTC_INT1);		
		break;
		case AVR32_SERIAL_2:
			uart_ = &AVR32_USART2;
			INTC_register_interrupt( &irq2, AVR32_USART2_IRQ, AVR32_INTC_INT1);		
		break;
		case AVR32_SERIAL_3:
			uart_ = &AVR32_USART3;
			INTC_register_interrupt( &irq3, AVR32_USART3_IRQ, AVR32_INTC_INT1);		
		break;
		case AVR32_SERIAL_4:
			uart_ = &AVR32_USART4;
			INTC_register_interrupt( &irq4, AVR32_USART4_IRQ, AVR32_INTC_INT1);		
		break;
		default:
		break;
	}
	
	// Init peripheral
	usart_init_rs232( uart_, &config_.options, sysclk_get_cpu_hz() ); 
	
	
	if( config_.mode==AVR32_SERIAL_IN || config_.mode==AVR32_SERIAL_IN_OUT )
	{
		uart_->ier = AVR32_USART_IER_RXRDY_MASK;
	}

	return true;
}


	
uint32_t Serial_avr32::readable(void)
{
	return rx_buffer_.readable();
}



uint32_t Serial_avr32::writeable(void)
{
	return tx_buffer_.writeable();
}


void Serial_avr32::flush(void)
{
	uart_->ier = AVR32_USART_IER_TXRDY_MASK;

	// Wait until the transmit buffer is empty
	while( !tx_buffer_.empty() )
	{
		;
	}
}


bool Serial_avr32::attach(serial_interrupt_callback_t func)
{
	irq_callback = func;
	return true;
}


bool Serial_avr32::write(const uint8_t* bytes, const uint32_t size)
{
	bool ret = false;

	// // Queue byte
	if (writeable() >= size)
	{
		for (uint32_t i = 0; i < size; ++i)
		{
			tx_buffer_.put(bytes[i]);
		}
		ret = true;
	}

	// // Start transmission								TODO: check if this should not be at the begining of the function to avoid infinite while loop
	if( tx_buffer_.readable() >= 1 )
	{ 
		uart_->ier = AVR32_USART_IER_TXRDY_MASK;
	}

	return ret;
}


bool Serial_avr32::read(uint8_t* bytes, const uint32_t size)
{
	bool ret = false;

	if (readable() >= size)
	{
		ret = true;
		for( uint32_t i = 0; i < size; ++i )
		{
			ret &= rx_buffer_.get(bytes[i]);
		}
	}
	
	return ret;
}


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

Serial_avr32* Serial_avr32::handlers_[AVR32_SERIAL_MAX_NUMBER] = {0};


__attribute__((__interrupt__))
void Serial_avr32::irq0(void)
{
	handlers_[AVR32_SERIAL_0]->irq_handler();
} 	


__attribute__((__interrupt__))
void Serial_avr32::irq1(void)
{
	handlers_[AVR32_SERIAL_1]->irq_handler();
}


__attribute__((__interrupt__))
void Serial_avr32::irq2(void)
{
	handlers_[AVR32_SERIAL_2]->irq_handler();
}


__attribute__((__interrupt__))
void Serial_avr32::irq3(void)
{
	handlers_[AVR32_SERIAL_3]->irq_handler();
}


__attribute__((__interrupt__))
void Serial_avr32::irq4(void)
{
	handlers_[AVR32_SERIAL_4]->irq_handler();
}


void Serial_avr32::irq_handler(void)
{
	uint8_t c1 = 0;
	int32_t csr = uart_->csr;
	
	// Incoming data
	if( csr & AVR32_USART_CSR_RXRDY_MASK ) 
	{
		c1 = (uint8_t)uart_->rhr;
		// usart_read_char(uart_, (int*)&c1);

		rx_buffer_.put_lossy(c1);
	}

	// Outgoing data
	if( csr & AVR32_USART_CSR_TXRDY_MASK ) 
	{
		if( tx_buffer_.readable() > 0 ) 
		{
			tx_buffer_.get(c1);
			uart_->thr = c1;
		}
		else
		{
			// nothing more to send, disable interrupt
			uart_->idr = AVR32_USART_IDR_TXRDY_MASK;
		}
	}

	// Call callback function if attached
	if( irq_callback != NULL )
	{
		irq_callback(this);
	}
}