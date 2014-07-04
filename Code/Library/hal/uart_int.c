/** 
 * \page The MAV'RIC license
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file uart_int.c
 *
 * This file implments the UART communication protocol
 */


#include "uart_int.h"
#include "buffer.h"
#include "gpio.h"
#include "streams.h"
#include "sysclk.h"


// macro for interrupt handler
#define UART_HANDLER(UID) ISR(uart_handler_##UID, usart_opt[UID].uart_device.IRQ, AVR32_INTC_INTLEV_INT1) {\
	uint8_t c1;\
	int csr = usart_opt[UID].uart_device.uart->csr;\
	if (csr & AVR32_USART_CSR_RXRDY_MASK) {\
		c1 = (uint8_t)usart_opt[UID].uart_device.uart->rhr;\
		if (usart_opt[UID].uart_device.receive_stream == NULL) {\
			buffer_put_lossy(&(usart_opt[UID].uart_device.receive_buffer), c1);\
		} else {\
			usart_opt[UID].uart_device.receive_stream->put(usart_opt[UID].uart_device.receive_stream->data, c1);\
		}\
	}\
	if (csr & AVR32_USART_CSR_TXRDY_MASK) {\
		if (buffer_bytes_available(&(usart_opt[UID].uart_device.transmit_buffer)) > 0) {\
			c1 = buffer_get(&(usart_opt[UID].uart_device.transmit_buffer));\
			usart_opt[UID].uart_device.uart->thr = c1;\
		}\
		if (buffer_bytes_available(&(usart_opt[UID].uart_device.transmit_buffer)) == 0) {\
				usart_opt[UID].uart_device.uart->idr = AVR32_USART_IDR_TXRDY_MASK;\
		}\
	}\
}			
/*
ISR(uart_handler_4, usart_opt[4].uart_device.IRQ, AVR32_INTC_INTLEV_INT1) {\
	uint8_t c1;\
	int csr = usart_opt[4].uart_device.uart->csr;\
	if (csr & AVR32_USART_CSR_RXRDY_MASK) {\
		c1 = (uint8_t)usart_opt[4].uart_device.uart->rhr;\
		if (usart_opt[4].uart_device.receive_stream == NULL) {\
			buffer_put_lossy(&(usart_opt[4].uart_device.receive_buffer), c1);\
		} else {\
			usart_opt[4].uart_device.receive_stream->put(usart_opt[4].uart_device.receive_stream->data, c1);\
		}\
	}\
	if (csr & AVR32_USART_CSR_TXRDY_MASK) {\
		if (buffer_bytes_available(&(usart_opt[4].uart_device.transmit_buffer)) > 0) {\
			c1 = buffer_get(&(usart_opt[4].uart_device.transmit_buffer));\
			usart_opt[4].uart_device.uart->thr = c1;\
		}\
		if (buffer_bytes_available(&(usart_opt[4].uart_device.transmit_buffer))==0) {\
				usart_opt[4].uart_device.uart->idr = AVR32_USART_IDR_TXRDY_MASK;\
		}\
	}\
}*/			

// define interrupt handlers using above macro
UART_HANDLER(0);
UART_HANDLER(1);
UART_HANDLER(2);
UART_HANDLER(3);
UART_HANDLER(4);

///< Function prototype definitions
void register_UART_handler(int UID);
int uart_out_buffer_empty(usart_config_t *usart_opt);

void register_UART_handler(int UID)
{
	switch(UID)
	{
		case 0: 	
			INTC_register_interrupt( (__int_handler) &uart_handler_0, AVR32_USART0_IRQ, AVR32_INTC_INT1);
			break;
		case 1: 
			INTC_register_interrupt( (__int_handler) &uart_handler_1, usart_opt[1].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 2: 
			INTC_register_interrupt( (__int_handler) &uart_handler_2, usart_opt[2].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 3: 
			INTC_register_interrupt( (__int_handler) &uart_handler_3, usart_opt[3].uart_device.IRQ, AVR32_INTC_INT1);
			break;
		case 4: 
			INTC_register_interrupt( (__int_handler) &uart_handler_4, usart_opt[4].uart_device.IRQ, AVR32_INTC_INT1);
			break;
	}
}

void uart_int_init(int UID) {
	if ((usart_opt[UID].mode&UART_IN) > 0)
	{
		gpio_enable_module_pin(usart_opt[UID].rx_pin_map.pin, usart_opt[UID].rx_pin_map.function); 
	}
	
	if ((usart_opt[UID].mode&UART_OUT) > 0)
	{
		gpio_enable_module_pin(usart_opt[UID].tx_pin_map.pin, usart_opt[UID].tx_pin_map.function); 
	}

	usart_init_rs232( usart_opt[UID].uart_device.uart, &(usart_opt[UID].options), sysclk_get_cpu_hz()); 
	//usart_write_line(usart_opt[UID].uart_device.uart, "UART initialised");
	
	register_UART_handler(UID);
	
	buffer_init(&(usart_opt[UID].uart_device.transmit_buffer));
	buffer_init(&(usart_opt[UID].uart_device.receive_buffer));
	
	if (((usart_opt[UID].mode)&UART_IN) > 0)
	{
		usart_opt[UID].uart_device.uart->ier = AVR32_USART_IER_RXRDY_MASK;
	}
	
	//if (usart_opt[UID].mode&UART_OUT > 0)
	//{
		//usart_opt[UID].uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	//}
} 

usart_config_t *uart_int_get_uart_handle(int UID) 
{
	return &usart_opt[UID];
}

char uart_int_get_byte(usart_config_t *usart_opt) 
{
	return buffer_get(&(usart_opt->uart_device.receive_buffer));
}

int uart_int_bytes_available(usart_config_t *usart_opt) 
{
	return buffer_bytes_available(&(usart_opt->uart_device.receive_buffer));
}

uint8_t uart_int_send_byte(usart_config_t *usart_opt, char data) 
{
//	usart_write_line(usart_opt->uart_device.uart, "\ns");
	while (buffer_put(&(usart_opt->uart_device.transmit_buffer), data) < 0);
	if ((buffer_bytes_available(&(usart_opt->uart_device.transmit_buffer)) >= 1))//&&
//	  (usart_opt->uart_device.uart->csr & AVR32_USART_CSR_TXRDY_MASK)) 
	{ // if there is exactly one byte in the buffer (this one...), and transmitter ready
		 // kick-start transmission
//		usart_opt->uart_device.uart->thr='c';//buffer_get(&(usart_opt->uart_device.transmit_buffer));
		usart_opt->uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	}
	return 0;		
}

void uart_int_flush(usart_config_t *usart_opt) 
{
	usart_opt->uart_device.uart->ier = AVR32_USART_IER_TXRDY_MASK;
	while (!buffer_empty(&(usart_opt->uart_device.transmit_buffer)));
}

int uart_out_buffer_empty(usart_config_t *usart_opt) 
{
	return buffer_empty(&(usart_opt->uart_device.transmit_buffer));
}

void uart_int_register_write_stream(usart_config_t *usart_opt, byte_stream_t *stream) 
{
	stream->get = NULL;
	//stream->get = &uart_int_get_byte;
	stream->put = (uint8_t(*)(stream_data_t*, uint8_t))&uart_int_send_byte;			// Here we need to explicitly cast the function to match the prototype
	stream->flush = (void(*)(stream_data_t*))&uart_int_flush;						// stream->get and stream->put expect stream_data_t* as first argument
	stream->buffer_empty = (int(*)(stream_data_t*))&uart_out_buffer_empty;			// but buffer_get and buffer_put take Buffer_t* as first argument
	stream->data = usart_opt;
}

void uart_int_register_read_stream(usart_config_t *usart_opt,  byte_stream_t *stream) 
{
	usart_opt->uart_device.receive_stream = stream;
}


	

