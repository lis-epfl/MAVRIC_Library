/*
 * uart_int.c
 *
 * Created: 11/06/2012 14:09:00
 *  Author: sfx
 */ 

#include "uart_int.h"
#include "buffer.h"
#include "gpio.h"
#include "streams.h"
#include "sysclk.h"


// macro for interrupt handler
#define UART_HANDLER(UID) ISR(uart_handler_##UID, usart_opt[UID].uart_device.IRQ, AVR32_INTC_INTLEV_INT1) {\
	uint8_t c1;\
	int csr=usart_opt[UID].uart_device.uart->csr;\
	if (csr & AVR32_USART_CSR_RXRDY_MASK) {\
		c1=(uint8_t)usart_opt[UID].uart_device.uart->rhr;\
		if (usart_opt[UID].uart_device.receive_stream==NULL) {\
			buffer_put(&(usart_opt[UID].uart_device.receive_buffer), c1);\
		} else {\
			usart_opt[UID].uart_device.receive_stream->put(usart_opt[UID].uart_device.receive_stream->data, c1);\
		}\
	}\
	if (csr & AVR32_USART_CSR_TXRDY_MASK) {\
		if (buffer_bytes_available(&(usart_opt[UID].uart_device.transmit_buffer))>0) {\
			c1=buffer_get(&(usart_opt[UID].uart_device.transmit_buffer));\
			usart_opt[UID].uart_device.uart->thr=c1;\
		}\
		if (buffer_bytes_available(&(usart_opt[UID].uart_device.transmit_buffer))==0) {\
				usart_opt[UID].uart_device.uart->idr=AVR32_USART_IDR_TXRDY_MASK;\
		}\
	}\
}\			


// define interrupt handlers using above macro
UART_HANDLER(0);
UART_HANDLER(1);
UART_HANDLER(2);
UART_HANDLER(3);
UART_HANDLER(4);


void register_UART_handler(int UID) {
	switch(UID) {
		case 0: 	INTC_register_interrupt( (__int_handler) &uart_handler_0, AVR32_USART0_IRQ, AVR32_INTC_INT1); break;
		case 1: 	INTC_register_interrupt( (__int_handler) &uart_handler_1, usart_opt[1].uart_device.IRQ, AVR32_INTC_INT1); break;
		case 2: 	INTC_register_interrupt( (__int_handler) &uart_handler_2, usart_opt[2].uart_device.IRQ, AVR32_INTC_INT1); break;
		case 3: 	INTC_register_interrupt( (__int_handler) &uart_handler_3, usart_opt[3].uart_device.IRQ, AVR32_INTC_INT1); break;
		case 4: 	INTC_register_interrupt( (__int_handler) &uart_handler_4, usart_opt[4].uart_device.IRQ, AVR32_INTC_INT1); break;

	}
	
}

usart_config_t *init_UART_int(UID) {
	if (usart_opt[UID].mode&UART_IN >0)  gpio_enable_module_pin(usart_opt[UID].rx_pin_map.pin, usart_opt[UID].rx_pin_map.function); 
	if (usart_opt[UID].mode&UART_OUT>0) gpio_enable_module_pin(usart_opt[UID].tx_pin_map.pin, usart_opt[UID].tx_pin_map.function); 

	usart_init_rs232( usart_opt[UID].uart_device.uart, &(usart_opt[UID].options), sysclk_get_cpu_hz()); 
	//usart_write_line(usart_opt[UID].uart_device.uart, "UART initialised");
	register_UART_handler(UID);
	buffer_init(&usart_opt[UID].uart_device.transmit_buffer);
	buffer_init(&usart_opt[UID].uart_device.receive_buffer);
	if (usart_opt[UID].mode&UART_IN >0) usart_opt[UID].uart_device.uart->ier=AVR32_USART_IER_RXRDY_MASK;
	//if (usart_opt[UID].mode&UART_OUT>0) usart_opt[UID].uart_device.uart->ier=AVR32_USART_IER_TXRDY_MASK;
} 

/************************************************************************/
/* blocking operation to retrieve a received byte from uart             */
/************************************************************************/
char uart_int_get_byte(usart_config_t *usart_opt) {
	return buffer_get(&(usart_opt->uart_device.receive_buffer));
}

/************************************************************************/
/* returns number of received bytes in the receive buffer               */
/************************************************************************/
int uart_int_bytes_available(usart_config_t *usart_opt) {
	return buffer_bytes_available(&(usart_opt->uart_device.receive_buffer));
}

/************************************************************************/
/* non-blocking operation to append a byte to the uart send buffer      */
/* if buffer is full, the command has no effect  (returns -1).          */
/************************************************************************/
short uart_int_send_byte(usart_config_t *usart_opt, char data) {
//	usart_write_line(usart_opt->uart_device.uart, "\ns");
	while (buffer_put(&(usart_opt->uart_device.transmit_buffer), data)<0);
	if ((buffer_bytes_available(&(usart_opt->uart_device.transmit_buffer)) >= 1))//&&
//	  (usart_opt->uart_device.uart->csr & AVR32_USART_CSR_TXRDY_MASK)) 
	{ // if there is exactly one byte in the buffer (this one...), and transmitter ready
		 // kick-start transmission
//		usart_opt->uart_device.uart->thr='c';//buffer_get(&(usart_opt->uart_device.transmit_buffer));
		usart_opt->uart_device.uart->ier=AVR32_USART_IER_TXRDY_MASK;
	} 		
}


/** 
 * blocking operation to flush the uart buffer. Returns once the last byte has been passed to hardware for transmission.
 */
short uart_int_flush(usart_config_t *usart_opt) {
	usart_opt->uart_device.uart->ier=AVR32_USART_IER_TXRDY_MASK;
	while (!buffer_empty(&(usart_opt->uart_device.transmit_buffer)));
}

usart_config_t *get_UART_handle(int UID) {
	return &usart_opt[UID];
}

void register_write_stream(usart_config_t *usart_opt, byte_stream_t *stream) {
	stream->get=NULL;
	stream->put=&uart_int_send_byte;
	stream->flush=&uart_int_flush;
	stream->data=usart_opt;
}


void register_read_stream(usart_config_t *usart_opt,  byte_stream_t *stream) {
	usart_opt->uart_device.receive_stream=stream;
}


	

