/*
 * ishtar_stream.c
 *
 * Created: 29/08/2012 15:15:28
 *  Author: schill
 */ 


#include "ishtar_stream.h"
#include "buffer.h"
#include "print_util.h"
#include "uart_int.h"
#include "delay.h"
byte_stream_t inbound_stream;
byte_stream_t *outbound_stream;

byte_stream_t* deb_stream;

void ishtar_stream_send(unsigned char *data, unsigned long size);
void ishtar_stream_send(unsigned char *data, unsigned long size) {
	uint8_t c;
	long i;
	dbg_print( "i.s");
	for (i=size-1; i>=0; i--) {
		c=data[i];
		outbound_stream->put(outbound_stream->data, c);
		dbg_print_num( c, 16);
		dbg_print( " ");
		delay_ms(3);
	}		
	dbg_print( "\n");

}

void ishtar_flush() {
	dbg_print("flush!\n");
	uart_int_flush(get_UART_handle(4));
}

/* wrap the ishtar_ReceiveDataByte function into a stream-compatible method
*/
void ishtar_receive(stream_data_t* data, uint8_t element);
void ishtar_receive(stream_data_t* data, uint8_t element) {
	ishtar_ReceiveDataByte(element);
	dbg_print_num(element, 16);
	dbg_print(".");

}


void init_ishtar_streams(byte_stream_t *transmit_stream) {
	
	// remember the outbound stream, to which data will be sent. This can be registered by the calling entity
	// with various hardware stream interfaces (e.g. UART)
	outbound_stream= transmit_stream;
	
	// set the local receive function as the "put" operation, which will be called by the source of this stream when data arrives
	inbound_stream.put=&ishtar_receive;
	
	
	
	deb_stream=get_debug_stream();
	dbg_log_value("ishtar stream send: ", &ishtar_stream_send, 16);
	
	// initialise ishtar with the locally defined send function
	ishtar_Init(&ishtar_stream_send, &ishtar_flush);
	dbg_print("ishtar initialised\n");
}

byte_stream_t* ishtar_get_inbound_stream() {
	return &inbound_stream;
	
}

