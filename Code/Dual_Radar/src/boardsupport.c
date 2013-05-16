/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: sfx
 */ 

#include "boardsupport.h"


static volatile board_hardware_t board_hardware;

board_hardware_t* initialise_board() {
		
		init_UART_int(4);

		
		board_hardware.xbee_out_stream.put=NULL;
		dbg_print_init(&board_hardware.xbee_out_stream);
		
		
		register_write_stream(get_UART_handle(4), &board_hardware.debug_stream);

		register_read_stream(get_UART_handle(4), &board_hardware.debug_in_stream);
		board_hardware.telemetry_down_stream=&board_hardware.debug_stream;
		board_hardware.telemetry_up_stream=&board_hardware.debug_in_stream;
		// init mavlink
		init_mavlink(board_hardware.telemetry_down_stream, board_hardware.telemetry_up_stream);
		

		return &board_hardware;
}

board_hardware_t* get_board_hardware() {
	return &board_hardware;
}


byte_stream_t* get_telemetry_upstream() {
	return board_hardware.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return board_hardware.telemetry_down_stream;
}
byte_stream_t* get_debug_stream() {
	return &board_hardware.debug_stream;
}

