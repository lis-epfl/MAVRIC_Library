/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: sfx
 */ 

#include "boardsupport.h"


initialise_board(central_data_t *central_data){
		
		init_UART_int(4);

		
		central_data->xbee_out_stream.put=NULL;
		
		
		dbg_print_init(&central_data->xbee_out_stream);
		
		register_write_stream(get_UART_handle(4), &central_data->wired_out_stream);

		make_buffered_stream_lossy(&(central_data->wired_in_buffer), &(central_data->wired_in_stream));
		register_read_stream(get_UART_handle(4), &(central_data->wired_in_stream));

		central_data->telemetry_down_stream=&central_data->wired_out_stream;
		central_data->telemetry_up_stream=&central_data->wired_in_stream;
		
		central_data->debug_out_stream=&central_data->wired_out_stream;

		// init mavlink
		init_mavlink(central_data->telemetry_down_stream, central_data->telemetry_up_stream, 100);
		dbg_print_init(&central_data->debug_out_stream);
		
		Init_DAC(0);
		DAC_set_value(0);

}




