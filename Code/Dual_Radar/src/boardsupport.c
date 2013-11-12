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
		
		
		register_write_stream(get_UART_handle(4), &central_data->debug_stream);

		register_read_stream(get_UART_handle(4), &central_data->debug_in_stream);
		central_data->telemetry_down_stream=&central_data->debug_stream;
		central_data->telemetry_up_stream=&central_data->debug_in_stream;
		// init mavlink
		init_mavlink(central_data->telemetry_down_stream, central_data->telemetry_up_stream, 100);
		
		Init_ADCI(100000, ADCIFA_REF06VDD, 16, 4);
		Init_DAC(0);
		DAC_set_value(0);

}




