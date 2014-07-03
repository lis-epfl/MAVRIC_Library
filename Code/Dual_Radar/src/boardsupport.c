/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: sfx
 */ 

#include "boardsupport.h"
#include "sysclk.h"
#include "sleepmgr.h"
#include "time_keeper.h"
#include "delay.h"


initialise_board(central_data_t *central_data){
		
		irq_initialize_vectors();
		cpu_irq_enable();
		Disable_global_interrupt();
			
		// Initialize the sleep manager
		sleepmgr_init();
		sysclk_init();

		board_init();
		delay_init(sysclk_get_cpu_hz());
		time_keeper_init();
			
		INTC_init_interrupts();

		/*
		if (i2c_driver_init(1)!=STATUS_OK) {
			//putstring(STDOUT, "Error initialising I2C\n");
			while (1==1);
			} else {
			//putstring(STDOUT, "initialised I2C.\n");
		};
		i2c_slave_interface_init(1);
		*/
		
		uart_int_init(4);

		
		
		
		uart_int_register_write_stream(uart_int_get_uart_handle(4), &central_data->wired_out_stream);

		make_buffered_stream(&(central_data->wired_in_buffer), &(central_data->wired_in_stream));
		uart_int_register_read_stream(uart_int_get_uart_handle(4), &(central_data->wired_in_stream));

		central_data->telemetry_down_stream=&(central_data->wired_out_stream);
		central_data->telemetry_up_stream  =&(central_data->wired_in_stream);
		
		central_data->debug_out_stream=&central_data->wired_out_stream;

		// init mavlink
		mavlink_stream_init(central_data->telemetry_down_stream, central_data->telemetry_up_stream, 100);
		dbg_print_init(central_data->debug_out_stream);
		
		dac_dma_init(0);
		dac_dma_set_value(0);
		Enable_global_interrupt();
}




