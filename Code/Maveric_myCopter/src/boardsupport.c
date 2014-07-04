 /* The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */
 

/**
 * \file boardsupport.c
 *
 *  Initialization of all hardware related elements (communication lines, sensors devices, etc)
 */

#include "boardsupport.h"
#include "uart_int.h"
#include "sysclk.h"
#include "sleepmgr.h"
#include "led.h"
#include "delay.h"
#include "user_board/user_board.h"
#include "buffer.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "imu.h"
#include "remote_controller.h"
#include "print_util.h"

#include "mavlink_stream.h"
#include "servo_pwm.h"

#include "simulation.h"
#include "bmp085.h"
#include "analog_monitor.h"
#include "piezo_speaker.h"

void boardsupport_init(central_data_t *centralData) {
	// int i;
	// enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;
	

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
		
	

		
	if (i2c_driver_init(0)!=STATUS_OK)
	{
		//print_util_dbg_print("Error initialising I2C\n");
		//while (1==1);
	} else {
		//print_util_dbg_print("initialised I2C.\n");
	}
	if (i2c_driver_init(1)!=STATUS_OK)
	{
		//print_util_dbg_print("Error initialising I2C\n");
		//while (1==1);
	} else {
		//print_util_dbg_print("initialised I2C.\n");
	}

	LED_On(LED1);
	// Configure the pins connected to LEDs as output and set their default
	// initial state to high (LEDs off).
	//gpio_configure_pin(LED0_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	//gpio_configure_pin(LED1_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	servo_pwm_init();
	
	// Init UART 0 for XBEE communication
	uart_int_init(0);
	uart_int_register_write_stream(uart_int_get_uart_handle(0), &(centralData->xbee_out_stream));
				
	// Init UART 3 for GPS communication
	uart_int_init(3);
	buffer_make_buffered_stream(&(centralData->gps_buffer), &(centralData->gps_stream_in));
	uart_int_register_read_stream(uart_int_get_uart_handle(3), &(centralData->gps_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(3), &(centralData->gps_stream_out));
	
	// Init UART 4 for wired communication
	uart_int_init(4);
	uart_int_register_write_stream(uart_int_get_uart_handle(4), &(centralData->wired_out_stream));

	// Registering streams
	buffer_make_buffered_stream_lossy(&(centralData->xbee_in_buffer), &(centralData->xbee_in_stream));
	buffer_make_buffered_stream_lossy(&(centralData->wired_in_buffer), &(centralData->wired_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(4), &(centralData->wired_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(0), &(centralData->xbee_in_stream));
		
	// connect abstracted aliases to hardware ports
	centralData->telemetry_down_stream = &(centralData->xbee_out_stream);
	centralData->telemetry_up_stream = &(centralData->xbee_in_stream);
	centralData->debug_out_stream = &(centralData->wired_out_stream);
	centralData->debug_in_stream = &(centralData->wired_in_stream);

/*
	centralData->telemetry_down_stream = &(centralData->wired_out_stream);
	centralData->telemetry_up_stream  = &(centralData->wired_in_stream);		
	centralData->debug_out_stream     = &(centralData->xbee_out_stream);
	centralData->debug_in_stream      = &(centralData->xbee_in_stream);
*/

	// Bind RC receiver with remote
	//remote_dsm2_rc_activate_bind_mode();

	// RC receiver initialization
	remote_dsm2_rc_init();

	// Init analog rails
	centralData->adc.enable[ANALOG_RAIL_2]  = false;
	centralData->adc.enable[ANALOG_RAIL_3]  = false;
	centralData->adc.enable[ANALOG_RAIL_4]  = false;
	centralData->adc.enable[ANALOG_RAIL_5]  = false;
	centralData->adc.enable[ANALOG_RAIL_6]  = false;
	centralData->adc.enable[ANALOG_RAIL_7]  = false;
	centralData->adc.enable[ANALOG_RAIL_10] = true;		// Battery filtered
	centralData->adc.enable[ANALOG_RAIL_11] = true;		// Battery 
	centralData->adc.enable[ANALOG_RAIL_12] = true;		// sonar
	centralData->adc.enable[ANALOG_RAIL_13] = false;    // pitot
	analog_monitor_init(&centralData->adc);
	
	// init imu & compass
	imu_init(&(centralData->imu1));
	bmp085_init();
	
	// init mavlink
	mavlink_stream_init(centralData->telemetry_down_stream, centralData->telemetry_up_stream, MAVLINK_SYS_ID);
		
	// init debug output
	print_util_dbg_print_init(centralData->debug_out_stream);
	
	// init 6V enable
	gpio_enable_gpio_pin(AVR32_PIN_PA04);
	gpio_set_gpio_pin(AVR32_PIN_PA04);
	
	Enable_global_interrupt();

	// Init piezo speaker
	piezo_speaker_init_binary();
	
	print_util_dbg_print("Board initialised.\n");
}
