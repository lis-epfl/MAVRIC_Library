/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: sfx
 */ 

#include "boardsupport.h"
#include "conf_sim_model.h"
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
#include "uart_int.h"
#include "print_util.h"

#include "mavlink_stream.h"
#include "servo_pwm.h"

#include "simulation.h"
#include "bmp085.h"
#include "analog_monitor.h"
#include "piezo_speaker.h"

//static volatile board_hardware_t board_hardware;

void initialise_board(central_data_t *centralData) {
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
	init_time_keeper();
		
	INTC_init_interrupts();
		
	

		
	if (init_i2c(0)!=STATUS_OK) {
		//putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
		} else {
		//putstring(STDOUT, "initialised I2C.\n");
	};
	if (init_i2c(1)!=STATUS_OK) {
		//putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
		} else {
		//putstring(STDOUT, "initialised I2C.\n");
	};

	LED_Off(LED1);
	// Configure the pins connected to LEDs as output and set their default
	// initial state to high (LEDs off).
	//gpio_configure_pin(LED0_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	//gpio_configure_pin(LED1_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	init_Servos();
		
	init_UART_int(0);
	register_write_stream(get_UART_handle(0), &(centralData->xbee_out_stream));
				
		
	init_UART_int(3);
	make_buffered_stream(&(centralData->gps_buffer), &(centralData->gps_stream_in));
	register_read_stream(get_UART_handle(3), &(centralData->gps_stream_in));
	register_write_stream(get_UART_handle(3), &(centralData->gps_stream_out));
		
	init_UART_int(4);
	register_write_stream(get_UART_handle(4), &(centralData->wired_out_stream));

	make_buffered_stream_lossy(&(centralData->xbee_in_buffer), &(centralData->xbee_in_stream));
	make_buffered_stream_lossy(&(centralData->wired_in_buffer), &(centralData->wired_in_stream));
	register_read_stream(get_UART_handle(4), &(centralData->wired_in_stream));
	register_read_stream(get_UART_handle(0), &(centralData->xbee_in_stream));
		
	// connect abstracted aliases to hardware ports
/**/
	centralData->telemetry_down_stream=&(centralData->xbee_out_stream);
	centralData->telemetry_up_stream=&(centralData->xbee_in_stream);
	centralData->debug_out_stream=&(centralData->wired_out_stream);
	centralData->debug_in_stream=&(centralData->wired_in_stream);
/**/
/*
	centralData->telemetry_down_stream=&(centralData->wired_out_stream);
	centralData->telemetry_up_stream  =&(centralData->wired_in_stream);		
	centralData->debug_out_stream     =&(centralData->xbee_out_stream);
	centralData->debug_in_stream      =&(centralData->xbee_in_stream);
/**/

	//rc_activate_bind_mode();

	rc_init();

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
	init_analog_monitor(&centralData->adc);
	
	// init imu & compass
	init_imu(&(centralData->imu1));
	init_bmp085();
	
	// init mavlink
	init_mavlink(centralData->telemetry_down_stream, centralData->telemetry_up_stream, MAVLINK_SYS_ID);
		
	// init debug output
	dbg_print_init(centralData->debug_out_stream);
	
	// init 6V enable
	gpio_enable_gpio_pin(AVR32_PIN_PA04);
	gpio_set_gpio_pin(AVR32_PIN_PA04);
	
	Enable_global_interrupt();

	init_piezo_speaker_binary();
	
	dbg_print("Board initialised.\n");
}
