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

//static volatile board_hardware_t board_hardware;

void boardsupport_init(central_data_t *centralData) {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;
	

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
		
	

		
	if (i2c_driver_init(0)!=STATUS_OK) {
		//print_util_putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
		} else {
		//print_util_putstring(STDOUT, "initialised I2C.\n");
	};
	/* Remove ic1 which was for the radar interface to activate uart2 for the gumstix interface
	if (i2c_driver_init(1)!=STATUS_OK) {
		//print_util_putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
		} else {
		//print_util_putstring(STDOUT, "initialised I2C.\n");
	};*/

	LED_On(LED1);
	// Configure the pins connected to LEDs as output and set their default
	// initial state to high (LEDs off).
	//gpio_configure_pin(LED0_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);
	//gpio_configure_pin(LED1_GPIO,GPIO_DIR_OUTPUT | GPIO_INIT_LOW);

	servo_pwm_init();
	servo_pwm_set(&servo_failsafe);
	
		
	uart_int_init(0);
	uart_int_register_write_stream(uart_int_get_uart_handle(0), &(centralData->xbee_out_stream));
				
	//gumstix interface
	uart_int_init(2);
	buffer_make_buffered_stream(&(centralData->gumstix_buffer), &(centralData->gumstix_stream_in));
	uart_int_register_read_stream(uart_int_get_uart_handle(3), &(centralData->gumstix_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(3), &(centralData->gumstix_stream_out));
		
	uart_int_init(3);
	buffer_make_buffered_stream(&(centralData->gps_buffer), &(centralData->gps_stream_in));
	uart_int_register_read_stream(uart_int_get_uart_handle(3), &(centralData->gps_stream_in));
	uart_int_register_write_stream(uart_int_get_uart_handle(3), &(centralData->gps_stream_out));
		
	uart_int_init(4);
	uart_int_register_write_stream(uart_int_get_uart_handle(4), &(centralData->wired_out_stream));


	buffer_make_buffered_stream_lossy(&(centralData->xbee_in_buffer), &(centralData->xbee_in_stream));
	buffer_make_buffered_stream_lossy(&(centralData->wired_in_buffer), &(centralData->wired_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(4), &(centralData->wired_in_stream));
	uart_int_register_read_stream(uart_int_get_uart_handle(0), &(centralData->xbee_in_stream));

		
	// connect abstracted aliases to hardware ports


	centralData->telemetry_down_stream=&(centralData->xbee_out_stream);
	centralData->telemetry_up_stream=&(centralData->xbee_in_stream);
	centralData->debug_out_stream=&(centralData->wired_out_stream);
	centralData->debug_in_stream=&(centralData->wired_in_stream);
/*
	centralData->telemetry_down_stream=&(centralData->wired_out_stream);
	centralData->telemetry_up_stream  =&(centralData->wired_in_stream);		
	centralData->debug_out_stream     =&(centralData->xbee_out_stream);
	centralData->debug_in_stream      =&(centralData->xbee_in_stream);
*/

	//remote_dsm2_rc_activate_bind_mode();

	remote_dsm2_rc_init();

	analog_monitor_init();
	// init mavlink
	mavlink_stream_init(centralData->telemetry_down_stream, centralData->telemetry_up_stream, MAVLINK_SYS_ID);
		
	// init debug output
	print_util_dbg_print_init(centralData->debug_out_stream);
		
	imu_init(&(centralData->imu1));
	bmp085_init();



	Enable_global_interrupt();
	print_util_dbg_print("Board initialised.\n");
}

//board_hardware_t* get_board_hardware() {
	//return &board_hardware;
//}


//byte_stream_t* get_telemetry_upstream() {
	//return board_hardware.telemetry_up_stream;
//}
//byte_stream_t* get_telemetry_downstream() {
	//return board_hardware.telemetry_down_stream;
//}
//byte_stream_t* print_util_get_debug_stream() {
	//return board_hardware.debug_out_stream;
//}
//
//Imu_Data_t* get_imu() {
	//return &board_hardware.imu1;
//}
//Control_Command_t* get_control_inputs() {
	//return &board_hardware.controls;
//}
