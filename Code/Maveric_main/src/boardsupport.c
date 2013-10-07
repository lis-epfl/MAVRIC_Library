/*
 * boardsupport.c
 *
 * Created: 20/03/2013 12:14:18
 *  Author: sfx
 */ 

#include "boardsupport.h"
#include "waypoint_navigation.h"
#include "conf_sim_model.h"
#include "sysclk.h"
#include "sleepmgr.h"
#include "led.h"
#include "delay.h"

static volatile board_hardware_t board_hardware;

board_hardware_t* initialise_board() {
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
		init_time_keeper();
		//board=initialise_board();
		
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
		set_servos(&servo_failsafe);
	
		
		init_UART_int(0);
		register_write_stream(get_UART_handle(0), &board_hardware.xbee_out_stream);
				
		
		init_UART_int(3);
		make_buffered_stream(&(board_hardware.gps_buffer), &board_hardware.gps_stream_in);
		register_read_stream(get_UART_handle(3), &board_hardware.gps_stream_in);
		register_write_stream(get_UART_handle(3), &board_hardware.gps_stream_out);
		
		init_UART_int(4);
		register_write_stream(get_UART_handle(4), &board_hardware.wired_out_stream);


		make_buffered_stream_lossy(&board_hardware.xbee_in_buffer, &board_hardware.xbee_in_stream);
		make_buffered_stream_lossy(&board_hardware.wired_in_buffer, &board_hardware.wired_in_stream);
		register_read_stream(get_UART_handle(4), &board_hardware.wired_in_stream);
		register_read_stream(get_UART_handle(0), &board_hardware.xbee_in_stream);

		
		// connect abstracted aliases to hardware ports


		board_hardware.telemetry_down_stream=&board_hardware.xbee_out_stream;
		board_hardware.telemetry_up_stream=&board_hardware.xbee_in_stream;
		board_hardware.debug_out_stream=&board_hardware.wired_out_stream;
		board_hardware.debug_in_stream=&board_hardware.wired_in_stream;
/*
		board_hardware.telemetry_down_stream=&board_hardware.wired_out_stream;
		board_hardware.telemetry_up_stream  =&board_hardware.wired_in_stream;		
		board_hardware.debug_out_stream     =&board_hardware.xbee_out_stream;
		board_hardware.debug_in_stream      =&board_hardware.xbee_in_stream;
*/

		// init mavlink
		init_mavlink(board_hardware.telemetry_down_stream, board_hardware.telemetry_up_stream, MAVLINK_SYS_ID);
		
		// init debug output
		dbg_print_init(board_hardware.debug_out_stream);
		
		init_imu(&board_hardware.imu1);
		init_bmp085();

		rc_init();

		init_simulation(&board_hardware.sim_model);
		
		board_hardware.controls.rpy[ROLL]=0;
		board_hardware.controls.rpy[PITCH]=0;
		board_hardware.controls.rpy[YAW]=0;
		board_hardware.controls.thrust=-1.0;
		
		board_hardware.number_of_waypoints = 0;

		board_hardware.simulation_mode=0;
		
		board_hardware.waypoint_set = false;
		board_hardware.mission_started = false;
		board_hardware.waypoint_sending = false;
		board_hardware.waypoint_receiving = false;
		board_hardware.waypoint_hold_init = false;
		
		// default GPS home position
		board_hardware.imu1.attitude.localPosition.origin.longitude=   HOME_LONGITUDE;
		board_hardware.imu1.attitude.localPosition.origin.latitude =   HOME_LATITUDE;
		board_hardware.imu1.attitude.localPosition.origin.altitude =   HOME_ALTITUDE;
		board_hardware.imu1.attitude.localPosition.pos[0]=0;
		board_hardware.imu1.attitude.localPosition.pos[1]=0; 
		board_hardware.imu1.attitude.localPosition.pos[2]=0;
		
		init_waypoint_list(board_hardware.waypoint_list,&board_hardware.number_of_waypoints);

		Enable_global_interrupt();
		dbg_print("Board initialised.\n");

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
	return board_hardware.debug_out_stream;
}

Imu_Data_t* get_imu() {
	return &board_hardware.imu1;
}
Control_Command_t* get_control_inputs() {
	return &board_hardware.controls;
}
