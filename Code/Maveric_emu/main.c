/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * AVR Software Framework (ASF).
 */
//#include <asf.h>
#include "led.h"
#include "delay.h"
//#include "stdio_serial.h"
#include "print_util.h"


#include "time_keeper.h"
#include "streams.h"

#include "bmp085.h"

#include "scheduler.h"
#include "boardsupport.h"

#include "tasks.h"
//#include "flashvault.h"

board_hardware_t *board;


void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	board=initialise_board();
	
	
	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//init_gps_ubx(engine_nav_settings);
	
	set_servos(&servo_failsafe);
	
	//delay_ms(1000);
	init_stabilisation();

	init_onboard_parameters();
	init_mavlink_actions();
	
	board->imu1.attitude.calibration_level=LEVELING;	
	board->mav_state = MAV_STATE_CALIBRATING;
	board->mav_mode = MAV_MODE_PREFLIGHT;

	calibrate_Gyros(&board->imu1);
	for (i=400; i>0; i--) {
		imu_update(&board->imu1);
		mavlink_protocol_update();	
		delay_ms(5);
	}
	// after initial leveling, initialise accelerometer biases
	/*
	board->imu1.attitude.calibration_level=LEVEL_PLUS_ACCEL;
	for (i=200; i>0; i--) {
		imu_update(&board->imu1);
		mavlink_protocol_update();			
		delay_ms(5);
	}*/
	board->imu1.attitude.calibration_level=OFF;
	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		board->imu1.attitude.vel_bf[i]=0.0;
		board->imu1.attitude.vel[i]=0.0;
		board->imu1.attitude.localPosition.pos[i]=0.0;
	}
	board->mav_state = MAV_STATE_STANDBY;
	board->mav_mode = MAV_MODE_MANUAL_DISARMED;
	
	//e_init();
	
	
}



void main (void)
{
	int i=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;
	
	global_position_t actualPos, originPos;
	local_coordinates_t localPos;
	
	initialisation();
	
	create_tasks();
	
	// turn on simulation mode: 1: simulation mode, 0: reality
	board->simulation_mode = 1;
	
	// main loop
	counter=0;
	while (1==1) {
		this_looptime=get_millis();
		
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
		
		LED_On(LED1);

		//if (counter==0) LED_Toggle(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


