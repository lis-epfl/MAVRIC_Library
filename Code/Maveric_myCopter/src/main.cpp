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
//#include "generator.h"

//#include "time_keeper.h"
//#include "streams.h"

//#include "bmp085.h"

//#include "scheduler.h"
#include "central_data.h"
#include "boardsupport.h"
//#include "mavlink_waypoint_handler.h"
#include "navigation.h"
#include "tasks.h"
//#include "neighbor_selection.h"
#include "orca.h"
//#include "flashvault.h"

#include "rectangle.hpp"

central_data_t *centralData;

void initialisation() {
	int i;
	// enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	dbg_print("Initialising...\n");
	
	centralData = get_central_data();
	initialise_board(centralData);
	initialise_central_data();



	init_radar_modules();

	//init_gps_ubx(engine_nav_settings);
	
	init_onboard_parameters();
	init_mavlink_actions();
	init_pos_integration(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);

	
	init_nav();
	init_waypoint_handler();
	//e_init();
	
	init_neighbors();
	init_orca();
	
	create_tasks();

	centralData->imu1.attitude.calibration_level=LEVELING;	
	centralData->mav_state = MAV_STATE_CALIBRATING;
	centralData->mav_mode = MAV_MODE_PREFLIGHT;

	dbg_print("calibrating IMU...\n");
	//calibrate_Gyros(&centralData->imu1);
	for (i=700; i>0; i--) {
		run_imu_update();
		mavlink_protocol_update();	
		delay_ms(5);
	}
	// after initial leveling, initialise accelerometer biases
	
	/*
	centralData->imu1.attitude.calibration_level=LEVEL_PLUS_ACCEL;
	for (i=100; i>0; i--) {
		imu_update(&(centralData->imu1), &centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);	
		mavlink_protocol_update();			
		delay_ms(5);
	}*/
	centralData->imu1.attitude.calibration_level=OFF;
	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
		centralData->position_estimator.localPosition.pos[i]=0.0;
	}
	centralData->mav_state = MAV_STATE_STANDBY;
	centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
	
	LED_On(LED1);
}

int main (void)
{
	
	initialisation();
	
	create_tasks();
	
	// turn on simulation mode: 1: simulation mode, 0: reality
	centralData->simulation_mode = 1;
	
	dbg_print("Initialise HIL Simulator...\n");	
	init_simulation(&(centralData->sim_model),&(centralData->imu1.attitude));

	// main loop
	delay_ms(10);
	dbg_print("Reset home position...\n");
	position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	dbg_print("OK. Starting up.\n");

	Rectangle rec = Rectangle();
	float rectangle_size;
	
	// main loop
	while (1==1) {
		//run_scheduler_update(get_main_taskset(), FIXED_PRIORITY);
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
		
		//LED_On(LED1);
		dbg_print("abcd\n");
		rectangle_size = rec.get_size();
	}

	return 0;		
}


