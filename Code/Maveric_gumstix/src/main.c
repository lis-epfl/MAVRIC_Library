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
#include <asf.h>
#include "led.h"
#include "delay.h"
//#include "stdio_serial.h"
#include "print_util.h"
#include "generator.h"

#include "time_keeper.h"
#include "streams.h"

#include "bmp085.h"

#include "scheduler.h"
#include "central_data.h"
#include "boardsupport.h"
#include "mavlink_waypoint_handler.h"
#include "navigation.h"
#include "tasks.h"
#include "neighbor_selection.h"
#include "orca.h"
//#include "flashvault.h"

#include "gumstix_api.h"

central_data_t *centralData;

void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	
	centralData = central_data_get_pointer_to_struct();
	boardsupport_init(centralData);
	central_data_init();
	

	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//gps_ublox_init(engine_nav_settings);
	
	set_servos(&servo_failsafe);

	init_onboard_parameters();
	mavlink_actions_init();
	position_estimation_init(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	
	centralData->imu1.attitude.calibration_level=LEVELING;	
	centralData->mav_state = MAV_STATE_BOOT;
	centralData->mav_mode = MAV_MODE_PREFLIGHT;

//	imu_calibrate_gyros(&centralData->imu1);
	for (i=400; i>0; i--) {
		tasks_run_imu_update();
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
	
	centralData->mav_mode_previous = centralData->mav_mode;
	centralData->mav_state_previous = centralData->mav_state;
	
	init_nav();
	init_waypoint_handler();
	//e_init();
	
	neighbors_selection_init();
	init_orca();
	
	//init gumstix
	//gum_Setup();
	
	LED_On(LED1);
}



void main (void)
{
	
	initialisation();
	
	create_tasks();
	
	// turn on simulation mode: 1: simulation mode, 0: reality
	centralData->simulation_mode = 0;
	
	// main loop
	
	while (1==1) {
		
		//scheduler_run_update(tasks_get_main_taskset(), FIXED_PRIORITY);
		scheduler_run_update(tasks_get_main_taskset(), ROUND_ROBIN);
		
		//LED_On(LED1);

		
	}		
}


