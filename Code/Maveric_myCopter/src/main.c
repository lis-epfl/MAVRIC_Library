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

central_data_t *centralData;

void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	
	
	initialise_board(centralData);
	initialise_central_data();

	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//init_gps_ubx(engine_nav_settings);
	
	set_servos(&servo_failsafe);

	init_onboard_parameters();
	init_mavlink_actions();
	init_pos_integration(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	
	init_nav();
	init_waypoint_handler();
	//e_init();
	
	init_neighbors();
	init_orca();
	
	LED_On(LED1);
}



void main (void)
{
	int i;
	centralData = get_central_data();
	// turn on simulation mode: 1: simulation mode, 0: reality
	centralData->simulation_mode = 1;
	centralData->simulation_mode_previous = centralData->simulation_mode;
	initialisation();
	
	create_tasks();
	
	relevel_imu();

	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
		centralData->position_estimator.localPosition.pos[i]=0.0;
	}
	
	//dbg_print("Initialise HIL Simulator...\n");
	init_simulation(&(centralData->sim_model),&(centralData->imu1.attitude),centralData->position_estimator.localPosition);

	// main loop
	delay_ms(10);
	dbg_print("Reset home position...\n");
	position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data, &centralData->sim_model);
	dbg_print("OK. Starting up.\n");

	while (1==1) {
		
		//run_scheduler_update(get_main_taskset(), FIXED_PRIORITY);
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
		
		//LED_On(LED1);

		
	}		
}


