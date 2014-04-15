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

extern "C" {
	#include "led.h"
	#include "delay.h"
	#include "print_util.h"
	#include "central_data.h"
	#include "boardsupport.h"
	#include "navigation.h"
	#include "tasks.h"
	#include "orca.h"
	#include "piezo_speaker.h"
}
 
//#include <asf.h>
//#include "stdio_serial.h"
//#include "mavlink_waypoint_handler.h"
//#include "neighbor_selection.h"
//#include "flashvault.h"
//#include "generator.h"
//#include "time_keeper.h"
//#include "streams.h"
//#include "bmp085.h"
//#include "scheduler.h"

central_data_t *centralData;

void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	
	centralData = get_central_data();
	initialise_board(centralData);
	initialise_central_data();

	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//init_gps_ubx(engine_nav_settings);
	
	servos_failsafe(centralData->servos);
	set_servos(centralData->servos);
	
	init_onboard_parameters();
	init_mavlink_actions();
	init_pos_integration(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	
	init_nav();
	init_waypoint_handler();
	//e_init();
	
	init_neighbors();
	init_orca();
	
	LED_On(LED1);
	init_piezo_speaker_binary();

}



int main (void)
{
	int i;
	// turn on simulation mode: 1: simulation mode, 0: reality
	initialisation();
	centralData->simulation_mode = 1;
	centralData->simulation_mode_previous = 1;
	
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
	position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data, &centralData->sim_model.localPosition);
	dbg_print("OK. Starting up.\n");

	for (i=1; i<8; i++) {
		beep(100, 500*i);
		delay_ms(2);
	}

	while (1==1) {
		
		//run_scheduler_update(get_main_taskset(), FIXED_PRIORITY);
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
		
		//LED_On(LED1);		
	}
	return 0;		
}