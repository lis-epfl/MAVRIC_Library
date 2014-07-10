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
#include "central_data.h"
#include "boardsupport.h"
#include "mavlink_waypoint_handler.h"
#include "navigation.h"
#include "tasks.h"
#include "neighbor_selection.h"
#include "orca.h"
#include "piezo_speaker.h"

//#include "flashvault.h"

central_data_t *centralData;

void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	
	boardsupport_init(centralData);
	central_data_init();

	radar_module_init();
	print_util_dbg_print("Debug stream initialised\n");

	//gps_ublox_init(engine_nav_settings);
	
	servo_pwm_set(&servo_failsafe);

	onboard_parameters_init();
	mavlink_actions_init();
	position_estimation_init(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	
	navigation_init();
	waypoint_handler_init();
	//e_init();
	
	neighbors_selection_init();
	orca_init();
	
	LED_On(LED1);
	piezo_speaker_init_binary();

}



void main (void)
{
	int i;
	centralData = central_data_get_pointer_to_struct();
	// turn on simulation mode: 1: simulation mode, 0: reality
	centralData->simulation_mode = 1;
	centralData->simulation_mode_previous = centralData->simulation_mode;
	initialisation();
		
	create_tasks();
	
	tasks_relevel_imu();

	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
		centralData->position_estimator.localPosition.pos[i]=0.0;
	}
	
	//print_util_dbg_print("Initialise HIL Simulator...\n");
	simulation_init(&(centralData->sim_model),&(centralData->imu1.attitude),centralData->position_estimator.localPosition);

	// main loop
	delay_ms(10);
	print_util_dbg_print("Reset home position...\n");
	position_estimation_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data, &centralData->sim_model.localPosition);
	print_util_dbg_print("OK. Starting up.\n");

	for (i=1; i<8; i++) {
		piezo_speaker_beep(100, 500*i);
		delay_ms(2);
	}

	while (1==1) {
		
		//scheduler_update(tasks_get_main_taskset(), FIXED_PRIORITY);
		scheduler_update(tasks_get_main_taskset(), ROUND_ROBIN);
		
		//LED_On(LED1);
		delay_ms(1);
		
	}
}
