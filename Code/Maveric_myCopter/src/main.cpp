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
	#include "tasks.h"

	#include "piezo_speaker.h"
}
 
central_data_t *centralData;

void initialisation() 
{
	int i;
	GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;
	
	centralData = central_data_get_pointer_to_struct();
	boardsupport_init(centralData);
	central_data_init();
	
	dbg_print("Debug stream initialised\n");
	
	servos_failsafe(centralData->servos);
	set_servos(centralData->servos);
	
	init_onboard_parameters();


	mavlink_actions_init(); // TODO: move read from flash elsewhere
	// TODO: this second simulation init is required because we have to wait for the parameters stored on flash
	simulation_init(&(centralData->sim_model),&(centralData->imu1),centralData->position_estimator.localPosition); // TODO: init only once

	tasks_relevel_imu(); // TODO: MOVE	

	//reset position estimate
	for (i=0; i<3; i++) 
	{
		// clean acceleration estimate without gravity:
		centralData->position_estimator.vel_bf[i]=0.0;
		centralData->position_estimator.vel[i]=0.0;
		centralData->position_estimator.localPosition.pos[i]=0.0;
	} // TODO: move to module

	delay_ms(10);
	dbg_print("Reset home position...\n");
	position_estimation_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data, &centralData->sim_model.localPosition);
	// TODO: move to module
	
	LED_On(LED1);
	for (i=1; i<8; i++)
	{
		beep(100, 500*i);
		delay_ms(2);
	}
	dbg_print("OK. Starting up.\n");
}

int main (void)
{
	initialisation();
	tasks_create_tasks();
	
	while (1==1) 
	{
		scheduler_run_update(tasks_get_main_taskset(), ROUND_ROBIN);
	}

	return 0;
}