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
	#include "mavlink_telemetry.h"
	#include "piezo_speaker.h"
}
 
central_data_t *centralData;

void initialisation() 
{	
	centralData = central_data_get_pointer_to_struct();
	boardsupport_init(centralData);
	central_data_init();
		
	mavlink_actions_init(); // TODO: move read from flash elsewhere
	mavlink_telemetry_init();
		
	LED_On(LED1);

	piezo_speaker_startup_melody();

	print_util_dbg_print("OK. Starting up.\n");
	
	centralData->state_structure.mav_state = MAV_STATE_STANDBY;
	centralData->state_structure.mav_state_previous = centralData->state_structure.mav_state;
	
	centralData->imu.calibration_level = OFF;
}

int main (void)
{
	initialisation();
	tasks_create_tasks();
	
	while (1 == 1) 
	{
		scheduler_update(&centralData->scheduler);
	}

	return 0;
}