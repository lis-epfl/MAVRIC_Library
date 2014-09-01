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
	
	#include "gpio.h"
	#include "spi.h"
	#include "sd_spi.h"
}
 
central_data_t *central_data;

void initialisation() 
{	
	central_data = central_data_get_pointer_to_struct();
	boardsupport_init(central_data);
	central_data_init();

	mavlink_telemetry_init();
	
	piezo_speaker_startup_melody();

	onboard_parameters_read_parameters_from_flashc(&central_data->mavlink_communication.onboard_parameters);

	// Switch off red LED
	LED_Off(LED2);
	
	print_util_dbg_print("OK. Starting up.\n");

	central_data->state.mav_state = MAV_STATE_STANDBY;

	central_data->imu.calibration_level = OFF;
}

int main (void)
{
	initialisation();
	tasks_create_tasks();
	
	while (1 == 1) 
	{
		scheduler_update(&central_data->scheduler);
	}

	return 0;
}