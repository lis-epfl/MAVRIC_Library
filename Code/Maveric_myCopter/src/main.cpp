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
	int32_t i;
	
	centralData = central_data_get_pointer_to_struct();
	boardsupport_init(centralData);
	central_data_init();
	
	print_util_dbg_print("Debug stream initialised\n");
	
	servo_pwm_failsafe(centralData->servos);
	servo_pwm_set(centralData->servos);
	
//	onboard_parameters_init(&centralData->onboard_parameters);	// TODO: remove
	
	mavlink_actions_init(); // TODO: move read from flash elsewhere
	mavlink_telemetry_init();
	
		/*	
	 * Use this to store or read or reset parameters on flash memory
	 *
	onboard_parameters_write_parameters_to_flashc();
	if(onboard_parameters_read_parameters_from_flashc())
	{
		simulation_calib_set(&(centralData->sim_model));
	}
	 *
	 */

	tasks_relevel_imu(); // TODO: MOVE	

	delay_ms(10);
	print_util_dbg_print("Reset home position...\n");
	position_estimation_reset_home_altitude(&centralData->position_estimator);
	
	LED_On(LED1);
	for (i = 1; i < 8; i++)
	{
		piezo_speaker_beep(100, 500 * i);
		delay_ms(2);
	}
	print_util_dbg_print("OK. Starting up.\n");
	
	centralData->state_structure.mav_state = MAV_STATE_STANDBY;
	centralData->state_structure.mav_state_previous = centralData->state_structure.mav_state;
}

int main (void)
{
	initialisation();
	tasks_create_tasks();
	
	while (1 == 1) 
	{
		scheduler_update(tasks_get_main_taskset(), ROUND_ROBIN);
	}

	return 0;
}