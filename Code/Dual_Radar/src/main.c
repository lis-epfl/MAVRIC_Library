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
#include "sysclk.h"
#include "sleepmgr.h"
#include "led.h"
#include "delay.h"
//#include "stdio_serial.h"
#include "print_util.h"
#include "generator.h"

#include "time_keeper.h"
#include "i2c_driver_int.h"
#include "qfilter.h"
#include "stabilisation.h"
#include "streams.h"

#include "bmp085.h"

#include "scheduler.h"
#include "boardsupport.h"
#include "mavlink_actions.h"
#include "doppler_radar.h"
#include "radar_driver.h"
#include "i2c_slave_interface.h"



central_data_t *central_data;

int16_t input_buffer[ADCI_BUFFER_SIZE*4];


NEW_TASK_SET(main_tasks, 10)
	

void initialisation() {
	int i;
	irq_initialize_vectors();
	cpu_irq_enable();
	Disable_global_interrupt();
	
	// Initialize the sleep manager
	sleepmgr_init();

	sysclk_init();
	board_init();
	delay_init(sysclk_get_cpu_hz());
	init_time_keeper();
	
	INTC_init_interrupts();
	
	if (init_i2c(1)!=STATUS_OK) {
		//putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
	} else {
		//putstring(STDOUT, "initialised I2C.\n");
	};
	init_i2c_slave_interface(1);

	initialise_board(central_data);
	

	init_radar();

	Enable_global_interrupt();
		
	//dbg_print("Debug stream initialised\n");


	init_onboard_parameters();
	init_mavlink_actions();
	
	
}

void main (void)
{
	int i=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;

	initialisation();
	
	init_scheduler(&main_tasks);
	register_task(&main_tasks, 1, 10000, RUN_REGULAR, &mavlink_protocol_update);
	// main loop
	counter=0;
	// turn on radar power:
	switch_power(1,0);

	ADCI_Start_Sampling(&input_buffer, 4, ADCI_BUFFER_SIZE, Sampling_frequency, false);
	while (1==1) {
		this_looptime=get_millis();
		
		if (ADCI_Sampling_Complete()) {
			DAC_set_value(0);

			calculate_radar();
			mavlink_send_radar();
			ADCI_Start_Sampling(&input_buffer, 4, ADCI_BUFFER_SIZE, Sampling_frequency, false);
		}			
		
		run_scheduler_update(&main_tasks, FIXED_PRIORITY);
				
		LED_On(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


