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

NEW_TASK_SET(main_tasks, 10)


int16_t input_buffer[ADCI_BUFFER_SIZE*4];


	

void initialisation() {
	int i;
	

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
	register_task(&main_tasks, 0, 1000, RUN_REGULAR, &mavlink_protocol_update);
	// main loop
	counter=0;
	// turn on radar power:
	switch_power(1,0);


	Init_ADCI(1000000, ADCIFA_REF06VDD, 1, 1);
	adc_sequencer_add(AVR32_ADCIFA_INP_ADCIN0, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(AVR32_ADCIFA_INP_ADCIN2, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(AVR32_ADCIFA_INP_ADCIN3, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(AVR32_ADCIFA_INP_ADCIN4, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16); 


	ADCI_Start_Sampling(&input_buffer, 4, ADCI_BUFFER_SIZE, Sampling_frequency, false);
	while (1==1) {
		this_looptime=get_millis();
		
		if (ADCI_Sampling_Complete()) {
			//DAC_set_value(0);
			LED_On(LED1);
			calculate_radar();
			mavlink_send_radar();
			mavlink_send_radar_raw();
			//dbg_putfloat(get_tracked_target()->velocity,2);
			//dbg_print(".\n");
			ADCI_Start_Sampling(&input_buffer, 4, ADCI_BUFFER_SIZE, Sampling_frequency, false);
		}			
		
		//run_scheduler_update(&main_tasks, FIXED_PRIORITY);
				
		LED_Off(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


