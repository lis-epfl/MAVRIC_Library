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

//NEW_TASK_SET(main_tasks, 10)
task_set main_tasks;

int16_t input_buffer[4][ADCI_BUFFER_SIZE];


	

void initialisation() {
	int i;
	
	main_tasks.number_of_tasks=30;
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


	Init_ADCI(1500000, ADCIFA_REF1V);
	adc_sequencer_add(&input_buffer[0], AVR32_ADCIFA_INP_ADCIN0, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(&input_buffer[1], AVR32_ADCIFA_INP_ADCIN1, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(&input_buffer[2], AVR32_ADCIFA_INP_ADCIN3, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_sequencer_add(&input_buffer[3], AVR32_ADCIFA_INP_ADCIN4, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16); 
	
	LED_On(LED0);
	delay_ms(1000);
	LED_Off(LED0);
	delay_ms(1000);

	
	
	ADCI_Start_Sampling(ADCI_BUFFER_SIZE, Sampling_frequency, 16, 1, false);
	while (1==1) {
		this_looptime=get_millis();
		
		if (ADCI_Sampling_Complete()) {
			//DAC_set_value(0);
			LED_On(LED1);
			calculate_radar(&input_buffer[0], &input_buffer[1]);
			mavlink_send_radar();
			//mavlink_send_radar_raw();
			
			//dbg_putfloat(get_tracked_target()->velocity,2);
			//dbg_print(".\n");
			ADCI_Start_Sampling(ADCI_BUFFER_SIZE, Sampling_frequency, 16, 1, false);

		}			
		
		//run_scheduler_update(&main_tasks, FIXED_PRIORITY);
		mavlink_protocol_update();
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "ADC_period", get_adc_int_period());

		LED_Off(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


