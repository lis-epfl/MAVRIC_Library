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
task_set_t main_tasks;

// size of the double buffer (processing occurs when half is filled)
#define SAMPLE_BUFFER_SIZE 256


int16_t sample_buffer[4][SAMPLE_BUFFER_SIZE];
int16_t input_buffer[4][RADAR_BUFFER_SIZE];


	

void initialisation() {
	int i;
	
	main_tasks.number_of_tasks=30;
	boardsupport_init(central_data);
	

	radar_driver_init();

	Enable_global_interrupt();
		
	//print_util_dbg_print("Debug stream initialised\n");


	onboard_parameters_init();
	mavlink_actions_init();
	
	
}

void main (void)
{
	int i=0;
	int ch=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;
	int wait_for_buffer=0;
	
	initialisation();
	
	scheduler_init(&main_tasks);
	scheduler_register_task(&main_tasks, 0, 1000, RUN_REGULAR, &mavlink_protocol_update);
	// main loop
	counter=0;
	// turn on radar power:
	radar_driver_switch_power(1,0);


	adc_int_init(1500000, ADCIFA_REF1V);
	adc_int_sequencer_add(&sample_buffer[0], AVR32_ADCIFA_INP_ADCIN0, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_int_sequencer_add(&sample_buffer[1], AVR32_ADCIFA_INP_ADCIN1, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_int_sequencer_add(&sample_buffer[2], AVR32_ADCIFA_INP_ADCIN3, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16);  
	adc_int_sequencer_add(&sample_buffer[3], AVR32_ADCIFA_INP_ADCIN4, AVR32_ADCIFA_INN_ADCIN8, ADCIFA_SHG_16); 
	
	LED_Off(LED0);
	delay_ms(1000);
	LED_On(LED0);
	delay_ms(1000);

	
	// start sampling in continuous mode
	adc_int_start_sampling(SAMPLE_BUFFER_SIZE, Sampling_frequency, 16, 1, true);
	while (1==1) {
		this_looptime=time_keeper_get_millis();
		
		if (((adc_int_get_sampling_status()>=SAMPLE_BUFFER_SIZE/2) && (wait_for_buffer==0)) ||
		   ((adc_int_get_sampling_status()<SAMPLE_BUFFER_SIZE/2) && (wait_for_buffer==1)))
		 {  // half of the sample buffer is ready for processing
			// copy samples to end of input buffer
			for (ch=0; ch<4; ch++) {
				for (i=0; i<RADAR_BUFFER_SIZE-(SAMPLE_BUFFER_SIZE/2); i++) {
					input_buffer[ch][i]=input_buffer[ch][i+(SAMPLE_BUFFER_SIZE/2)];
				}
				// get new samples
				if (wait_for_buffer==0) {
					for (i=0; i<SAMPLE_BUFFER_SIZE/2; i++) {
						input_buffer[ch][RADAR_BUFFER_SIZE-(SAMPLE_BUFFER_SIZE/2)+i]=sample_buffer[ch][i];
					}
				} else {
					for (i=0; i<SAMPLE_BUFFER_SIZE/2; i++) {
						input_buffer[ch][RADAR_BUFFER_SIZE-(SAMPLE_BUFFER_SIZE/2)+i]=sample_buffer[ch][i+(SAMPLE_BUFFER_SIZE/2)];
					}
				}
			}
			//adc_int_start_sampling(SAMPLE_BUFFER_SIZE, Sampling_frequency, 16, 1, true);
			wait_for_buffer=1-wait_for_buffer;
			LED_Off(LED1);

			calculate_radar(&input_buffer[0], &input_buffer[1]);
			mavlink_send_radar();
			//mavlink_send_radar_raw();
			
			//print_util_dbg_putfloat(get_tracked_target()->velocity,2);
			//print_util_dbg_print(".\n");

		}			
		
		//scheduler_update(&main_tasks, FIXED_PRIORITY);
		mavlink_schedule_update();
		//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, time_keeper_get_millis(), "ADC_period", adc_int_get_period());

		LED_On(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


