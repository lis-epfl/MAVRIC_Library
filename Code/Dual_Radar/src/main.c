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

board_hardware_t *board;

pressure_data *pressure;




NEW_TASK_SET(main_tasks, 10)
	
task_return_t run_stabilisation() {
	board->controls.rpy[ROLL]=-getChannel(S_ROLL)/350.0;
	board->controls.rpy[PITCH]=-getChannel(S_PITCH)/350.0;
	board->controls.rpy[YAW]=-getChannel(S_YAW)/350.0;
	board->controls.thrust=getChannel(S_THROTTLE)/350.0;

	imu_update(&(board->imu1));
	if (getChannel(4)>0) {
		board->controls.control_mode=RATE_COMMAND_MODE;
	} else {
		board->controls.control_mode=ATTITUDE_COMMAND_MODE;
	}
	
	// switch run_mode
	if ((board->controls.thrust<-0.95) && (board->controls.rpy[YAW]< -0.9)) {
		board->controls.run_mode=MOTORS_OFF;
		LED_On(LED1);
	}
	if ((board->controls.thrust<-0.95) && (board->controls.rpy[YAW] >0.9)) {
		board->controls.run_mode=MOTORS_ON;
		LED_Off(LED1);
	}
		
	quad_stabilise(&(board->imu1), &(board->controls));

}

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
	
	
	if (init_i2c(0)!=STATUS_OK) {
		//putstring(STDOUT, "Error initialising I2C\n");
		while (1==1);
	} else {
		//putstring(STDOUT, "initialised I2C.\n");
	};

	board=initialise_board();
	
	Enable_global_interrupt();
		
	dbg_print("Debug stream initialised\n");


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
	
	register_task(&main_tasks, 1, 10000, &mavlink_protocol_update);
	// main loop
	counter=0;
	while (1==1) {
		this_looptime=get_millis();
		
		run_scheduler_update(&main_tasks);
				
		LED_On(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;	
	}		
}


