/**
 * Maveric emulator main file
 *
 * 
 *
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
#include "boardsupport.h"

#include "tasks.h"
#include "all_tests.h"

central_data_t *central_data;


void initialisation() {
	int i;
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;

	central_data = get_central_data();
	initialise_board(central_data);

	init_pos_integration(&central_data->position_estimator, &central_data->pressure, &central_data->GPS_data);
	
	
	init_radar_modules();
	dbg_print("Debug stream initialised\n");

	//init_gps_ubx(engine_nav_settings);
	
	set_servos(&servo_failsafe);
	
	//delay_ms(1000);
	init_stabilisation();

	init_onboard_parameters();
	init_mavlink_actions();
	
	central_data->imu1.attitude.calibration_level=LEVELING;	
	central_data->mav_state = MAV_STATE_CALIBRATING;
	central_data->mav_mode = MAV_MODE_PREFLIGHT;

	//calibrate_Gyros(&centralData->imu1);
	/*for (i=400; i>0; i--) {
		imu_update(&(central_data->imu1), &central_data->position_estimator, &central_data->pressure, &central_data->GPS_data);	
		mavlink_protocol_update();	
		delay_ms(5);
	}
	// after initial leveling, initialise accelerometer biases
	/*
	central_data->imu1.attitude.calibration_level=LEVEL_PLUS_ACCEL;
	for (i=200; i>0; i--) {
		imu_update(&central_data->imu1);
		mavlink_protocol_update();			
		delay_ms(5);
	}*/
	central_data->imu1.attitude.calibration_level=OFF;
	//reset position estimate
	for (i=0; i<3; i++) {
		// clean acceleration estimate without gravity:
		central_data->position_estimator.vel_bf[i]=0.0;
		central_data->position_estimator.vel[i]=0.0;
		central_data->position_estimator.localPosition.pos[i]=0.0;
	}
	central_data->mav_state = MAV_STATE_STANDBY;
	central_data->mav_mode = MAV_MODE_MANUAL_DISARMED;
	
	//e_init();
	
	
}



void main (void)
{
	int i=0;
	int counter=0;
	uint32_t last_looptime, this_looptime;
	
	global_position_t actualPos, originPos;
	local_coordinates_t localPos;
	
	initialisation();
	
	create_tasks();
	
	// turn on simulation mode: 1: simulation mode, 0: reality
	central_data->simulation_mode = 1;
	
	// main loop
	counter=0;
	
	run_all_tests();
	while (1==1) {
		this_looptime=get_millis();
		
		run_scheduler_update(get_main_taskset(), ROUND_ROBIN);
		
		LED_On(LED1);

		//if (counter==0) LED_Toggle(LED1);

		counter=(counter+1)%1000;
		last_looptime=this_looptime;
		usleep(100);
	}		
}


