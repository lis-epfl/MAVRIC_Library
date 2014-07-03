/*
 * tasks.c
 *
 * Created: 16/09/2013 13:28:11
 *  Author: sfx
 */ 


#include "tasks.h"
#include "central_data.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "estimator.h"
#include "navigation.h"
#include "led.h"
#include "imu.h"
#include "orca.h"


NEW_TASK_SET(main_tasks, 10)

#define PRESSURE_LPF 0.1

central_data_t *centralData;

task_set* get_main_taskset() {
	return &main_tasks;
}

task_return_t run_imu_update() {
	imu_update(&(centralData->imu1), &centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);	
}	

void rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motorbool)
{
	
	remote_controller_get_channel_mode(chanSwitch);
	
/*	if ((remote_dsm2_rc_get_channel_neutral(RC_TRIM_P3) * RC_SCALEFACTOR)>0.0)
	{
		centralData->collision_avoidance = true;
	}else{
		centralData->collision_avoidance = false;
	}*/
	
	//dbg_print("chanSwitch ");
	//dbg_print_num(*chanSwitch,10);
	//dbg_print_num(getChannel(4),10);
	//dbg_print_num(getChannel(5),10);
	//dbg_print("\n");
	
	if((remote_controller_get_thrust_from_remote()<-0.95) && (remote_controller_get_yaw_from_remote() > 0.9))
	{
		//dbg_print("motor on\n");
		dbg_print("motor on: yaw=\n"); dbg_putfloat(remote_controller_get_yaw_from_remote(),2);
		*motorbool = 1;
	}else if((remote_controller_get_thrust_from_remote()<-0.95) && (remote_controller_get_yaw_from_remote() <-0.9))
	{
		//dbg_print("motor off\n");
		*motorbool = -1;
	}else{
		//dbg_print("motor nothing\n");
		*motorbool = 0;
	}
	
	switch (remote_dsm2_rc_check_receivers())
	{
		case 1:
		*rc_check = 1;
		break;
		case -1:
		*rc_check = -1;
		break;
		case -2:
		*rc_check = -2;
		break;
	}
	//dbg_print("rc_check: ");
	//dbg_print_num(rc_check,10);
	//dbg_print("; motorbool : ");
	//dbg_print_num(*motorbool,10);
	//dbg_print("\n");
}

task_return_t set_mav_mode_n_state()
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	LED_Toggle(LED1);
	
	rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(centralData->mav_state)
	{
		case MAV_STATE_CALIBRATING:
			break;
		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						dbg_print("Switching on the motors!\n");
						position_reset_home_altitude(&centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
						centralData->controls.run_mode = MOTORS_ON;
						centralData->mav_state = MAV_STATE_ACTIVE;
						centralData->mav_mode = MAV_MODE_MANUAL_ARMED;
						break;
					case 1:
						dbg_print("Switches not ready, both should be pushed!\n");
						//centralData->controls.run_mode = MOTORS_ON;
						//centralData->mav_state = MAV_STATE_ACTIVE;
						//centralData->mav_mode = MAV_MODE_STABILIZE_ARMED;
						break;
					case 2:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
					case 3:
						dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			break;
		case MAV_STATE_ACTIVE:
			switch(channelSwitches)
			{
				case 0:
					centralData->waypoint_hold_init = false;
					centralData->mav_mode= MAV_MODE_MANUAL_ARMED;
					break;
				case 1:
					centralData->waypoint_hold_init = false;
					centralData->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 2:
					centralData->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 3:
					
					centralData->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
			
			switch (centralData->mav_mode)
			{
				case MAV_MODE_MANUAL_ARMED:
					centralData->waypoint_hold_init = false;
					break;
				case MAV_MODE_STABILIZE_ARMED:
					centralData->waypoint_hold_init = false;
					break;
				case MAV_MODE_GUIDED_ARMED:
					waypoint_hold_position_handler();
					break;
				case MAV_MODE_AUTO_ARMED:
					if (centralData->waypoint_set)
					{
						centralData->waypoint_hold_init = false;
					}
					waypoint_navigation_handler();
					break;
			}
			
			//dbg_print("motor_switch: ");
			//dbg_print_num(motor_switch,10);
			if (motor_switch == -1)
			{
				dbg_print("Switching off motors!\n");
				centralData->controls.run_mode = MOTORS_OFF;
				centralData->mav_state = MAV_STATE_STANDBY;
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
		
			switch (RC_check)
			{
				case 1:
					break;
				case -1:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
				case -2:
					centralData->mav_state = MAV_STATE_CRITICAL;
					break;
			}
			break;
		case MAV_STATE_CRITICAL:
			switch(channelSwitches)
			{
				case 0:
					centralData->mav_mode= MAV_MODE_MANUAL_ARMED;
					break;
				case 1:
					centralData->mav_mode= MAV_MODE_STABILIZE_ARMED;
					break;
				case 2:
					centralData->mav_mode = MAV_MODE_GUIDED_ARMED;
					break;
				case 3:
					centralData->mav_mode = MAV_MODE_AUTO_ARMED;
					break;
			}
			if (motor_switch == -1)
			{
				dbg_print("Switching off motors!\n");
				centralData->controls.run_mode = MOTORS_OFF;
				centralData->mav_state = MAV_STATE_STANDBY;
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
			}
			
			switch (centralData->mav_mode)
			{
				case MAV_MODE_GUIDED_ARMED:
				case MAV_MODE_AUTO_ARMED:
					waypoint_critical_handler();
					break;
			}
			
			switch (RC_check)
			{
				case 1:
					centralData->mav_state = MAV_STATE_ACTIVE;
					centralData->critical_init = false;
					break;
				case -1:
					break;
				case -2:
					if (centralData->critical_landing)
					{
						centralData->mav_state = MAV_STATE_EMERGENCY;
					}
					//centralData->mav_state = MAV_STATE_EMERGENCY;
					break;
			}
			break;
		case MAV_STATE_EMERGENCY:
			if (centralData->position_estimator.localPosition.pos[Z] < 1.0)
			{
				centralData->mav_mode = MAV_MODE_MANUAL_DISARMED;
				centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
				switch (RC_check)
				{
					case 1:
						centralData->mav_state = MAV_STATE_STANDBY;
						break;
					case -1:
						break;
					case -2:
						break;
				}
			}
			break;
	}

	//dbg_print("MAV state :");
	//dbg_print_num(centralData->mav_state,10);
	//dbg_print(", MAV mode :");
	//dbg_print_num(centralData->mav_mode,10);
	//dbg_print("\n");
	
}

task_return_t run_stabilisation() {
	int i;
	
	if (centralData->simulation_mode==1) {
		simu_update(&centralData->sim_model, &centralData->servos, &(centralData->imu1), &centralData->position_estimator);
		
		
		imu_update(&(centralData->imu1), &(centralData->position_estimator), &centralData->pressure, &centralData->GPS_data);
		
		//for (i=0; i<3; i++) centralData->position_estimator.vel[i]=centralData->sim_model.vel[i];
		//centralData->position_estimator.localPosition=centralData->sim_model.localPosition;
	} else {
		imu_get_raw_data(&(centralData->imu1));
		imu_update(&(centralData->imu1), &centralData->position_estimator, &centralData->pressure, &centralData->GPS_data);
	}

	switch(centralData->mav_mode)
	{
		
		case MAV_MODE_MANUAL_ARMED:
			centralData->controls = remote_controller_get_command_from_remote();
			
			centralData->controls.yaw_mode=YAW_RELATIVE;
			centralData->controls.control_mode = RATE_COMMAND_MODE;
			
			stabilisation_hybrid_cascade_stabilise_hybrid(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		case MAV_MODE_STABILIZE_ARMED:
			centralData->controls = remote_controller_get_command_from_remote();
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
			
			stabilisation_hybrid_cascade_stabilise_hybrid(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			
			break;
		case MAV_MODE_GUIDED_ARMED:
			// centralData->controls = centralData->controls_nav;
			centralData->controls = remote_controller_get_command_from_remote();
			
			
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;
			stabilisation_hybrid_cascade_stabilise_hybrid(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		case MAV_MODE_AUTO_ARMED:
			// centralData->controls = centralData->controls_nav;
			centralData->controls = remote_controller_get_command_from_remote();
			
			centralData->controls.control_mode = ATTITUDE_COMMAND_MODE;	
			stabilisation_hybrid_cascade_stabilise_hybrid(&(centralData->imu1), &centralData->position_estimator, &(centralData->controls));
			break;
		
		case MAV_MODE_PREFLIGHT:
		case MAV_MODE_MANUAL_DISARMED:
		case MAV_MODE_STABILIZE_DISARMED:
		case MAV_MODE_GUIDED_DISARMED:
		case MAV_MODE_AUTO_DISARMED:
			centralData->controls.run_mode = MOTORS_OFF;
			for (i=0; i<NUMBER_OF_SERVO_OUTPUTS; i++) {
				centralData->servos[i]=servo_failsafe[i];
			}
			break;
		
	}
	
	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if (centralData->simulation_mode!=1) {
		servo_pwm_set(&(centralData->servos));
	}
		

}

task_return_t gps_task() {
	uint32_t tnow = time_keeper_get_millis();	
	if (centralData->simulation_mode==1) {
		simulate_gps(&centralData->sim_model, &centralData->GPS_data);
	} else {
		gps_update();
	}
}

task_return_t run_estimator()
{
	estimator_loop();
}

task_return_t run_navigation_task()
{
	int8_t i;
	
		switch (centralData->mav_state)
		{
			case MAV_STATE_ACTIVE:
				switch (centralData->mav_mode)
				{
					case MAV_MODE_AUTO_ARMED:
						if (centralData->waypoint_set)
						{
							navigation_run(centralData->waypoint_coordinates);
					
							//orca_computeNewVelocity(centralData->controls_nav.tvel,newVelocity);
						}else if(centralData->waypoint_hold_init)
						{
							navigation_run(centralData->waypoint_hold_coordinates);
						}
						break;
					case MAV_MODE_GUIDED_ARMED:
						if(centralData->waypoint_hold_init)
						{
							navigation_run(centralData->waypoint_hold_coordinates);
						}
						break;
				}
				break;
			case MAV_STATE_CRITICAL:
				if ((centralData->mav_mode == MAV_MODE_GUIDED_ARMED)||(centralData->mav_mode == MAV_MODE_AUTO_ARMED))
				{
					if(centralData->critical_init)
					{
						navigation_run(centralData->waypoint_critical_coordinates);
					}
				}
				break;
		}
	
}
uint32_t last_baro_update;
task_return_t run_barometer()
{
	uint32_t tnow = time_keeper_get_micros();
	central_data_t *central_data=get_central_data();
	
	pressure_data *pressure= bmp085_get_pressure_data_slow(centralData->pressure.altitude_offset);
	if (central_data->simulation_mode==1) {
		simulate_barometer(&centralData->sim_model, pressure);
	} 
	centralData->pressure=*pressure;
	
}


void create_tasks() {
	
	init_scheduler(&main_tasks);
	
	centralData = get_central_data();
	
	register_task(&main_tasks, 0, 4000, RUN_REGULAR, &run_stabilisation );
	
	register_task(&main_tasks, 1, 15000, RUN_REGULAR, &run_barometer);
	main_tasks.tasks[1].timing_mode=PERIODIC_RELATIVE;

	register_task(&main_tasks, 2, 100000, RUN_REGULAR, &gps_task);
	//register_task(&main_tasks, 4, 4000, RUN_REGULAR, &run_estimator);
	//register_task(&main_tasks, , 100000, RUN_REGULAR, &radar_module_read);

	register_task(&main_tasks, 3, ORCA_TIME_STEP_MILLIS * 1000.0, RUN_REGULAR, &run_navigation_task);

	register_task(&main_tasks, 4, 200000, RUN_REGULAR, &set_mav_mode_n_state);
	

	register_task(&main_tasks, 5, 4000, RUN_REGULAR, &mavlink_stream_protocol_update);

}