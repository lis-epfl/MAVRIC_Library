/**
 * \page The MAV'RIC License
 *
 * The MAV'RIC Framework
 *
 * Copyright © 2011-2014
 *
 * Laboratory of Intelligent Systems, EPFL
 */


/**
 * \file tasks.c
 *
 * Definition of the tasks executed on the autopilot
 */ 


#include "tasks.h"
#include "central_data.h"
#include "print_util.h"
#include "stabilisation.h"
#include "gps_ublox.h"
#include "navigation.h"
#include "led.h"
#include "imu.h"
#include "orca.h"
#include "delay.h"
#include "i2cxl_sonar.h"
#include "analog_monitor.h"
#include "lsm330dlc_driver.h"
#include "compass_hmc5883l.h"

central_data_t* central_data;

/**
 * \brief	Function to call when the motors should be switched off
 */
void switch_off_motors(void);

task_set_t* tasks_get_main_taskset() 
{
	central_data = central_data_get_pointer_to_struct();

	return central_data->scheduler.task_set;
}

void tasks_rc_user_channels(uint8_t *chanSwitch, int8_t *rc_check, int8_t *motor_state)
{
	
	remote_controller_get_channel_mode(chanSwitch);
	
	if ((remote_dsm2_rc_get_channel_neutral(RC_TRIM_P3) * RC_SCALEFACTOR) > 0.0f)
	{
		central_data->waypoint_handler.collision_avoidance = true;
	}
	else
	{
		central_data->waypoint_handler.collision_avoidance = false;
	}
	
	remote_controller_get_motor_state(motor_state);
	
	*rc_check = remote_dsm2_rc_check_receivers();
	}

void switch_off_motors(void)
{
	print_util_dbg_print("Switching off motors!\n");

	central_data->state_structure.mav_state = MAV_STATE_STANDBY;
	state_disable_mode(&central_data->state_structure,MAV_MODE_FLAG_SAFETY_ARMED);
	state_set_new_mode(&central_data->state_structure,MAV_MODE_SAFE);
	
	central_data->waypoint_handler.in_the_air = false;
}

task_return_t tasks_set_mav_mode_n_state(void* arg)
{
	uint8_t channelSwitches = 0;
	int8_t RC_check = 0;
	int8_t motor_switch = 0;
	
	float distFromHomeSqr;
	
	LED_Toggle(LED1);
	
	tasks_rc_user_channels(&channelSwitches,&RC_check, &motor_switch);
	
	switch(central_data->state_structure.mav_state)
	{
		case MAV_STATE_BOOT:
			break;

		case MAV_STATE_CALIBRATING:
			break;

		case MAV_STATE_STANDBY:
			if (motor_switch == 1)
			{
				switch(channelSwitches)
				{
					case 0:
						print_util_dbg_print("Switching on the motors!\n");

						position_estimation_reset_home_altitude(&central_data->position_estimator);
						
						central_data->waypoint_handler.waypoint_set = false;
						state_set_new_mode(&central_data->state_structure,MAV_MODE_ATTITUDE_CONTROL);
						break;

					default:
						print_util_dbg_print("Switches not ready, both should be pushed!\n");
						break;
				}
			}
			if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_SAFETY_ARMED))
			{
				switch (channelSwitches)
				{
					case 0:
						state_set_new_mode(&central_data->state_structure,MAV_MODE_ATTITUDE_CONTROL);
						if (central_data->waypoint_handler.in_the_air)
						{
							central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
						}
						break;

					case 1:
						state_set_new_mode(&central_data->state_structure,MAV_MODE_VELOCITY_CONTROL);
						if (central_data->waypoint_handler.in_the_air)
						{
							central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
						}
						break;

					case 2:
						if (central_data->waypoint_handler.in_the_air)
						{
							state_set_new_mode(&central_data->state_structure,MAV_MODE_POSITION_HOLD);
							
							// Activate automatic take-off mode
							if (state_test_if_first_time_in_mode(&central_data->state_structure,MAV_MODE_POSITION_HOLD))
							{
								waypoint_handler_waypoint_take_off(&central_data->waypoint_handler);
							}
							
							distFromHomeSqr =	SQR(central_data->position_estimator.localPosition.pos[X] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[X]) +
							SQR(central_data->position_estimator.localPosition.pos[Y] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[Y]) +
							SQR(central_data->position_estimator.localPosition.pos[Z] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[Z]);
						
							if (central_data->waypoint_handler.dist2wp_sqr <= 16.0f)
							{
								central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
								print_util_dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
								print_util_dbg_print_num(distFromHomeSqr * 10.0f,10);
								print_util_dbg_print(".\n");
							}
						}
						break;

					case 3:
						if (central_data->waypoint_handler.in_the_air)
						{
							//central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
							state_set_new_mode(&central_data->state_structure,MAV_MODE_GPS_NAVIGATION);
							
							// Automatic take-off mode
							if(state_test_if_first_time_in_mode(&central_data->state_structure,MAV_MODE_GPS_NAVIGATION))
							{
								waypoint_handler_waypoint_take_off(&central_data->waypoint_handler);
							}

							if (!central_data->waypoint_handler.waypoint_set)
							{
								waypoint_handler_waypoint_init(&central_data->waypoint_handler);
							}

							distFromHomeSqr =	SQR(central_data->position_estimator.localPosition.pos[X] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[X]) +
							SQR(central_data->position_estimator.localPosition.pos[Y] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[Y]) +
							SQR(central_data->position_estimator.localPosition.pos[Z] - central_data->waypoint_handler.waypoint_hold_coordinates.pos[Z]);
						
							if (central_data->waypoint_handler.dist2wp_sqr <= 16.0f)
							{
								central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
								print_util_dbg_print("Automatic take-off finised, distFromHomeSqr (10x):");
								print_util_dbg_print_num(distFromHomeSqr * 10.0f,10);
								print_util_dbg_print(".\n");
							}
						}
						break;
				}
				
				switch (RC_check)
				{
					case 1:
						break;

					case -1:
						central_data->state_structure.mav_state = MAV_STATE_CRITICAL;
						break;

					case -2:
						central_data->state_structure.mav_state = MAV_STATE_CRITICAL;
						break;
				}
				if (remote_controller_get_thrust_from_remote() > -0.7f)
				{
					central_data->waypoint_handler.in_the_air = true;
				}
			}

			if (motor_switch == -1)
			{
				switch_off_motors();
			}
			
			break;

		case MAV_STATE_ACTIVE:
			switch(channelSwitches)
			{
				case 0:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_ATTITUDE_CONTROL);
					break;

				case 1:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_VELOCITY_CONTROL);
					break;

				case 2:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_POSITION_HOLD);
					if (state_test_if_first_time_in_mode(&central_data->state_structure,MAV_MODE_POSITION_HOLD))
					{
						waypoint_handler_waypoint_hold_init(&central_data->waypoint_handler,central_data->position_estimator.localPosition);
					}
					break;

				case 3:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_GPS_NAVIGATION);
					if (state_test_if_first_time_in_mode(&central_data->state_structure,MAV_MODE_GPS_NAVIGATION))
					{
						central_data->waypoint_handler.auto_landing_enum = DESCENT_TO_SMALL_ALTITUDE;
						waypoint_handler_waypoint_hold_init(&central_data->waypoint_handler,central_data->position_estimator.localPosition);
					}
					break;
			}
			
			if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_AUTO_ENABLED))
			{
				if (!central_data->waypoint_handler.waypoint_set)
				{
					waypoint_handler_waypoint_init(&central_data->waypoint_handler);
				}

				waypoint_handler_waypoint_navigation_handler(&central_data->waypoint_handler);
			}
			
			if (motor_switch == -1)
			{
				switch_off_motors();
			}
		
			switch (RC_check)
			{
				case 1:
					break;

				case -1:
					central_data->state_structure.mav_state = MAV_STATE_CRITICAL;
					break;

				case -2:
					central_data->state_structure.mav_state = MAV_STATE_CRITICAL;
					break;
			}
			break;

		case MAV_STATE_CRITICAL:
			switch(channelSwitches)
			{
				case 0:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_ATTITUDE_CONTROL);
					break;

				case 1:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_VELOCITY_CONTROL);
					break;

				case 2:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_POSITION_HOLD);
					break;

				case 3:
					state_set_new_mode(&central_data->state_structure,MAV_MODE_GPS_NAVIGATION);
					break;
			}

			if (motor_switch == -1)
			{
				switch_off_motors();
			}
			
			// In MAV_MODE_VELOCITY_CONTROL, MAV_MODE_POSITION_HOLD and MAV_MODE_GPS_NAVIGATION
			if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_STABILIZE_ENABLED))
			{
				waypoint_handler_waypoint_critical_handler(&central_data->waypoint_handler);
			}
			
			switch (RC_check)
			{
				case 1:  
					// !! only if receivers are back, switch into appropriate mode
					central_data->state_structure.mav_state = MAV_STATE_ACTIVE;
					central_data->waypoint_handler.critical_behavior = CLIMB_TO_SAFE_ALT;
					central_data->waypoint_handler.critical_next_state = false;
					break;

				case -1:
					break;

				case -2:
					if (central_data->waypoint_handler.critical_landing)
					{
						central_data->state_structure.mav_state = MAV_STATE_EMERGENCY;
					}

					break;
			}
			break;

		case MAV_STATE_EMERGENCY:
			state_set_new_mode(&central_data->state_structure,MAV_MODE_SAFE);
			switch (RC_check)
			{
				case 1:
					central_data->state_structure.mav_state = MAV_STATE_STANDBY;
					break;

				case -1:
					break;

				case -2:
					break;
			}
			break;
	}
	
	central_data->state_structure.mav_mode_previous = central_data->state_structure.mav_mode;
	
	if (central_data->state_structure.simulation_mode_previous != central_data->state_structure.simulation_mode)
	{
		simulation_switch_between_reality_n_simulation(&central_data->sim_model);
		central_data->state_structure.simulation_mode_previous = central_data->state_structure.simulation_mode;
	}
	
	return TASK_RUN_SUCCESS;
}


void tasks_run_imu_update(void* arg)
{
	if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_HIL_ENABLED))
	{
		simulation_update(&central_data->sim_model);
	} 
	else 
	{
		lsm330dlc_gyro_update(&(central_data->imu.raw_gyro));
		lsm330dlc_acc_update(&(central_data->imu.raw_accelero));
		compass_hmc58831l_update(&(central_data->imu.raw_compass));
	}
	
	imu_update(	&central_data->imu);
	qfilter_update(&central_data->attitude_filter);
	
	if (central_data->imu.calibration_level == OFF)
	{
		position_estimation_update(&central_data->position_estimator);
	}
}


task_return_t tasks_run_stabilisation(void* arg) 
{
	tasks_run_imu_update(0);
	
	switch(central_data->state_structure.mav_mode - (central_data->state_structure.mav_mode & MAV_MODE_FLAG_DECODE_POSITION_HIL))
	{
		case MAV_MODE_ATTITUDE_CONTROL:
			remote_controller_get_command_from_remote(&central_data->controls);
			central_data->controls.control_mode = ATTITUDE_COMMAND_MODE;
			central_data->controls.yaw_mode=YAW_RELATIVE;
		
			stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
			break;
		
		case MAV_MODE_VELOCITY_CONTROL:
			remote_controller_get_velocity_vector_from_remote(&central_data->controls);
			
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
			central_data->controls.yaw_mode = YAW_RELATIVE;
			
			stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
		
			break;
		
		case MAV_MODE_POSITION_HOLD:
			central_data->controls = central_data->controls_nav;
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
		
			if ((central_data->state_structure.mav_state == MAV_STATE_CRITICAL) && (central_data->waypoint_handler.critical_behavior == FLY_TO_HOME_WP))
			{
				central_data->controls.yaw_mode = YAW_COORDINATED;
			}
			else
			{
				central_data->controls.yaw_mode = YAW_ABSOLUTE;
			}
		
			stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
		
			break;
		
		case MAV_MODE_GPS_NAVIGATION:
			central_data->controls = central_data->controls_nav;
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
			
			// if no waypoints are set, we do position hold therefore the yaw mode is absolute
			if (((central_data->waypoint_handler.waypoint_set&&(central_data->state_structure.mav_state != MAV_STATE_STANDBY)))||((central_data->state_structure.mav_state == MAV_STATE_CRITICAL)&&(central_data->waypoint_handler.critical_behavior == FLY_TO_HOME_WP)))
			{
				central_data->controls.yaw_mode = YAW_COORDINATED;
			}
			else
			{
				central_data->controls.yaw_mode = YAW_ABSOLUTE;
			}
		
			stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
			break;
		
		default:
			servo_pwm_failsafe(central_data->servos);
			break;
	}
	
	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if (!state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_HIL_ENABLED))
	{
		servo_pwm_set(central_data->servos);
	}
	
	return TASK_RUN_SUCCESS;
}

task_return_t tasks_run_gps_update(void* arg) 
{
	if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_HIL_ENABLED))
	{
		simulation_simulate_gps(&central_data->sim_model);
	} 
	else 
	{
		gps_ublox_update(&central_data->GPS_data);
	}
	
	return TASK_RUN_SUCCESS;
}


task_return_t tasks_run_barometer_update(void* arg)
{
	if (state_test_if_in_flag_mode(&central_data->state_structure,MAV_MODE_FLAG_HIL_ENABLED))
	{
		simulation_simulate_barometer(&central_data->sim_model);
	} 
	else
	{
		bmp085_update(&(central_data->pressure));
	}

	return TASK_RUN_SUCCESS;
}


//task_return_t sonar_update(void* arg)
//{
	// TODO: add the simulation sonar task
	//central_data_t* central_data = central_data_get_pointer_to_struct();
	//i2cxl_sonar_update(&central_data->i2cxl_sonar);
	//
	//return TASK_RUN_SUCCESS;
//}

void tasks_create_tasks() 
{	
	central_data = central_data_get_pointer_to_struct();
	
	scheduler_t* scheduler = &central_data->scheduler;

	scheduler_add_task(scheduler    , 4000                            , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_HIGHEST, &tasks_run_stabilisation                                          , 0                                                    , 0);
	scheduler_add_task(scheduler    , 15000                           , RUN_REGULAR , PERIODIC_RELATIVE, PRIORITY_HIGH   , &tasks_run_barometer_update                                       , 0                                                    , 1);
	scheduler_add_task(scheduler    , 100000                          , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_HIGH   , &tasks_run_gps_update                                             , 0                                                    , 2);
	scheduler_add_task(scheduler    , ORCA_TIME_STEP_MILLIS * 1000.0f , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_HIGH   , (task_function_t)&navigation_update                               , (task_argument_t)&central_data->navigationData		 , 3);
	scheduler_add_task(scheduler    , 200000                          , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_NORMAL , &tasks_set_mav_mode_n_state                                       , 0                                                    , 4);
	scheduler_add_task(scheduler    , 4000                            , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&mavlink_communication_update                    , (task_argument_t)&central_data->mavlink_communication , 5);
	scheduler_add_task(scheduler    , 100000                          , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&analog_monitor_update                           , (task_argument_t)&central_data->adc                   , 6);
	scheduler_add_task(scheduler    , 10000                           , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&waypoint_handler_control_time_out_waypoint_msg  , (task_argument_t)&central_data->waypoint_handler                                                    , 7);
	// scheduler_add_task(scheduler , 100000                          , RUN_REGULAR , PERIODIC_ABSOLUTE, PRIORITY_NORMAL , &sonar_update                                                     , 0                                                    , 0);

	scheduler_sort_tasks(scheduler);
}