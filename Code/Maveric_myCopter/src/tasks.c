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
#include "lsm330dlc.h"
#include "hmc5883l.h"
#include "stdio_usb.h"
#include "data_logging.h"

#include "pwm_servos.h"


#include "remote_controller.h"
#include "remote.h"
#include "attitude_controller_p2.h"

central_data_t* central_data;


task_set_t* tasks_get_main_taskset() 
{
	central_data = central_data_get_pointer_to_struct();

	return central_data->scheduler.task_set;
}

void tasks_run_imu_update(void* arg)
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_update(&central_data->sim_model);
	} 
	else 
	{
		lsm330dlc_gyro_update(&(central_data->imu.raw_gyro));
		lsm330dlc_acc_update(&(central_data->imu.raw_accelero));
		hmc5883l_update(&(central_data->imu.raw_compass));
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
	
	mav_mode_t mode = central_data->state.mav_mode;

	if( mode.ARMED == ARMED_ON )
	{
		if ( mode.AUTO == AUTO_ON )
		{
			central_data->controls = central_data->controls_nav;
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
			
			// if no waypoints are set, we do position hold therefore the yaw mode is absolute
			if (((central_data->state.nav_plan_active&&(central_data->state.mav_state != MAV_STATE_STANDBY)))||((central_data->state.mav_state == MAV_STATE_CRITICAL)&&(central_data->waypoint_handler.critical_behavior == FLY_TO_HOME_WP)))
			{
				central_data->controls.yaw_mode = YAW_COORDINATED;
			}
			else
			{
				central_data->controls.yaw_mode = YAW_ABSOLUTE;
			}
		
			if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
			{
				stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
			}
		}
		else if ( mode.GUIDED == GUIDED_ON )
		{
			central_data->controls = central_data->controls_nav;
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
		
			if ((central_data->state.mav_state == MAV_STATE_CRITICAL) && (central_data->waypoint_handler.critical_behavior == FLY_TO_HOME_WP))
			{
				central_data->controls.yaw_mode = YAW_COORDINATED;
			}
			else
			{
				central_data->controls.yaw_mode = YAW_ABSOLUTE;
			}
		
			if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
			{
				stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
			}
		}
		else if ( mode.STABILISE == STABILISE_ON )
		{
			if (central_data->state.remote_active)
			{
				remote_controller_get_velocity_vector_from_remote(&central_data->controls);
			}
			else
			{
				joystick_parsing_get_velocity_vector_from_joystick(&central_data->joystick_parsing,&central_data->controls);
			}
			
			
			central_data->controls.control_mode = VELOCITY_COMMAND_MODE;
			central_data->controls.yaw_mode = YAW_RELATIVE;
			
			if (central_data->state.in_the_air || central_data->navigation.auto_takeoff)
			{
				stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);
			}		
		}
		else if ( mode.MANUAL == MANUAL_ON )
		{
			if (central_data->state.remote_active)
			{
				remote_controller_get_command_from_remote(&central_data->controls);
			}
			else
			{
				joystick_parsing_get_attitude_command_from_joystick(&central_data->joystick_parsing,&central_data->controls);
			}
			
			central_data->controls.control_mode = ATTITUDE_COMMAND_MODE;
			central_data->controls.yaw_mode=YAW_RELATIVE;
		
			stabilisation_copter_cascade_stabilise(&central_data->stabilisation_copter);		
		}
		else
		{
			servos_set_value_failsafe( &central_data->servos );
		}
	}
	else
	{
		servos_set_value_failsafe( &central_data->servos );
	}

		
	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if ( mode.HIL == HIL_OFF )
	{
		pwm_servos_write_to_hardware( &central_data->servos );
	}
	
	return TASK_RUN_SUCCESS;
}


// new task to test P^2 attutude controller
task_return_t tasks_run_stabilisation_quaternion(void* arg);
task_return_t tasks_run_stabilisation_quaternion(void* arg)
{
	tasks_run_imu_update(0);
	
	mav_mode_t mode = central_data->state.mav_mode;

	if( mode.MANUAL == MANUAL_ON && mode.STABILISE == STABILISE_ON )
	{
		remote_controller_get_command_from_remote(&central_data->controls);
		
		central_data->command.attitude.rpy[0] 	= 2 * central_data->controls.rpy[0];
		central_data->command.attitude.rpy[1] 	= 2 * central_data->controls.rpy[1];
		central_data->command.attitude.rpy[2] 	= 2 * central_data->controls.rpy[2];
		central_data->command.attitude.mode 	= ATTITUDE_COMMAND_MODE_RPY;
		central_data->command.thrust.thrust 	= central_data->controls.thrust;
	
		attitude_controller_p2_update( &central_data->attitude_controller );			
		servos_mix_quadcopter_diag_update( &central_data->servo_mix );
	}
	else
	{
		servos_set_value_failsafe( &central_data->servos );
	}

	// !!! -- for safety, this should remain the only place where values are written to the servo outputs! --- !!!
	if ( mode.ARMED == ARMED_ON && mode.HIL == HIL_OFF )
	{
		pwm_servos_write_to_hardware( &central_data->servos );
	}
	
	return TASK_RUN_SUCCESS;
} 



task_return_t tasks_run_gps_update(void* arg) 
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_simulate_gps(&central_data->sim_model);
	} 
	else 
	{
		gps_ublox_update(&central_data->gps);
	}
	
	return TASK_RUN_SUCCESS;
}


task_return_t tasks_run_barometer_update(void* arg)
{
	if (central_data->state.mav_mode.HIL == HIL_ON)
	{
		simulation_simulate_barometer(&central_data->sim_model);
	} 
	else
	{
		bmp085_update(&(central_data->pressure));
	}

	return TASK_RUN_SUCCESS;
}

task_return_t tasks_led_toggle(void* arg)
{
	LED_Toggle(LED1);
	
	return TASK_RUN_SUCCESS;
}


void tasks_create_tasks() 
{	
	central_data = central_data_get_pointer_to_struct();
	
	scheduler_t* scheduler = &central_data->scheduler;

	scheduler_add_task(scheduler, 4000,	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGHEST, &tasks_run_stabilisation                                          , 0															, 0);
	// scheduler_add_task(scheduler, 4000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGHEST, &tasks_run_stabilisation_quaternion                               , 0 													, 0);

	scheduler_add_task(scheduler, 20000, 	RUN_REGULAR, PERIODIC_RELATIVE, PRIORITY_HIGH   , (task_function_t)&remote_update 									, (task_argument_t)&central_data->remote 				, 1);
	
	scheduler_add_task(scheduler, 15000, 	RUN_REGULAR, PERIODIC_RELATIVE, PRIORITY_HIGH   , &tasks_run_barometer_update                                       , 0 													, 2);
	scheduler_add_task(scheduler, 100000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGH   , &tasks_run_gps_update                                             , 0 													, 3);
	scheduler_add_task(scheduler, 10000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_HIGH   , (task_function_t)&navigation_update                               , (task_argument_t)&central_data->navigation 			, 4);
	
	scheduler_add_task(scheduler, 200000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&state_machine_update              				, (task_argument_t)&central_data->state_machine         , 5);

	scheduler_add_task(scheduler, 4000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_NORMAL , (task_function_t)&mavlink_communication_update                    , (task_argument_t)&central_data->mavlink_communication , 6);
	scheduler_add_task(scheduler, 100000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&analog_monitor_update                           , (task_argument_t)&central_data->analog_monitor 		, 7);
	scheduler_add_task(scheduler, 10000, 	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW    , (task_function_t)&waypoint_handler_control_time_out_waypoint_msg  , (task_argument_t)&central_data->waypoint_handler 		, 8);
	
	scheduler_add_task(scheduler, 100000,   RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOW	, (task_function_t)&data_logging_run								, (task_argument_t)&central_data->data_logging			, 9);
	
	scheduler_add_task(scheduler, 500000,	RUN_REGULAR, PERIODIC_ABSOLUTE, PRIORITY_LOWEST , &tasks_led_toggle													, 0														, 10);

	scheduler_sort_tasks(scheduler);
}
