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
 * \file mavlink_telemetry.c
 *
 * Definition of the messages sent by the autopilot to the ground station
 */ 


#include "mavlink_telemetry.h"
#include "mavlink_actions.h"
#include "central_data.h"
#include "onboard_parameters.h"
#include "mavlink_stream.h"
#include "scheduler.h"
#include "radar_module_driver.h"
#include "remote_controller.h"
#include "analog_monitor.h"
#include "tasks.h"
#include "mavlink_waypoint_handler.h"
#include "neighbor_selection.h"
#include "analog_monitor.h"
#include "state.h"
#include "position_estimation.h"
#include "hud.h"

central_data_t *centralData;


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------


/**
 * \brief	Task to send the mavlink real time statistics message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_rt_stats(void* arg);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------


task_return_t mavlink_telemetry_send_rt_stats(void* arg) 
{
	task_set_t* main_tasks = tasks_get_main_taskset();
	
	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"stabAvgDelay", 
										main_tasks->tasks[0].delay_avg);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"stabDelayVar", 
										sqrt(main_tasks->tasks[0].delay_var_squared));

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"stabMaxDelay", 
										main_tasks->tasks[0].delay_max);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"stabRTvio", 
										main_tasks->tasks[0].rt_violations);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"baroAvgDelay", 
										main_tasks->tasks[1].delay_avg);


	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"imuExTime", 
										main_tasks->tasks[0].execution_time);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"navExTime", 
										main_tasks->tasks[3].execution_time);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										"imu_dt", 
										central_data_get_pointer_to_struct()->imu.dt);

	
	main_tasks->tasks[1].rt_violations = 0;
	main_tasks->tasks[1].delay_max = 0;

	return TASK_RUN_SUCCESS;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init(void)
{
	centralData = central_data_get_pointer_to_struct();

	scheduler_t* mavlink_scheduler = &centralData->mavlink_communication.scheduler; 

	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_heartbeat,								&centralData->state_structure, 					MAVLINK_MSG_ID_HEARTBEAT	);							// ID 0
	scheduler_add_task(mavlink_scheduler,  1000000,	 RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&state_send_status,									&centralData->state_structure,					MAVLINK_MSG_ID_SYS_STATUS	);							// ID 1
	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&gps_ublox_send_raw,								&centralData->GPS_data,							MAVLINK_MSG_ID_GPS_RAW_INT	);							// ID 24
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_scaled,									&centralData->imu, 								MAVLINK_MSG_ID_SCALED_IMU	);							// ID 26
	scheduler_add_task(mavlink_scheduler,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_raw,										&centralData->imu, 								MAVLINK_MSG_ID_RAW_IMU	);								// ID 27
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&bmp085_send_pressure,								&centralData->pressure,							MAVLINK_MSG_ID_SCALED_PRESSURE	);						// ID 29
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_attitude,									&centralData->attitude_estimation, 				MAVLINK_MSG_ID_ATTITUDE	);								// ID 30
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&imu_send_attitude_quaternion,						&centralData->attitude_estimation, 				MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);					// ID 31
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&position_estimation_send_position,					&centralData->position_estimator, 				MAVLINK_MSG_ID_LOCAL_POSITION_NED	);					// ID 32
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&position_estimation_send_global_position,			&centralData->position_estimator, 				MAVLINK_MSG_ID_GLOBAL_POSITION_INT	);					// ID 33
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_dsm2_send_scaled_rc_channels,				0, 												MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);					// ID 34
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&remote_dsm2_send_raw_rc_channels,					0, 												MAVLINK_MSG_ID_RC_CHANNELS_RAW	);						// ID 35
	scheduler_add_task(mavlink_scheduler,  1000000,  RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&servo_pwm_send_servo_output,						&centralData->servos, 							MAVLINK_MSG_ID_SERVO_OUTPUT_RAW	);						// ID 36
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&stabilisation_send_rpy_thrust_setpoint,			&centralData->controls, 						MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT	);		// ID 58
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&stabilisation_send_rpy_speed_thrust_setpoint,		&centralData->stabiliser_stack.rate_stabiliser,	MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);	// ID 59
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&hud_send_message,									&centralData->hud_structure, 					MAVLINK_MSG_ID_VFR_HUD	);								// ID 74
	scheduler_add_task(mavlink_scheduler,  200000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&stabilisation_send_rpy_rates_error,				&centralData->stabiliser_stack.rate_stabiliser,	MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT	);	// ID 80
	scheduler_add_task(mavlink_scheduler,  500000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&simulation_send_data,								&centralData->sim_model, 						MAVLINK_MSG_ID_HIL_STATE	);							// ID 90
	scheduler_add_task(mavlink_scheduler,  250000,   RUN_NEVER,    PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&mavlink_telemetry_send_rt_stats,					0, 												MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251
	//scheduler_add_task(mavlink_scheduler,  100000,   RUN_REGULAR,  PERIODIC_ABSOLUTE, PRIORITY_NORMAL, (task_function_t)&mavlink_telemetry_send_sonar,						&centralData->i2cxl_sonar, 						MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251

	scheduler_sort_tasks(mavlink_scheduler);
	
	print_util_dbg_print("MAVlink actions initialiased\n");
}