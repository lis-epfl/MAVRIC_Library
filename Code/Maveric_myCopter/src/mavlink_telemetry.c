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

central_data_t *centralData;


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS DECLARATION
//------------------------------------------------------------------------------

/**
 * \brief	Task to send the mavlink system status message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_status(analog_monitor_t* adc);

/**
 * \brief	Task to send the mavlink scaled pressure message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_pressure(pressure_data_t* pressure);

/**
 * \brief	Task to send the mavlink GPS global position message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_global_position(void* arg);


/**
 * \brief	Task to send the mavlink RC scaled message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_scaled_rc_channels(void* arg);


/**
 * \brief	Task to send the mavlink RC raw message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_raw_rc_channels(void* arg);


/**
 * \brief	Task to send the mavlink servo output message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_servo_output(void* arg);


/**
 * \brief	Task to send the mavlink roll, pitch, yaw and thrust setpoints message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_rpy_thrust_setpoint(Control_Command_t* controls);


/**
 * \brief	Task to send the mavlink roll, pitch, yaw angular speeds and thrust setpoints message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_rpy_speed_thrust_setpoint(Stabiliser_t* rate_stabiliser);


/**
 * \brief	Task to send the mavlink HUD message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_hud(void* arg);


/**
 * \brief	Task to send the mavlink roll, pitch and yaw errors message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_rpy_rates_error(Stabiliser_t* rate_stabiliser);

/**
 * \brief	Task to send the mavlink real time statistics message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_rt_stats(void* arg);


/**
 * \brief	Task to send the mavlink sonar message
 * 
 * \return	The status of execution of the task
 */
task_return_t mavlink_telemetry_send_sonar(i2cxl_sonar_t* i2cxl_sonar);


//------------------------------------------------------------------------------
// PRIVATE FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

task_return_t mavlink_telemetry_send_status(analog_monitor_t* adc)
{
	float battery_voltage = adc->avg[ANALOG_RAIL_10];		// bat voltage (mV), actual battery pack plugged to the board
	float battery_remaining = adc->avg[ANALOG_RAIL_11] / 12.4f * 100.0f;
	
	mavlink_msg_sys_status_send(MAVLINK_COMM_0, 
								0b1111110000100111, 									// sensors present
								0b1111110000100111, 									// sensors enabled
								0b1111110000100111, 									// sensors health
								0,                  									// load
								(int32_t)(1000.0f * battery_voltage), 						// bat voltage (mV)
								0,               										// current (mA)
								battery_remaining,										// battery remaining
								0, 0,  													// comms drop, comms errors
								0, 0, 0, 0);        									// autopilot specific errors
	return TASK_RUN_SUCCESS;
}

task_return_t mavlink_telemetry_send_pressure(pressure_data_t* pressure) 
{	
	mavlink_msg_scaled_pressure_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(), 
										pressure->pressure / 100.0f, 
										pressure->vario_vz, 
										pressure->temperature * 100.0f);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										time_keeper_get_millis(),
										"pressAlt", 
										pressure->altitude);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										time_keeper_get_millis(),
										"lastAlt", 
										pressure->last_altitudes[0]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										time_keeper_get_millis(),
										"baro_dt", 
										pressure->dt);
	return TASK_RUN_SUCCESS;
}

task_return_t mavlink_telemetry_send_global_position(void* arg) 
{				
	// send integrated position (for now there is no GPS error correction...!!!)
	global_position_t gpos = coord_conventions_local_to_global_position(centralData->position_estimator.localPosition);
	
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	mavlink_msg_global_position_int_send(	MAVLINK_COMM_0, 
											time_keeper_get_millis(), 
											gpos.latitude * 10000000, 
											gpos.longitude * 10000000, 
											gpos.altitude * 1000.0f, 
											1, 
											centralData->position_estimator.vel[0] * 100.0f, 
											centralData->position_estimator.vel[1] * 100.0f, 
											centralData->position_estimator.vel[2] * 100.0f, 
											centralData->imu.scaled_gyro.data[2]);
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_scaled_rc_channels(void* arg)
{
	mavlink_msg_rc_channels_scaled_send(	MAVLINK_COMM_0,time_keeper_get_millis(),
											1,
											remote_dsm2_rc_get_channel(0) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(1) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(2) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(3) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(4) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(5) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(6) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_get_channel(7) * 1000.0f * RC_SCALEFACTOR,
											remote_dsm2_rc_check_receivers()	);
	
	mavlink_msg_named_value_int_send(	MAVLINK_COMM_0,
										time_keeper_get_millis(),
										"Coll_Avoidance",
										centralData->waypoint_handler.collision_avoidance	);			// TODO: remove this !
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_raw_rc_channels(void* arg)
{
	mavlink_msg_rc_channels_raw_send(	MAVLINK_COMM_0,time_keeper_get_millis(),
										1,
										remote_dsm2_rc_get_channel(0) + 1000,
										remote_dsm2_rc_get_channel(1) + 1000,
										remote_dsm2_rc_get_channel(2) + 1000,
										remote_dsm2_rc_get_channel(3) + 1000,
										remote_dsm2_rc_get_channel(4) + 1000,
										remote_dsm2_rc_get_channel(5) + 1000,
										remote_dsm2_rc_get_channel(6) + 1000,
										remote_dsm2_rc_get_channel(7) + 1000,
										remote_dsm2_rc_check_receivers()	);
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_servo_output(void* arg) 
{
	mavlink_msg_servo_output_raw_send(	MAVLINK_COMM_0, 
										time_keeper_get_micros(), 
										0, 
										(uint16_t)(centralData->servos[0].value + 1500),
										(uint16_t)(centralData->servos[1].value + 1500),
										(uint16_t)(centralData->servos[2].value + 1500),
										(uint16_t)(centralData->servos[3].value + 1500),
										(uint16_t)(centralData->servos[4].value + 1500),
										(uint16_t)(centralData->servos[5].value + 1500),
										(uint16_t)(centralData->servos[6].value + 1500),
										(uint16_t)(centralData->servos[7].value + 1500)	);
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_rpy_thrust_setpoint(Control_Command_t* controls) 
{	
	// Controls output
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(	MAVLINK_COMM_0, 
														time_keeper_get_millis(), 
														controls->rpy[ROLL], 
														controls->rpy[PITCH], 
														controls->rpy[YAW], 
														controls->thrust);
	return TASK_RUN_SUCCESS;
}


task_return_t  mavlink_telemetry_send_rpy_speed_thrust_setpoint(Stabiliser_t* rate_stabiliser)
{
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(	MAVLINK_COMM_0,
															time_keeper_get_millis(),
															rate_stabiliser->rpy_controller[0].output,
															rate_stabiliser->rpy_controller[1].output,
															rate_stabiliser->rpy_controller[2].output,
															0 );
	return TASK_RUN_SUCCESS;
}


task_return_t mavlink_telemetry_send_hud(void* arg) 
{
	float groundspeed = sqrt(centralData->position_estimator.vel[0] * centralData->position_estimator.vel[0] + centralData->position_estimator.vel[1] * centralData->position_estimator.vel[1]);
	float airspeed=groundspeed;

	Aero_Attitude_t aero_attitude;
	aero_attitude = coord_conventions_quat_to_aero(centralData->attitude_estimation.qe);
	
	int16_t heading;
	if(aero_attitude.rpy[2] < 0)
	{
		heading = (int16_t)(360.0f - 180.0f * aero_attitude.rpy[2] / PI);
	}
	else
	{
		heading = (int16_t)(180.0f * aero_attitude.rpy[2] / PI);
	}
	
	// mavlink_msg_vfr_hud_send(mavlink_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
	mavlink_msg_vfr_hud_send(	MAVLINK_COMM_0, 
								airspeed, 
								groundspeed, 
								heading, 
								(int32_t)((centralData->controls.thrust + 1.0f) * 50), 
								-centralData->position_estimator.localPosition.pos[2] + centralData->position_estimator.localPosition.origin.altitude, 
								-centralData->position_estimator.vel[2]	);
	return TASK_RUN_SUCCESS;
}


task_return_t  mavlink_telemetry_send_rpy_rates_error(Stabiliser_t* rate_stabiliser) 
{
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(	MAVLINK_COMM_0, 
															time_keeper_get_millis(), 
															rate_stabiliser->rpy_controller[0].error, 
															rate_stabiliser->rpy_controller[1].error,
															rate_stabiliser->rpy_controller[2].error,
															0 );
	return TASK_RUN_SUCCESS;
}

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


task_return_t mavlink_telemetry_send_sonar(i2cxl_sonar_t* i2cxl_sonar)
{
	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										time_keeper_get_millis(),
										"sonar(m)", 
										i2cxl_sonar->distance_m);
	return TASK_RUN_SUCCESS;
}


//------------------------------------------------------------------------------
// PUBLIC FUNCTIONS IMPLEMENTATION
//------------------------------------------------------------------------------

void mavlink_telemetry_init(void)
{
	centralData = central_data_get_pointer_to_struct();

	scheduler_add_task(&centralData->mavlink_communication.task_set,  1000000,  RUN_REGULAR,  (task_function_t)&state_send_heartbeat,								&centralData->state_structure, 					MAVLINK_MSG_ID_HEARTBEAT	);							// ID 0
	scheduler_add_task(&centralData->mavlink_communication.task_set,  1000000,	RUN_REGULAR,  (task_function_t)&mavlink_telemetry_send_status,						&centralData->adc,								MAVLINK_MSG_ID_SYS_STATUS	);							// ID 1
	scheduler_add_task(&centralData->mavlink_communication.task_set,  1000000,  RUN_NEVER,    (task_function_t)&gps_ublox_send_raw,									&centralData->GPS_data,							MAVLINK_MSG_ID_GPS_RAW_INT	);							// ID 24
	scheduler_add_task(&centralData->mavlink_communication.task_set,  250000,   RUN_REGULAR,  (task_function_t)&imu_send_scaled,									&centralData->imu, 								MAVLINK_MSG_ID_SCALED_IMU	);							// ID 26
	scheduler_add_task(&centralData->mavlink_communication.task_set,  100000,   RUN_REGULAR,  (task_function_t)&imu_send_raw,										&centralData->imu, 								MAVLINK_MSG_ID_RAW_IMU	);								// ID 27
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_pressure,					&centralData->pressure,							MAVLINK_MSG_ID_SCALED_PRESSURE	);						// ID 29
	scheduler_add_task(&centralData->mavlink_communication.task_set,  200000,   RUN_REGULAR,  (task_function_t)&imu_send_attitude,									&centralData->attitude_estimation, 				MAVLINK_MSG_ID_ATTITUDE	);								// ID 30
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_REGULAR,  (task_function_t)&imu_send_attitude_quaternion,						&centralData->attitude_estimation, 				MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);					// ID 31
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_NEVER,    (task_function_t)&postition_estimation_send_position,					&centralData->position_estimator, 				MAVLINK_MSG_ID_LOCAL_POSITION_NED	);					// ID 32
	scheduler_add_task(&centralData->mavlink_communication.task_set,  250000,   RUN_REGULAR,  (task_function_t)&mavlink_telemetry_send_global_position,				0, 												MAVLINK_MSG_ID_GLOBAL_POSITION_INT	);					// ID 33
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_scaled_rc_channels,			0, 												MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);					// ID 34
	scheduler_add_task(&centralData->mavlink_communication.task_set,  250000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_raw_rc_channels,				0, 												MAVLINK_MSG_ID_RC_CHANNELS_RAW	);						// ID 35
	scheduler_add_task(&centralData->mavlink_communication.task_set,  1000000,  RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_servo_output,				0, 												MAVLINK_MSG_ID_SERVO_OUTPUT_RAW	);						// ID 36
	scheduler_add_task(&centralData->mavlink_communication.task_set,  200000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_rpy_thrust_setpoint,			&centralData->controls, 						MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT	);		// ID 58
	scheduler_add_task(&centralData->mavlink_communication.task_set,  200000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_rpy_speed_thrust_setpoint,	&centralData->stabiliser_stack.rate_stabiliser,	MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);	// ID 59
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_REGULAR,  (task_function_t)&mavlink_telemetry_send_hud,							0, 												MAVLINK_MSG_ID_VFR_HUD	);								// ID 74
	scheduler_add_task(&centralData->mavlink_communication.task_set,  200000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_rpy_rates_error,				&centralData->stabiliser_stack.rate_stabiliser,	MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT	);	// ID 80
	scheduler_add_task(&centralData->mavlink_communication.task_set,  500000,   RUN_REGULAR,  (task_function_t)&simulation_send_data,								&centralData->sim_model, 						MAVLINK_MSG_ID_HIL_STATE	);							// ID 90
	scheduler_add_task(&centralData->mavlink_communication.task_set,  250000,   RUN_NEVER,    (task_function_t)&mavlink_telemetry_send_rt_stats,					0, 												MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251
	//scheduler_add_task(&centralData->mavlink_communication.task_set,  100000,   RUN_REGULAR,  (task_function_t)&mavlink_telemetry_send_sonar,&centralData->i2cxl_sonar, 	MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);					// ID 251

	scheduler_sort_taskset_by_period(&centralData->mavlink_communication.task_set);
	
	print_util_dbg_print("MAVlink actions initialiased\n");
}