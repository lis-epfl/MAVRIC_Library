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
 * \file mavlink_action.c
 *
 * Definition of the tasks executed on the autopilot
 */ 


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


central_data_t *centralData;


void mavlink_send_heartbeat(void) 
{
	central_data_t *centralData = central_data_get_pointer_to_struct();

	float battery_voltage = centralData->adc.avg[ANALOG_RAIL_10];		// bat voltage (mV), actual battery pack plugged to the board
	float battery_remaining = centralData->adc.avg[ANALOG_RAIL_11] / 12.4 * 100.0;

	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, centralData->mav_mode, 0, centralData->mav_state);

	mavlink_msg_sys_status_send(MAVLINK_COMM_0, 
								0b1111110000100111, 									// sensors present
								0b1111110000100111, 									// sensors enabled
								0b1111110000100111, 									// sensors health
								0,                  									// load
								(int)(1000.0 * battery_voltage), 						// bat voltage (mV)
								0,               										// current (mA)
								battery_remaining,										// battery remaining
								0, 0,  													// comms drop, comms errors
								0, 0, 0, 0);        									// autopilot specific errors
}


void mavlink_send_raw_imu(void) 
{
	mavlink_msg_raw_imu_send(	MAVLINK_COMM_0, 
								get_micros(), 
								centralData->imu1.raw_channels[ACC_OFFSET+IMU_X], 
								centralData->imu1.raw_channels[ACC_OFFSET+IMU_Y], 
								centralData->imu1.raw_channels[ACC_OFFSET+IMU_Z], 
								centralData->imu1.raw_channels[GYRO_OFFSET+IMU_X], 
								centralData->imu1.raw_channels[GYRO_OFFSET+IMU_Y], 
								centralData->imu1.raw_channels[GYRO_OFFSET+IMU_Z], 
								centralData->imu1.raw_channels[COMPASS_OFFSET+IMU_X], 
								centralData->imu1.raw_channels[COMPASS_OFFSET+IMU_Y], 
								centralData->imu1.raw_channels[COMPASS_OFFSET+IMU_Z] );
}


void mavlink_send_scaled_imu(void) 
{
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, 
								get_millis(),
								1000*centralData->imu1.attitude.a [IMU_X],
								1000*centralData->imu1.attitude.a [IMU_Y], 
								1000*centralData->imu1.attitude.a [IMU_Z], 
								1000*centralData->imu1.attitude.om[IMU_X], 
								1000*centralData->imu1.attitude.om[IMU_Y], 
								1000*centralData->imu1.attitude.om[IMU_Z], 
								1000*centralData->imu1.attitude.mag[IMU_X],
								1000*centralData->imu1.attitude.mag[IMU_Y],
								1000*centralData->imu1.attitude.mag[IMU_Z]
	);
	
}


void  mavlink_send_rpy_rates_error(void) 
{
	Stabiliser_t *rate_stab = &centralData->stabiliser_stack.rate_stabiliser;

	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(	MAVLINK_COMM_0, 
															get_millis(), 
															rate_stab->rpy_controller[0].error, 
															rate_stab->rpy_controller[1].error,
															rate_stab->rpy_controller[2].error,
															0 );
}


void  mavlink_send_rpy_speed_thrust_setpoint(void) 
{
	Stabiliser_t *rate_stab = &centralData->stabiliser_stack.rate_stabiliser;

	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(	MAVLINK_COMM_0, 
															get_millis(), 
															rate_stab->rpy_controller[0].output, 
															rate_stab->rpy_controller[1].output,
															rate_stab->rpy_controller[2].output,
															0 );
}


void mavlink_send_rpy_thrust_setpoint(void) 
{	
	// Controls output
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(	MAVLINK_COMM_0, 
														get_millis(), 
														centralData->controls.rpy[ROLL], 
														centralData->controls.rpy[PITCH], 
														centralData->controls.rpy[YAW], 
														centralData->controls.thrust);
}


void mavlink_send_servo_output(void) 
{
	Stabiliser_t *rate_stab = &centralData->stabiliser_stack.rate_stabiliser;

	// FIXME: send only servos values in this message!
	mavlink_msg_servo_output_raw_send(	MAVLINK_COMM_0, 
										get_micros(), 
										0, 
										(uint16_t)(centralData->servos[0].value+1500),
										(uint16_t)(centralData->servos[1].value+1500),
										(uint16_t)(centralData->servos[2].value+1500),
										(uint16_t)(centralData->servos[3].value+1500),
										(uint16_t)(centralData->servos[4].value+1500),
										(uint16_t)(centralData->servos[5].value+1500),
										(uint16_t)(centralData->servos[6].value+1500),
										(uint16_t)(centralData->servos[7].value+1500)	);
}


void mavlink_send_attitude_quaternion(void) 
{
	// ATTITUDE QUATERNION
	mavlink_msg_attitude_quaternion_send(	MAVLINK_COMM_0, 
											get_millis(), 
											centralData->imu1.attitude.qe.s, 
											centralData->imu1.attitude.qe.v[0], 
											centralData->imu1.attitude.qe.v[1], 
											centralData->imu1.attitude.qe.v[2], 
											centralData->imu1.attitude.om[0], 
											centralData->imu1.attitude.om[1], 
											centralData->imu1.attitude.om[2]	);
}


void mavlink_send_attitude(void) 
{
	// ATTITUDE
	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);

	mavlink_msg_attitude_send(	MAVLINK_COMM_0, 
								get_millis(), 
								aero_attitude.rpy[0], 
								aero_attitude.rpy[1], 
								aero_attitude.rpy[2], 
								centralData->imu1.attitude.om[0], 
								centralData->imu1.attitude.om[1], 
								centralData->imu1.attitude.om[2]);
}


void mavlink_send_global_position(void) 
{				
	// send integrated position (for now there is no GPS error correction...!!!)
	global_position_t gpos = local_to_global_position(centralData->position_estimator.localPosition);
	
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	mavlink_msg_global_position_int_send(	MAVLINK_COMM_0, 
											get_millis(), 
											gpos.latitude * 10000000, 
											gpos.longitude * 10000000, 
											gpos.altitude * 1000.0, 
											1, 
											centralData->position_estimator.vel[0] * 100.0, 
											centralData->position_estimator.vel[1] * 100.0, 
											centralData->position_estimator.vel[2] * 100.0, 
											centralData->imu1.attitude.om[2]);
}


void mavlink_send_hud(void) 
{
	float groundspeed = sqrt(centralData->position_estimator.vel[0] * centralData->position_estimator.vel[0] + centralData->position_estimator.vel[1] * centralData->position_estimator.vel[1]);
	float airspeed=groundspeed;

	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(centralData->imu1.attitude.qe);
	
	int16_t heading;
	if(aero_attitude.rpy[2] < 0)
	{
		heading = (int16_t)(360.0 - 180.0 * aero_attitude.rpy[2] / PI);
	}
	else
	{
		heading = (int16_t)(180.0 * aero_attitude.rpy[2] / PI);
	}
	
	// mavlink_msg_vfr_hud_send(mavlink_channel_t chan, float airspeed, float groundspeed, int16_t heading, uint16_t throttle, float alt, float climb)
	mavlink_msg_vfr_hud_send(	MAVLINK_COMM_0, 
								airspeed, 
								groundspeed, 
								heading, 
								(int)((centralData->controls.thrust+1.0)*50), 
								-centralData->position_estimator.localPosition.pos[2] + centralData->position_estimator.localPosition.origin.altitude, 
								-centralData->position_estimator.vel[2]	);
}

void mavlink_send_gps_raw(void) 
{	
	if (centralData->GPS_data.status == GPS_OK)
	{
		mavlink_msg_gps_raw_int_send(	MAVLINK_COMM_0,
										1000 * centralData->GPS_data.timeLastMsg, 
										centralData->GPS_data.status, 
										centralData->GPS_data.latitude * 10000000.0, 
										centralData->GPS_data.longitude * 10000000.0, 
										centralData->GPS_data.altitude * 1000.0, 
										centralData->GPS_data.hdop * 100.0, 
										centralData->GPS_data.speedAccuracy * 100.0,
										centralData->GPS_data.groundSpeed * 100.0, 
										centralData->GPS_data.course, 
										centralData->GPS_data.num_sats	);	
	}
	else
	{
		mavlink_msg_gps_raw_int_send(	MAVLINK_COMM_0,
										get_micros(), 
										centralData->GPS_data.status, 
										46.5193 * 10000000, 
										6.56507 * 10000000, 
										400 * 1000, 
										0, 
										0, 
										0, 
										0, 
										centralData->GPS_data.num_sats);
	}
}


void mavlink_send_pressure(void) 
{	
	mavlink_msg_scaled_pressure_send(	MAVLINK_COMM_0, 
										get_millis(), 
										centralData->pressure.pressure / 100.0, 
										centralData->pressure.vario_vz, 
										centralData->pressure.temperature * 100.0);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"pressAlt", 
										centralData->pressure.altitude);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"lastAlt", 
										centralData->position_estimator.last_alt);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"baro_dt", 
										centralData->pressure.dt);
}


void mavlink_send_estimator(void)
{
	mavlink_msg_local_position_ned_send(	MAVLINK_COMM_0, 
											get_millis(), 
											centralData->position_estimator.localPosition.pos[0], 
											centralData->position_estimator.localPosition.pos[1], 
											centralData->position_estimator.localPosition.pos[2], 
											centralData->position_estimator.vel[0], 
											centralData->position_estimator.vel[1], 
											centralData->position_estimator.vel[2]);
}


void mavlink_send_raw_rc_channels(void)
{
	mavlink_msg_rc_channels_raw_send(	MAVLINK_COMM_0,get_millis(),
										1,
										rc_get_channel(0) + 1000,
										rc_get_channel(1) + 1000,
										rc_get_channel(2) + 1000,
										rc_get_channel(3) + 1000,
										rc_get_channel(4) + 1000,
										rc_get_channel(5) + 1000,
										rc_get_channel(6) + 1000,
										rc_get_channel(7) + 1000,
										rc_check_receivers()	);
}


void mavlink_send_scaled_rc_channels(void)
{
	mavlink_msg_rc_channels_scaled_send(	MAVLINK_COMM_0,get_millis(),
											1,
											rc_get_channel(0) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(1) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(2) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(3) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(4) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(5) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(6) * 1000.0 * RC_SCALEFACTOR,
											rc_get_channel(7) * 1000.0 * RC_SCALEFACTOR,
											rc_check_receivers()	);
	
	mavlink_msg_named_value_int_send(	MAVLINK_COMM_0,
										get_millis(),
										"Coll_Avoidance",
										centralData->collision_avoidance	);
}


void mavlink_send_simulation(void) 
{
	Aero_Attitude_t aero_attitude;
	aero_attitude = Quat_to_Aero(centralData->sim_model.attitude.qe);

	global_position_t gpos = local_to_global_position(centralData->sim_model.localPosition);
	
	mavlink_msg_hil_state_send(	MAVLINK_COMM_0, 
								get_micros(),
								aero_attitude.rpy[0], 
								aero_attitude.rpy[1], 
								aero_attitude.rpy[2],
								centralData->sim_model.rates_bf[ROLL], 
								centralData->sim_model.rates_bf[PITCH], 
								centralData->sim_model.rates_bf[YAW],
								gpos.latitude * 10000000, 
								gpos.longitude * 10000000, 
								gpos.altitude * 1000.0,
								100 * centralData->sim_model.vel[X], 
								100 * centralData->sim_model.vel[Y], 
								100 * centralData->sim_model.vel[Z],
								1000 * centralData->sim_model.lin_forces_bf[0], 
								1000 * centralData->sim_model.lin_forces_bf[1], 
								1000 * centralData->sim_model.lin_forces_bf[2] 	);
	
	mavlink_msg_hil_state_quaternion_send(	MAVLINK_COMM_0,
											get_micros(),
											&centralData->sim_model.attitude.qe,
											aero_attitude.rpy[ROLL],
											aero_attitude.rpy[PITCH],
											aero_attitude.rpy[YAW],
											gpos.latitude * 10000000, 
											gpos.longitude * 10000000, 
											gpos.altitude * 1000.0,
											100 * centralData->sim_model.vel[X], 
											100 * centralData->sim_model.vel[Y], 
											100 * centralData->sim_model.vel[Z],
											100 * vector_norm(centralData->sim_model.vel),
											0.0,
											centralData->sim_model.attitude.acc_bf[X],
											centralData->sim_model.attitude.acc_bf[Y],
											centralData->sim_model.attitude.acc_bf[Z]	);
	
	mavlink_msg_named_value_int_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"rolltorque", 
										centralData->sim_model.torques_bf[0]	);

	mavlink_msg_named_value_int_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"pitchtorque", 
										centralData->sim_model.torques_bf[1]	);

	mavlink_msg_named_value_int_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"yawtorque", 
										centralData->sim_model.torques_bf[2]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"thrust", 
										centralData->sim_model.lin_forces_bf[2]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"rpm1",
										centralData->sim_model.rotorspeeds[0]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"rpm2",
										centralData->sim_model.rotorspeeds[1]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"rpm3",
										centralData->sim_model.rotorspeeds[2]);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0,
										get_millis(),
										"rpm4",
										centralData->sim_model.rotorspeeds[3]);
}


task_return_t mavlink_send_rt_stats() 
{
	task_set* main_tasks = tasks_get_main_taskset();
	
	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"stabAvgDelay", 
										main_tasks->tasks[0].delay_avg);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"stabDelayVar", 
										sqrt(main_tasks->tasks[0].delay_var_squared));

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"stabMaxDelay", 
										main_tasks->tasks[0].delay_max);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"stabRTvio", 
										main_tasks->tasks[0].rt_violations);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"baroAvgDelay", 
										main_tasks->tasks[1].delay_avg);


	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"imuExTime", 
										main_tasks->tasks[0].execution_time);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"navExTime", 
										main_tasks->tasks[3].execution_time);

	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(), 
										"imu_dt", 
										central_data_get_pointer_to_struct()->imu1.dt);

	
	main_tasks->tasks[1].rt_violations=0;
	main_tasks->tasks[1].delay_max=0;

}


void mavlink_actions_add_onboard_parameters(void) {
	Stabiliser_t* rate_stabiliser = &centralData->stabiliser_stack.rate_stabiliser;
	Stabiliser_t* attitude_stabiliser = &centralData->stabiliser_stack.attitude_stabiliser;
	Stabiliser_t* velocity_stabiliser= &centralData->stabiliser_stack.velocity_stabiliser;

	// Comp and sys ID
	// add_parameter_uint8(&(mavlink_system.sysid),"ID_System");
	// add_parameter_uint8(&(mavlink_mission_planner.sysid),"ID_Planner");
	
	// Simulation mode
	add_parameter_int32(&centralData->simulation_mode, "Sim_mode");
	
	// Roll rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain, "RollRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.clip, "RollRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollRPid_D_LPF");
	
	// Roll attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain, "RollAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.clip, "RollAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollAPid_D_LPF");

	// Pitch rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain, "PitchRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.clip, "PitchRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.clip, "PitchRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchRPid_D_LPF");
	
	// Pitch attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain, "PitchAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.clip, "PitchAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.clip, "PitchAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchAPid_D_LPF");

	// Yaw rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].p_gain, "YawRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_max, "YawRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].clip_min, "YawRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.clip, "YawRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.postgain, "YawRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].integrator.pregain, "YawRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.clip, "YawRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.gain, "YawRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawRPid_D_LPF");
	
	// Yaw attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].p_gain, "YawAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_max, "YawAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].clip_min, "YawAPid_P_CLmn");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.clip, "YawAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.postgain, "YawAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].integrator.pregain, "YawAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.clip, "YawAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain, "YawAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.LPF, "YawAPid_D_LPF");


	// Roll velocity PID
	add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].p_gain, "RollVPid_P_G");
	add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollVPid_I_PstG");
	add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollVPid_I_PreG");
	add_parameter_float(&velocity_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollVPid_D_Gain");

	// Pitch velocity PID
	add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].p_gain, "PitchVPid_P_G");
	add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchVPid_I_PstG");
	add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchVPid_I_PreG");
	add_parameter_float(&velocity_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchVPid_D_Gain");

	// Thrust velocity PID
	add_parameter_float(&velocity_stabiliser->thrust_controller.p_gain, "ThrVPid_P_G");
	add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.postgain, "ThrVPid_I_PstG");
	add_parameter_float(&velocity_stabiliser->thrust_controller.integrator.pregain, "ThrVPid_I_PreG");
	add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.gain, "ThrVPid_D_Gain");
	add_parameter_float(&velocity_stabiliser->thrust_controller.differentiator.LPF, "ThrVPid_D_LPF");
	add_parameter_float(&velocity_stabiliser->thrust_controller.soft_zone_width, "ThrVPid_soft");

	// qfilter
	add_parameter_float(&centralData->imu1.attitude.kp, "QF_kp_acc");
	add_parameter_float(&centralData->imu1.attitude.kp_mag, "QF_kp_mag");
	add_parameter_float(&attitude_stabiliser->rpy_controller[YAW].differentiator.gain, "YawAPid_D_Gain");
	
	// Biaises
	add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET+X],"Bias_Gyro_X");
	add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET+Y],"Bias_Gyro_Y");
	add_parameter_float(&centralData->imu1.attitude.be[GYRO_OFFSET+Z],"Bias_Gyro_Z");
	
	add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET+X],"Bias_Acc_X");
	add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET+Y],"Bias_Acc_Y");
	add_parameter_float(&centralData->imu1.attitude.be[ACC_OFFSET+Z],"Bias_Acc_Z");
	
	add_parameter_float(&centralData->imu1.attitude.be[COMPASS_OFFSET+X],"Bias_Mag_X");
	add_parameter_float(&centralData->imu1.attitude.be[COMPASS_OFFSET+Y],"Bias_Mag_Y");
	add_parameter_float(&centralData->imu1.attitude.be[COMPASS_OFFSET+Z],"Bias_Mag_Z");
	
	// Scale factor
	//add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET+X],"Scale_Gyro_X");
	//add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET+Y],"Scale_Gyro_Y");
	//add_parameter_float(&centralData->imu1.raw_scale[GYRO_OFFSET+Z],"Scale_Gyro_Z");
	//
	//add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET+X],"Scale_Acc_X");
	//add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET+Y],"Scale_Acc_Y");
	//add_parameter_float(&centralData->imu1.raw_scale[ACC_OFFSET+Z],"Scale_Acc_Z");
	//
	//add_parameter_float(&centralData->imu1.raw_scale[COMPASS_OFFSET+X],"Scale_Mag_X");
	//add_parameter_float(&centralData->imu1.raw_scale[COMPASS_OFFSET+Y],"Scale_Mag_Y");
	//add_parameter_float(&centralData->imu1.raw_scale[COMPASS_OFFSET+Z],"Scale_Mag_Z");
	
	add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET+X],"Scale_Gyro_X");
	add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET+Y],"Scale_Gyro_Y");
	add_parameter_float(&centralData->imu1.attitude.sf[GYRO_OFFSET+Z],"Scale_Gyro_Z");
	
	add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET+X],"Scale_Acc_X");
	add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET+Y],"Scale_Acc_Y");
	add_parameter_float(&centralData->imu1.attitude.sf[ACC_OFFSET+Z],"Scale_Acc_Z");
	
	add_parameter_float(&centralData->imu1.attitude.sf[COMPASS_OFFSET+X],"Scale_Mag_X");
	add_parameter_float(&centralData->imu1.attitude.sf[COMPASS_OFFSET+Y],"Scale_Mag_Y");
	add_parameter_float(&centralData->imu1.attitude.sf[COMPASS_OFFSET+Z],"Scale_Mag_Z");

	//add_parameter_float(&centralData->position_estimator.kp_alt,"Pos_kp_alt");
	//add_parameter_float(&centralData->position_estimator.kp_vel_baro,"Pos_kp_velb");
	//add_parameter_float(&centralData->position_estimator.kp_pos[0],"Pos_kp_pos0");
	//add_parameter_float(&centralData->position_estimator.kp_pos[1],"Pos_kp_pos1");
	//add_parameter_float(&centralData->position_estimator.kp_pos[2],"Pos_kp_pos2");
	
	add_parameter_float(&centralData->dist2vel_gain,"vel_dist2Vel");
	add_parameter_float(&centralData->cruise_speed,"vel_cruiseSpeed");
	add_parameter_float(&centralData->max_climb_rate,"vel_climbRate");
	add_parameter_float(&centralData->softZoneSize,"vel_softZone");
}


task_return_t control_waypoint_timeout (void) 
{
	control_time_out_waypoint_msg(	&(centralData->number_of_waypoints),
									&centralData->waypoint_receiving,
									&centralData->waypoint_sending);
}


void mavlink_actions_handle_specific_messages (Mavlink_Received_t* rec) 
{
	if (rec->msg.sysid == MAVLINK_BASE_STATION_ID) 
	{	
		/*	Use this block for message debugging		
		dbg_print("\n Received message with ID");
		dbg_print_num(rec->msg.msgid, 10);
		dbg_print(" from system");
		dbg_print_num(rec->msg.sysid, 10);
		dbg_print(" for component");
		dbg_print_num(rec->msg.compid,10);
		dbg_print( "\n");
		*/

		switch(rec->msg.msgid) 
		{
			case MAVLINK_MSG_ID_MISSION_ITEM:	// 39 
				suspend_downstream(500000);
				receive_waypoint(	rec, 
									centralData->waypoint_list, 
									centralData->number_of_waypoints,
									&centralData->waypoint_receiving	);
				break;
	
			case MAVLINK_MSG_ID_MISSION_REQUEST : // 40
				suspend_downstream(500000);
				send_waypoint(	rec, 
								centralData->waypoint_list, 
								centralData->number_of_waypoints,
								&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_SET_CURRENT :  // 41
				set_current_waypoint(	rec, 
										centralData->waypoint_list, 
										centralData->number_of_waypoints	);
				break;

			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:  // 43
				// this initiates all waypoints being sent to the base-station - therefore, we pause the downstream telemetry to free the channel
				// (at least until we have a radio system with guaranteed bandwidth)
				suspend_downstream(500000);
				send_count(	rec, 
							centralData->number_of_waypoints,
							&centralData->waypoint_receiving,
							&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_COUNT :  // 44
				// this initiates all waypoints being sent from base-station - therefore, we pause the downstream telemetry to free the channel
				// (at least until we have a radio system with guaranteed bandwidth)
				suspend_downstream(500000);
				receive_count(	rec, 
								&(centralData->number_of_waypoints),
								&centralData->waypoint_receiving,
								&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_MISSION_CLEAR_ALL :  // 45
				clear_waypoint_list(	rec, 
										&(centralData->number_of_waypoints),
										&centralData->waypoint_set	);
				break;

			case MAVLINK_MSG_ID_MISSION_ACK :  // 47
				receive_ack_msg(	rec,
									&centralData->waypoint_sending	);
				break;

			case MAVLINK_MSG_ID_SET_MODE :  // 11
				set_mav_mode(	rec, 
								&centralData->mav_mode, 
								&(centralData->mav_state),
								centralData->simulation_mode	);
				break;

			case MAVLINK_MSG_ID_COMMAND_LONG :  // 76
				mavlink_actions_receive_message_long(rec);
				break;

			case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:  // 48
				set_home(rec);
				break;
		}
	} 
	else if (rec->msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT)
	{
		/* 
		 * Use this block for message debugging
		 * 
		 * dbg_print("\n Received message with ID");
		 * dbg_print_num(rec->msg.msgid, 10);
		 * dbg_print(" from system");
		 * dbg_print_num(rec->msg.sysid, 10);
		 * dbg_print(" for component");
		 * dbg_print_num(rec->msg.compid,10);
		 * dbg_print( "\n");
		*/

		neighbors_selection_read_message_from_neighbors(rec);
	}
}


void mavlink_actions_receive_message_long(Mavlink_Received_t* rec)
{
	mavlink_command_long_t packet;
	
	mavlink_msg_command_long_decode(&rec->msg, &packet);

	// Check if this message is for this system and subsystem
	if (((uint8_t)packet.target_system == (uint8_t)mavlink_system.sysid) && ((uint8_t)packet.target_component == (uint8_t)0))		// TODO check this 0
	{
		// print packet command and parameters for debug
		dbg_print("parameters:");
		dbg_print_num(packet.param1,10);
		dbg_print_num(packet.param2,10);
		dbg_print_num(packet.param3,10);
		dbg_print_num(packet.param4,10);
		dbg_print_num(packet.param5,10);
		dbg_print_num(packet.param6,10);
		dbg_print_num(packet.param7,10);
		dbg_print(", command id:");
		dbg_print_num(packet.command,10);
		dbg_print(", confirmation:");
		dbg_print_num(packet.confirmation,10);
		dbg_print("\n");
		
		switch(packet.command) 
		{
			case MAV_CMD_NAV_WAYPOINT:
				/* Navigate to MISSION. 
				| Hold time in decimal seconds. (ignored by fixed wing, time to stay at MISSION for rotary wing)
				| Acceptance radius in meters (if the sphere with this radius is hit, the MISSION counts as reached)
				| 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.
				| Desired yaw angle at MISSION (rotary wing)
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("Nav waypoint command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_UNLIM:
				/* Loiter around this MISSION an unlimited amount of time 
				| Empty
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("Nav loiter unlim command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_TURNS:
				/* Loiter around this MISSION for X turns 
				| Turns
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("Nav loiter turns command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LOITER_TIME:
				/* Loiter around this MISSION for X seconds 
				| Seconds (decimal)
				| Empty
				| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("Nav loiter time command, not implemented!\n");
				break;

			case MAV_CMD_NAV_RETURN_TO_LAUNCH:
				/* Return to launch location 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Nav Return to launch command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LAND:
				/* Land at location 
				| Empty
				| Empty
				| Empty
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("Command for automatic land, not implemented!\n");
				break;

			case MAV_CMD_NAV_TAKEOFF:
				/* Takeoff from ground / hand 
				| Minimum pitch (if airspeed sensor present), desired pitch without sensor
				| Empty
				| Empty
				| Yaw angle (if magnetometer present), ignored without magnetometer
				| Latitude
				| Longitude
				| Altitude
				| */
				centralData->in_the_air = true;
				dbg_print("Starting automatic take-off from button\n");
				break;

			case MAV_CMD_NAV_ROI:
				/* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. 
				| Region of interest mode. (see MAV_ROI enum)
				| MISSION index/ target ID. (see MAV_ROI enum)
				| ROI index (allows a vehicle to manage multiple ROI's)
				| Empty
				| x the location of the fixed ROI (see MAV_FRAME)
				| y
				| z
				| */
				dbg_print("Nav ROI command, not implemented!\n");
				break;

			case MAV_CMD_NAV_PATHPLANNING:
				/* Control autonomous path planning on the MAV. 
				| 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning
				| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid
				| Empty
				| Yaw angle at goal, in compass degrees, [0..360]
				| Latitude/X of goal
				| Longitude/Y of goal
				| Altitude/Z of goal
				| */
				dbg_print("Nav pathplanning command, not implemented!\n");
				break;

			case MAV_CMD_NAV_LAST:
				/* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Nav last command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_DELAY:
				/* Delay mission state machine. 
				| Delay in seconds (decimal)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Condition Delay command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_CHANGE_ALT:
				/* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. 
				| Descent / Ascend rate (m/s)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Finish Altitude
				| */
				dbg_print("Condition change alt command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_DISTANCE:
				/* Delay mission state machine until within desired distance of next NAV point. 
				| Distance (meters)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Condition distance command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_YAW:
				/* Reach a certain target angle. 
				| target angle: [0-360], 0 is north
				| speed during yaw change:[deg per second]
				| direction: negative: counter clockwise, positive: clockwise [-1,1]
				| relative offset or absolute angle: [ 1,0]
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Condition yaw command, not implemented!\n");
				break;

			case MAV_CMD_CONDITION_LAST:
				/* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Condition last command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_MODE:
				/* Set system mode. 
				| Mode, as defined by ENUM MAV_MODE
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Do set mode command, not implemented!\n");
				break;

			case MAV_CMD_DO_JUMP:
				/* Jump to the desired command in the mission list.  Repeat this action only the specified number of times 
				| Sequence number
				| Repeat count
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Do jump command, not implemented!\n");
				break;

			case MAV_CMD_DO_CHANGE_SPEED:
				/* Change speed and/or throttle set points. 
				| Speed type (0=Airspeed, 1=Ground Speed)
				| Speed  (m/s, -1 indicates no change)
				| Throttle  ( Percent, -1 indicates no change)
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("Do change speed command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_HOME:
				/* Changes the home location either to the current location or a specified location. 
				| Use current (1=use current location, 0=use specified location)
				| Empty
				| Empty
				| Empty
				| Latitude
				| Longitude
				| Altitude
				| */
				if (packet.param1 == 1)
				{
					// Set new home position to actual position
					dbg_print("Set new home location to actual position.\n");
					centralData->position_estimator.localPosition.origin = local_to_global_position(centralData->position_estimator.localPosition);
					centralData->sim_model.localPosition.origin = centralData->position_estimator.localPosition.origin;
					
					dbg_print("New Home location: (");
					dbg_print_num(centralData->position_estimator.localPosition.origin.latitude*10000000.0,10);
					dbg_print(", ");
					dbg_print_num(centralData->position_estimator.localPosition.origin.longitude*10000000.0,10);
					dbg_print(", ");
					dbg_print_num(centralData->position_estimator.localPosition.origin.altitude*1000.0,10);
					dbg_print(")\n");
				}
				else
				{
					// Set new home position from msg
					dbg_print("Set new home location. \n");
					
					centralData->position_estimator.localPosition.origin.latitude = packet.param5;
					centralData->position_estimator.localPosition.origin.longitude = packet.param6;
					centralData->position_estimator.localPosition.origin.altitude = packet.param7;
					centralData->sim_model.localPosition.origin = centralData->position_estimator.localPosition.origin;
					
					dbg_print("New Home location: (");
					dbg_print_num(centralData->position_estimator.localPosition.origin.latitude*10000000.0,10);
					dbg_print(", ");
					dbg_print_num(centralData->position_estimator.localPosition.origin.longitude*10000000.0,10);
					dbg_print(", ");
					dbg_print_num(centralData->position_estimator.localPosition.origin.altitude*1000.0,10);
					dbg_print(")\n");
				}

				centralData->waypoint_set = false;
				break;

			case MAV_CMD_DO_SET_PARAMETER:
				/* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. 
				| Parameter number
				| Parameter value
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Set parameter command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_RELAY:
				/* Set a relay to a condition. 
				| Relay number
				| Setting (1=on, 0=off, others possible depending on system hardware)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Set relay command, not implemented!\n");
				break;

			case MAV_CMD_DO_REPEAT_RELAY:
				/* Cycle a relay on and off for a desired number of cyles with a desired period. 
				| Relay number
				| Cycle count
				| Cycle time (seconds, decimal)
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Repeat relay command, not implemented!\n");
				break;

			case MAV_CMD_DO_SET_SERVO:
				/* Set a servo to a desired PWM value. 
				| Servo number
				| PWM (microseconds, 1000 to 2000 typical)
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Set servo command, not implemented!\n");
				break;

			case MAV_CMD_DO_REPEAT_SERVO:
				/* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. 
				| Servo number
				| PWM (microseconds, 1000 to 2000 typical)
				| Cycle count
				| Cycle time (seconds)
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Repeat servo command, not implemented!\n");
				break;

			case MAV_CMD_DO_CONTROL_VIDEO:
				/* Control onboard camera system. 
				| Camera ID (-1 for all)
				| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw
				| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)
				| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Control video command, not implemented!\n");
				break;

			case MAV_CMD_DO_LAST:
				/* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Do last command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_CALIBRATION:
				/* Trigger calibration. This command will be only accepted if in pre-flight mode. 
				| Gyro calibration: 0: no, 1: yes
				| Magnetometer calibration: 0: no, 1: yes
				| Ground pressure: 0: no, 1: yes
				| Radio calibration: 0: no, 1: yes
				| Accelerometer calibration: 0: no, 1: yes
				| Empty
				| Empty
				|  */
				dbg_print("Preflight calibration command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS:
				/* Set sensor offsets. This command will be only accepted if in pre-flight mode. 
				| Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow
				| X axis offset (or generic dimension 1), in the sensor's raw units
				| Y axis offset (or generic dimension 2), in the sensor's raw units
				| Z axis offset (or generic dimension 3), in the sensor's raw units
				| Generic dimension 4, in the sensor's raw units
				| Generic dimension 5, in the sensor's raw units
				| Generic dimension 6, in the sensor's raw units
				|  */
				dbg_print("Set sensor offsets command, not implemented!\n");
				break;

			case MAV_CMD_PREFLIGHT_STORAGE:
				/* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. 
				| Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM
				| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM
				| Reserved
				| Reserved
				| Empty
				| Empty
				| Empty
				|  */
				
				// Onboard parameters storage
				if (packet.param1 == 0) 
				{
					// read parameters from flash
					dbg_print("Reading from flashc...\n");
					read_parameters_from_flashc();
				}
				else if (packet.param1 == 1) 
				{
					// write parameters to flash
					//dbg_print("No Writing to flashc\n");
					dbg_print("Writing to flashc\n");
					write_parameters_to_flashc();
				}
				
				// Mission parameters storage
				if (packet.param2 == 0) 
				{
					// read mission from flash
				}
				else if (packet.param2 == 1) 
				{
					// write mission to flash
				}
				break;

			case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
				/* Request the reboot or shutdown of system components. 
				| 0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.
				| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.
				| Reserved
				| Reserved
				| Empty
				| Empty
				| Empty
				|  */
				dbg_print("Reboot/Shutdown command, not implemented!\n");
				break;

			case MAV_CMD_OVERRIDE_GOTO:
				/* Hold / continue the current action 
				| MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan
				| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position
				| MAV_FRAME coordinate frame of hold point
				| Desired yaw angle in degrees
				| Latitude / X position
				| Longitude / Y position
				| Altitude / Z position
				|  */
				dbg_print("Goto command, not implemented!\n");
				break;

			case MAV_CMD_MISSION_START:
				/* start running a mission 
				| first_item: the first mission item to run
				| last_item:  the last mission item to run (after this item is run, the mission ends)
				|  */
				dbg_print("Mission start command, not implemented!\n");
				break;

			case MAV_CMD_COMPONENT_ARM_DISARM:
				/* Arms / Disarms a component 
				| 1 to arm, 0 to disarm
				|  */
				dbg_print("Disarm command, not implemented!\n");
				break;

			case MAV_CMD_ENUM_END:
				/*  
				| 
				*/
				break;

		}
	}
	

	if((uint8_t)packet.target_component == (uint8_t)MAV_COMP_ID_MISSIONPLANNER)		// TODO: this part needs comments
	{
		// print packet command and parameters for debug
		dbg_print("All vehicles parameters:");
		dbg_print_num(packet.param1,10);
		dbg_print_num(packet.param2,10);
		dbg_print_num(packet.param3,10);
		dbg_print_num(packet.param4,10);
		dbg_print_num(packet.param5,10);
		dbg_print_num(packet.param6,10);
		dbg_print_num(packet.param7,10);
		dbg_print(", command id:");
		dbg_print_num(packet.command,10);
		dbg_print(", confirmation:");
		dbg_print_num(packet.confirmation,10);
		dbg_print("\n");
		
		switch(packet.command) {
			case MAV_CMD_NAV_RETURN_TO_LAUNCH:
				/* Return to launch location 
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| Empty
				| */
				dbg_print("All MAVs: Return to first waypoint. \n");
				set_current_waypoint_from_parameter(centralData->waypoint_list,centralData->number_of_waypoints,0);
				break;

			case MAV_CMD_NAV_LAND:
				/* Land at location 
				| Empty
				| Empty
				| Empty
				| Desired yaw angle.
				| Latitude
				| Longitude
				| Altitude
				| */
				dbg_print("All MAVs: Auto-landing");
				break;

			case MAV_CMD_MISSION_START:
				/* start running a mission 
				| first_item: the first mission item to run
				| last_item:  the last mission item to run (after this item is run, the mission ends)
				| */
				dbg_print("All vehicles: Navigating to next waypoint. \n");
				continueToNextWaypoint();
				break;

			case MAV_CMD_CONDITION_LAST:
				/*
				| */
				dbg_print("All MAVs: setting circle scenario!\n");
				set_circle_scenario(centralData->waypoint_list, &(centralData->number_of_waypoints), packet.param1, packet.param2);
				break;			
		}
	}
	
}


void mavlink_send_sonar(void)
{
	mavlink_msg_named_value_float_send(	MAVLINK_COMM_0, 
										get_millis(),
										"sonar(m)", 
										centralData->i2cxl_sonar.distance_m);
}

void mavlink_actions_init(void) {
	
	centralData=central_data_get_pointer_to_struct();
	mavlink_actions_add_onboard_parameters();
	
	/*	
	 * Use this to store or read or reset parameters on flash memory
	 *
	write_parameters_to_flashc();
	read_parameters_from_flashc();
	 *
	 */

	scheduler_add_task(get_mavlink_taskset(),  10000,    RUN_REGULAR,  &control_waypoint_timeout,                0	);
	
	scheduler_add_task(get_mavlink_taskset(),  1000000,  RUN_REGULAR,  &mavlink_send_heartbeat,                  MAVLINK_MSG_ID_HEARTBEAT	);

	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_REGULAR,  &mavlink_send_attitude_quaternion,        MAVLINK_MSG_ID_ATTITUDE_QUATERNION	);

	scheduler_add_task(get_mavlink_taskset(),  200000,   RUN_REGULAR,  &mavlink_send_attitude,                   MAVLINK_MSG_ID_ATTITUDE	);
	
	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_NEVER,    &mavlink_send_hud,                        MAVLINK_MSG_ID_VFR_HUD	);

	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_NEVER,    &mavlink_send_pressure,                   MAVLINK_MSG_ID_SCALED_PRESSURE	);

	scheduler_add_task(get_mavlink_taskset(),  250000,   RUN_REGULAR,  &mavlink_send_scaled_imu,                 MAVLINK_MSG_ID_SCALED_IMU	);

	scheduler_add_task(get_mavlink_taskset(),  100000,   RUN_REGULAR,  &mavlink_send_raw_imu,                    MAVLINK_MSG_ID_RAW_IMU	);

	scheduler_add_task(get_mavlink_taskset(),  200000,   RUN_NEVER,    &mavlink_send_rpy_rates_error,            MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT	);

	scheduler_add_task(get_mavlink_taskset(),  200000,   RUN_NEVER,    &mavlink_send_rpy_speed_thrust_setpoint,  MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT	);

	scheduler_add_task(get_mavlink_taskset(),  200000,   RUN_NEVER,    &mavlink_send_rpy_thrust_setpoint,        MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT	);

	scheduler_add_task(get_mavlink_taskset(),  1000000,  RUN_NEVER,    &mavlink_send_servo_output,               MAVLINK_MSG_ID_SERVO_OUTPUT_RAW	);

	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_NEVER,    &mavlink_send_estimator,                  MAVLINK_MSG_ID_LOCAL_POSITION_NED	);

	scheduler_add_task(get_mavlink_taskset(),  250000,   RUN_REGULAR,  &mavlink_send_global_position,            MAVLINK_MSG_ID_GLOBAL_POSITION_INT	);

	scheduler_add_task(get_mavlink_taskset(),  1000000,  RUN_NEVER,    &mavlink_send_gps_raw,                    MAVLINK_MSG_ID_GPS_RAW_INT	);

	scheduler_add_task(get_mavlink_taskset(),  250000,   RUN_NEVER,    &mavlink_send_raw_rc_channels,            MAVLINK_MSG_ID_RC_CHANNELS_RAW	);

	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_NEVER,    &mavlink_send_scaled_rc_channels,         MAVLINK_MSG_ID_RC_CHANNELS_SCALED	);

	scheduler_add_task(get_mavlink_taskset(),  500000,   RUN_NEVER,    &mavlink_send_simulation,                 MAVLINK_MSG_ID_HIL_STATE	);

	scheduler_add_task(get_mavlink_taskset(),  250000,   RUN_NEVER,    &mavlink_send_rt_stats,                   MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);

	scheduler_add_task(get_mavlink_taskset(),  100000,   RUN_REGULAR,  &mavlink_send_sonar,                      MAVLINK_MSG_ID_NAMED_VALUE_FLOAT	);

	scheduler_sort_taskset_by_period(get_mavlink_taskset());
	
	dbg_print("MAVlink actions initialiased\n");
}
