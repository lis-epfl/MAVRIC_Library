/*
 * mavlink_actions.c
 *
 * Created: 21/03/2013 01:00:56
 *  Author: sfx
 */ 
#include "mavlink_actions.h"

#include "boardsupport.h"
#include "onboard_parameters.h"
#include "mavlink_stream.h"
#include "scheduler.h"
#include "radar_module_driver.h"

board_hardware_t *board;

void mavlink_send_heartbeat(void) {
	board_hardware_t *board=get_board_hardware();
	//if (board->controls.run_mode==MOTORS_OFF) {
		//mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_STANDBY);
	//}else {
		//mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);
	//}
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, board->mav_mode, 0, board->mav_state);
		mavlink_msg_sys_status_send(MAVLINK_COMM_0, 
								0b1111110000100111, // sensors present
								0b1111110000100111, // sensors enabled
								0b1111110000100111, // sensors health
								0,                  // load
								12000,              // bat voltage (mV)
								100,                // current (mA)
								0, 0,  				// comms drop, comms errors
								0, 0, 0, 0,         // autopilot specific errors
								99);                // battery remaining
}

void mavlink_send_raw_imu(void) {
	mavlink_msg_raw_imu_send(MAVLINK_COMM_0, get_micros(), 
	board->imu1.raw_channels[ACC_OFFSET+IMU_X], 
	board->imu1.raw_channels[ACC_OFFSET+IMU_Y], 
	board->imu1.raw_channels[ACC_OFFSET+IMU_Z], 
	board->imu1.raw_channels[GYRO_OFFSET+IMU_X], 
	board->imu1.raw_channels[GYRO_OFFSET+IMU_Y], 
	board->imu1.raw_channels[GYRO_OFFSET+IMU_Z], 
	board->imu1.raw_channels[COMPASS_OFFSET+IMU_X], 
	board->imu1.raw_channels[COMPASS_OFFSET+IMU_Y], 
	board->imu1.raw_channels[COMPASS_OFFSET+IMU_Z]
	);
}

void mavlink_send_scaled_imu(void) {
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, get_millis(),
	1000*board->imu1.attitude.a [IMU_X],
	1000*board->imu1.attitude.a [IMU_Y], 
	1000*board->imu1.attitude.a [IMU_Z], 
	1000*board->imu1.attitude.om[IMU_X], 
	1000*board->imu1.attitude.om[IMU_Y], 
	1000*board->imu1.attitude.om[IMU_Z], 
	1000*board->imu1.attitude.mag[IMU_X],
	1000*board->imu1.attitude.mag[IMU_Y],
	1000*board->imu1.attitude.mag[IMU_Z]
	//1000*board->imu1.attitude.up_vec.v[0],
	//1000*board->imu1.attitude.up_vec.v[1],
	//1000*board->imu1.attitude.up_vec.v[2]
	);
}
void  mavlink_send_rpy_rates_error(void) {
	Stabiliser_t *rate_stab=get_rate_stabiliser();
	mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(MAVLINK_COMM_0, get_millis(), rate_stab->rpy_controller[0].error, rate_stab->rpy_controller[1].error,rate_stab->rpy_controller[2].error,0 );
}
void  mavlink_send_rpy_speed_thrust_setpoint(void) {
	Stabiliser_t *rate_stab=get_rate_stabiliser();
	mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_send(MAVLINK_COMM_0, get_millis(), rate_stab->rpy_controller[0].output, rate_stab->rpy_controller[1].output,rate_stab->rpy_controller[2].output,0 );
}
void mavlink_send_rpy_thrust_setpoint(void) {
	
	// Controls output
	//mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust)
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, get_millis(), board->controls.rpy[ROLL], board->controls.rpy[PITCH], board->controls.rpy[YAW], board->controls.thrust);
}

void mavlink_send_servo_output(void) {
	Stabiliser_t *rate_stab=get_rate_stabiliser();
	mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, get_millis(), 0, 
	(uint16_t)(board->servos[0].value+1500),
	(uint16_t)(board->servos[1].value+1500),
	(uint16_t)(board->servos[2].value+1500),
	(uint16_t)(board->servos[3].value+1500),
	1000*rate_stab->output.rpy[0]+1000, 
	1000*rate_stab->output.rpy[1]+1000,
	1000*rate_stab->output.rpy[2]+1000,
	1000*rate_stab->output.thrust+1000
	);
}

void mavlink_send_attitude_quaternion(void) {
	// ATTITUDE QUATERNION
	mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, get_millis(), board->imu1.attitude.qe.s, board->imu1.attitude.qe.v[0], board->imu1.attitude.qe.v[1], board->imu1.attitude.qe.v[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
}
void mavlink_send_attitude(void) {
	// ATTITUDE
	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(board->imu1.attitude.qe);
	mavlink_msg_attitude_send(MAVLINK_COMM_0, get_millis(), aero_attitude.rpy[0], aero_attitude.rpy[1], aero_attitude.rpy[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
}

void mavlink_send_global_position(void) {				
	// GPS COORDINATES (TODO : Add GPS to the platform)
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	
   if (board->GPS_data.status == GPS_OK)
   {
	   // board->GPS_data.timeLastMsg
	   mavlink_msg_global_position_int_send(MAVLINK_COMM_0, get_millis() , board->GPS_data.latitude*10000000.0, board->GPS_data.longitude*10000000.0, board->GPS_data.altitude*1000.0, 1, board->GPS_data.northSpeed*100.0, board->GPS_data.eastSpeed*100.0, board->GPS_data.verticalSpeed*100.0, board->GPS_data.course);
   }else{
	   //mavlink_msg_global_position_int_send(MAVLINK_COMM_0, get_millis(), 46.5193*10000000, 6.56507*10000000, 400*1000, 1, 0, 0, 0, board->imu1.attitude.om[2]);
	   	// send integrated position (for now there is no GPS error correction...!!!)
		global_position_t gpos=local_to_global_position(board->imu1.attitude.localPosition);
		mavlink_msg_global_position_int_send(MAVLINK_COMM_0, get_millis(), gpos.latitude*10000000, gpos.longitude*10000000, gpos.altitude*1000.0, 1, board->imu1.attitude.vel[0]*100.0, board->imu1.attitude.vel[1]*100.0, board->imu1.attitude.vel[2]*100.0, board->imu1.attitude.om[2]);

   } 
   
}

void mavlink_send_hud(void) {
	float groundspeed=sqrt(board->imu1.attitude.vel[0]*board->imu1.attitude.vel[0] +board->imu1.attitude.vel[1]*board->imu1.attitude.vel[1]);
	float airspeed=groundspeed;
	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(board->imu1.attitude.qe);
	mavlink_msg_vfr_hud_send(MAVLINK_COMM_0, airspeed, groundspeed, 180.0*aero_attitude.rpy[2]/PI, (int)((board->controls.thrust+1.0)*50), -board->imu1.attitude.localPosition.pos[2], -board->imu1.attitude.vel[2]);

	
}

void mavlink_send_gps_raw(void) {	
	// mavlink_msg_gps_raw_int_send(mavlink_channel_t chan, uint64_t time_usec, uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, uint16_t cog, uint8_t satellites_visible)
	if (board->GPS_data.status == GPS_OK)
	{
		mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,get_millis(), board->GPS_data.status, board->GPS_data.latitude*10000000.0, board->GPS_data.longitude*10000000.0, board->GPS_data.altitude*1000.0, board->GPS_data.hdop*100.0, board->GPS_data.speedAccuracy*100.0 ,board->GPS_data.groundSpeed*100.0, board->GPS_data.course, board->GPS_data.num_sats);	
	}else{
		mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,get_millis(), board->GPS_data.status, 46.5193*10000000, 6.56507*10000000, 400 * 1000, 0, 0 , 0, 0, board->GPS_data.num_sats);
	}
	
	
}


void mavlink_send_pressure(void) {			
	pressure_data *pressure=get_pressure_data_slow();
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Pressure", pressure->pressure/100.0);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Temperature", pressure->temperature);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Altitude", pressure->altitude);
}

void mavlink_send_radar(void) {
	read_radar();
	radar_target *target=get_radar_main_target();
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Radar_velocity", target->velocity);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Radar_amplitude", target->amplitude/1000.0);
}

void mavlink_send_estimator(void)
{
	//mavlink_msg_local_position_ned_send(mavlink_channel_t chan, uint32_t time_boot_ms, float x, float y, float z, float vx, float vy, float vz)
	//mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, get_millis(), board->estimation.state[0][0], board->estimation.state[1][0], board->estimation.state[2][0], board->estimation.state[0][1], board->estimation.state[1][1], board->estimation.state[2][1]);
	mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, get_millis(), board->imu1.attitude.localPosition.pos[0], board->imu1.attitude.localPosition.pos[1], board->imu1.attitude.localPosition.pos[2], board->imu1.attitude.vel[0], board->imu1.attitude.vel[1], board->imu1.attitude.vel[2]);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0,0,"Estimation",0);
	
}

void mavlink_send_kalman_estimator(void)
{
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiX", board->estimation.state[0][0]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiY", board->estimation.state[1][0]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiZ", board->estimation.state[2][0]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiVx", board->estimation.state[0][1]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiVy", board->estimation.state[1][1]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiVz", board->estimation.state[2][1]);
	
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "acc_bfX", board->imu1.attitude.acc_bf[0]);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "acc_bfY", board->imu1.attitude.acc_bf[1]);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "acc_bfZ", board->imu1.attitude.acc_bf[2]);
	
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "estiDeltaT", board->estimation.delta_t_filter);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "imuDeltaT", board->imu1.dt);
}
void mavlink_send_raw_rc_channels(void)
{
	if (checkReceivers()>0)
	{
		mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0,get_millis(),1,
		getChannel(S_THROTTLE)+347,
		getChannel(S_ROLL)+347,
		getChannel(S_PITCH)+347,
		getChannel(S_YAW)+433,
		getChannel(4)+500,
		getChannel(5)+500,
		65535,65535,255);
	}else{
		mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0,get_millis(),1,0,0,0,0,0,0,65535,65535,0);
	}
}

void mavlink_send_scaled_rc_channels(void)
{
	if (checkReceivers()>0)
	{
		mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0,get_millis(),1,
		(getChannel(S_THROTTLE)+347) * 10000 / 716,
		(getChannel(S_ROLL)) * 10000 / 359,
		(getChannel(S_PITCH) - 11) * 10000 / 359,
		(getChannel(S_YAW) - 4) * 10000 / 456,
		getChannel(4)+500,
		getChannel(5)+500,
		32767,32767,255);
	}else{	
		mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0,get_millis(),1,0,0,0,0,0,0,32767,32767,0);
	}
}

void mavlink_send_simulation(void) {
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "rolltorque", board->sim_model.torques_bf[0]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "pitchtorque", board->sim_model.torques_bf[1]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "yawtorque", board->sim_model.torques_bf[2]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "thrust", board->sim_model.lin_forces_bf[2]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "rpm1", board->sim_model.rotorspeeds[0]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "rpm2", board->sim_model.rotorspeeds[1]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "rpm3", board->sim_model.rotorspeeds[2]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "rpm4", board->sim_model.rotorspeeds[3]);

	
}

void add_PID_parameters(void) {
	Stabiliser_t* rate_stabiliser = get_rate_stabiliser();
	Stabiliser_t* attitude_stabiliser = get_attitude_stabiliser();
	// Roll rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].p_gain, "RollRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].clip_max, "RollRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].clip_min, "RollRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.clip, "RollRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollRPid_D_LPF");
	
	// Roll attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].p_gain, "RollAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].clip_max, "RollAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].clip_min, "RollAPid_P_CLmn");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.clip, "RollAPid_I_CLip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.postgain, "RollAPid_I_PstG");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].integrator.pregain, "RollAPid_I_PreG");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.clip, "RollAPid_D_Clip");
	add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.gain, "RollAPid_D_Gain");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[ROLL].differentiator.LPF, "RollAPid_D_LPF");

	// Pitch rate PID
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].p_gain, "PitchRPid_P_G");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].clip_max, "PitchRPid_P_CLmx");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].clip_min, "PitchRPid_P_CLmn");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.clip, "PitchRPid_I_CLip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.postgain, "PitchRPid_I_PstG");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].integrator.pregain, "PitchRPid_I_PreG");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.clip, "PitchRPid_D_Clip");
	add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.gain, "PitchRPid_D_Gain");
	//add_parameter_float(&rate_stabiliser->rpy_controller[PITCH].differentiator.LPF, "PitchRPid_D_LPF");
	
	// Pitch attitude PID
	add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].p_gain, "PitchAPid_P_G");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].clip_max, "PitchAPid_P_CLmx");
	//add_parameter_float(&attitude_stabiliser->rpy_controller[PITCH].clip_min, "PitchAPid_P_CLmn");
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

}

void init_mavlink_actions(void) {
	board=get_board_hardware();
	add_PID_parameters();
	add_task(get_mavlink_taskset(), 1000000,RUN_REGULAR, &mavlink_send_heartbeat, MAVLINK_MSG_ID_HEARTBEAT);
	add_task(get_mavlink_taskset(), 100000, RUN_REGULAR, &mavlink_send_attitude, MAVLINK_MSG_ID_ATTITUDE);
	add_task(get_mavlink_taskset(), 100000, RUN_REGULAR, &mavlink_send_attitude_quaternion, MAVLINK_MSG_ID_ATTITUDE_QUATERNION);
	add_task(get_mavlink_taskset(), 250000, RUN_REGULAR, &mavlink_send_hud, MAVLINK_MSG_ID_VFR_HUD);
	add_task(get_mavlink_taskset(), 150000, RUN_REGULAR, &mavlink_send_pressure, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
	add_task(get_mavlink_taskset(), 200000, RUN_REGULAR, &mavlink_send_scaled_imu, MAVLINK_MSG_ID_SCALED_IMU);
	add_task(get_mavlink_taskset(), 200000, RUN_REGULAR, &mavlink_send_raw_imu, MAVLINK_MSG_ID_RAW_IMU);
	add_task(get_mavlink_taskset(), 100000, RUN_NEVER, &mavlink_send_rpy_rates_error, MAVLINK_MSG_ID_ROLL_PITCH_YAW_RATES_THRUST_SETPOINT);
	add_task(get_mavlink_taskset(), 100000, RUN_NEVER, &mavlink_send_rpy_speed_thrust_setpoint, MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT);
	add_task(get_mavlink_taskset(), 200000, RUN_NEVER, &mavlink_send_servo_output, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW);
	//add_task(get_mavlink_taskset(),  50000, &mavlink_send_radar);
	add_task(get_mavlink_taskset(), 100000, RUN_REGULAR, &mavlink_send_estimator, MAVLINK_MSG_ID_LOCAL_POSITION_NED);
	add_task(get_mavlink_taskset(), 100000, RUN_REGULAR, &mavlink_send_global_position, MAVLINK_MSG_ID_GLOBAL_POSITION_INT);
	add_task(get_mavlink_taskset(), 100000, RUN_NEVER, &mavlink_send_raw_rc_channels, MAVLINK_MSG_ID_RC_CHANNELS_RAW);
	add_task(get_mavlink_taskset(), 100000, RUN_NEVER, &mavlink_send_scaled_rc_channels, MAVLINK_MSG_ID_RC_CHANNELS_SCALED);
	add_task(get_mavlink_taskset(), 200000, RUN_NEVER, &mavlink_send_simulation, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
	add_task(get_mavlink_taskset(), 100000, RUN_REGULAR, &mavlink_send_kalman_estimator, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT);
	
	sort_taskset_by_period(get_mavlink_taskset());
}
