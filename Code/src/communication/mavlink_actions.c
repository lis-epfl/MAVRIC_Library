/*
 * mavlink_actions.c
 *
 * Created: 21/03/2013 01:00:56
 *  Author: sfx
 */ 
#include "mavlink_actions.h"

#include "boardsupport.h"
#include "onboard_parameters.h"

#include "scheduler.h"


mavlink_send_heartbeat() {
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_CALIBRATING);
}	

void mavlink_send_attitude() {
	// ATTITUDE QUATERNION
	board_hardware_t *board=get_board_hardware();
	mavlink_msg_attitude_quaternion_send(MAVLINK_COMM_0, 0, board->imu1.attitude.qe.s, board->imu1.attitude.qe.v[0], board->imu1.attitude.qe.v[1], board->imu1.attitude.qe.v[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
				
	// ATTITUDE
	Aero_Attitude_t aero_attitude;
	aero_attitude=Quat_to_Aero(board->imu1.attitude.qe);
	//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
	//mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, aero_attitude.rpy[0], aero_attitude.rpy[1], aero_attitude.rpy[2], imu1.attitude.om[0], imu1.attitude.om[1], imu1.attitude.om[2]);
				
	Schill_Attitude_t schill_attitude;
	schill_attitude=Quat_to_Schill(board->imu1.attitude.qe);
	//mavlink_msg_attitude_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed)
	mavlink_msg_attitude_send(MAVLINK_COMM_0, 0, schill_attitude.rpy[0], schill_attitude.rpy[1], schill_attitude.rpy[2], board->imu1.attitude.om[0], board->imu1.attitude.om[1], board->imu1.attitude.om[2]);
				
	// Controls output
	//mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(mavlink_channel_t chan, uint32_t time_boot_ms, float roll, float pitch, float yaw, float thrust)
	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, 0, board->controls.rpy[ROLL], board->controls.rpy[PITCH], board->controls.rpy[YAW], board->controls.thrust);
				
	// GPS COORDINATES (TODO : Add GPS to the platform)
	//mavlink_msg_global_position_int_send(mavlink_channel_t chan, uint32_t time_boot_ms, int32_t lat, int32_t lon, int32_t alt, int32_t relative_alt, int16_t vx, int16_t vy, int16_t vz, uint16_t hdg)
	mavlink_msg_global_position_int_send(MAVLINK_COMM_0, 0, 46.5193*10000000, 6.56507*10000000, 400, 1, 0, 0, 0, board->imu1.attitude.om[2]);

	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_X", board->imu1.raw_channels[COMPASS_OFFSET+IMU_X]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_Y", board->imu1.raw_channels[COMPASS_OFFSET+IMU_Y]);
	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Compass_Z", board->imu1.raw_channels[COMPASS_OFFSET+IMU_Z]);

}


void mavlink_send_pressure() {			
	//pressure=get_pressure_data_slow();
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Pressure", pressure->pressure);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Temperature", pressure->temperature);
	//mavlink_msg_named_value_float_send(MAVLINK_COMM_0, 0, "Altitude", pressure->altitude);
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


void init_mavlink_actions() {
	add_PID_parameters();
	
}