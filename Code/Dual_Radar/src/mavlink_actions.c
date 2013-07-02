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
#include "doppler_radar.h"

board_hardware_t *board;


mavlink_send_heartbeat() {
	board_hardware_t *board=get_board_hardware();
	if (board->controls.run_mode==MOTORS_OFF) {
		mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_DISARMED, 0, MAV_STATE_STANDBY);
	}else {
		mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_STABILIZE_ARMED, 0, MAV_STATE_ACTIVE);
	}		
}	


int even_odd;
void mavlink_send_radar() {
	radar_target *target=get_tracked_target();
	mavlink_msg_radar_tracked_target_send(MAVLINK_COMM_0, get_millis(), 0, 0, target->velocity, target->amplitude, 0.0);
	mavlink_msg_radar_raw_data_send(MAVLINK_COMM_0, get_millis(), 0, get_raw_FFT());
	mavlink_msg_radar_raw_data_send(MAVLINK_COMM_0, get_millis(), 1, ADCI_get_buffer(0, 0));
	mavlink_msg_radar_raw_data_send(MAVLINK_COMM_0, get_millis(), 2, ADCI_get_buffer(0, 1));
	

	
//	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Radar_velocity", target->velocity);
//	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, get_millis(), "Radar_amplitude", target->amplitude/1000.0);
}




void init_mavlink_actions() {
	board=get_board_hardware();
	register_task(get_mavlink_taskset(), 1, 500000, &mavlink_send_heartbeat);
	//register_task(get_mavlink_taskset(), 7,  50000, &mavlink_send_radar);

}
