/*
 * central_data.c
 *
 * Created: 16/09/2013 12:18:14
 *  Author: sfx
 */ 


#include "central_data.h"
#include "remote_controller.h"
#include "simulation.h"
#include "conf_sim_model.h"


static volatile central_data_t centralData;

void central_data_init(){
		
		// init controls
		centralData.controls.rpy[ROLL]=0;
		centralData.controls.rpy[PITCH]=0;
		centralData.controls.rpy[YAW]=0;
		centralData.controls.thrust=-1.0;
		
		centralData.controls.run_mode = MOTORS_OFF;
		
		// init stabilisers
		stabilisation_copter_init(&centralData.stabiliser_stack);
		// centralData.stabilisers_stack = 

		centralData.simulation_mode=0;

		// init waypoint navigation
		centralData.number_of_waypoints = 0;		
		centralData.waypoint_set = false;
		centralData.waypoint_sending = false;
		centralData.waypoint_receiving = false;
		
		centralData.critical_landing = false;
		
		centralData.collision_avoidance = false;
		centralData.automatic_take_off = false;
		
		// default GPS home position
		centralData.position_estimator.localPosition.origin.longitude=   HOME_LONGITUDE;
		centralData.position_estimator.localPosition.origin.latitude =   HOME_LATITUDE;
		centralData.position_estimator.localPosition.origin.altitude =   HOME_ALTITUDE;
		centralData.position_estimator.localPosition.pos[0]=0;
		centralData.position_estimator.localPosition.pos[1]=0;
		centralData.position_estimator.localPosition.pos[2]=0;

		// init simulation
		simulation_init(&(centralData.sim_model),&(centralData.imu1.attitude));
		centralData.sim_model.localPosition = centralData.position_estimator.localPosition;

}


byte_stream_t* get_telemetry_upstream() {
	return centralData.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return centralData.telemetry_down_stream;
}
byte_stream_t* get_debug_stream() {
	return centralData.debug_out_stream;
}

Imu_Data_t* get_imu_data() {
	return &centralData.imu1;
}
Control_Command_t* get_control_inputs_data() {
	return &centralData.controls;
}