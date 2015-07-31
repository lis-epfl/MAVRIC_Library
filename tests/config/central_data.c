/*
 * central_data.c
 *
 * Created: 16/09/2013 12:18:14
 *  Author: sfx
 */ 


#include "central_data.h"
#include "simulation.h"
#include "conf_sim_model.h"


static volatile central_data_t central_data;

void central_data_init(){
		
		// init controls
		central_data.controls.rpy[ROLL]=0;
		central_data.controls.rpy[PITCH]=0;
		central_data.controls.rpy[YAW]=0;
		central_data.controls.thrust=-1.0;
		
		central_data.controls.run_mode = MOTORS_OFF;
		
		// init stabilisers
		stabilisation_copter_init(&central_data.stabiliser_stack);
		// central_data.stabilisers_stack = 

		central_data.simulation_mode=0;

		// init waypoint navigation
		central_data.number_of_waypoints = 0;		
		central_data.waypoint_set = false;
		central_data.waypoint_sending = false;
		central_data.waypoint_receiving = false;
		
		central_data.critical_landing = false;
		
		central_data.collision_avoidance = false;
		central_data.automatic_take_off = false;
		
		// default GPS home position
		central_data.position_estimation.local_position.origin.longitude=   HOME_LONGITUDE;
		central_data.position_estimation.local_position.origin.latitude =   HOME_LATITUDE;
		central_data.position_estimation.local_position.origin.altitude =   HOME_ALTITUDE;
		central_data.position_estimation.local_position.pos[0]=0;
		central_data.position_estimation.local_position.pos[1]=0;
		central_data.position_estimation.local_position.pos[2]=0;

		// init simulation
		simulation_init(&(central_data.sim_model),&(central_data.imu.attitude));
		central_data.sim_model.local_position = central_data.position_estimation.local_position;

}

central_data_t* central_data_get_pointer_to_struct(void)
{
	return &central_data;
}

byte_stream_t* get_telemetry_upstream() {
	return central_data.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return central_data.telemetry_down_stream;
}


imu_t* get_imu_data() {
	return &central_data.imu;
}
control_command_t* get_control_inputs_data() {
	return &central_data.controls;
}