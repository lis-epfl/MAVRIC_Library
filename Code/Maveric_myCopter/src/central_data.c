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

void initialise_central_data(){
		// Init servos
		for (int i = 0; i < NUMBER_OF_SERVO_OUTPUTS; ++i)
		{
			centralData.servos[i].value = -600;
			centralData.servos[i].min = -600;
			centralData.servos[i].max = 600;
			centralData.servos[i].failsafe_position = -600;
		}
		servos_failsafe(centralData.servos);
		set_servos(centralData.servos);

		init_imu(&centralData.imu1);
		
		// init controls
		centralData.controls.rpy[ROLL]=0;
		centralData.controls.rpy[PITCH]=0;
		centralData.controls.rpy[YAW]=0;
		centralData.controls.tvel[X]=0;
		centralData.controls.tvel[Y]=0;
		centralData.controls.tvel[Z]=0;
		centralData.controls.thrust=-1.0;
		
		centralData.run_mode = MOTORS_OFF;
		
		// init stabilisers
		init_stabilisation_copter(&centralData.stabiliser_stack);

		centralData.simulation_mode=0;
		centralData.simulation_mode_previous = 0;

		// init waypoint navigation
		centralData.number_of_waypoints = 0;		
		centralData.waypoint_set = false;
		centralData.waypoint_sending = false;
		centralData.waypoint_receiving = false;
		
		centralData.critical_landing = false;
		
		centralData.collision_avoidance = false;
		centralData.automatic_take_off = false;
		centralData.automatic_landing = false;
		centralData.in_the_air = false;
		
		centralData.number_of_neighbors = 0;
		
		// default GPS home position
		centralData.position_estimator.localPosition.origin.longitude=   HOME_LONGITUDE;
		centralData.position_estimator.localPosition.origin.latitude =   HOME_LATITUDE;
		centralData.position_estimator.localPosition.origin.altitude =   HOME_ALTITUDE;
		centralData.position_estimator.localPosition.pos[X]=0;
		centralData.position_estimator.localPosition.pos[Y]=0;
		centralData.position_estimator.localPosition.pos[Z]=0;

		// init simulation
		init_simulation(&(centralData.sim_model),&(centralData.imu1),centralData.position_estimator.localPosition);
		//centralData.sim_model.localPosition = centralData.position_estimator.localPosition;

		centralData.dist2vel_gain = 0.7;
		centralData.cruise_speed = 3.0;
		centralData.max_climb_rate = 1.0;
		centralData.softZoneSize = 0.0;

		// Init sonar
		// i2cxl_sonar_init(&centralData.i2cxl_sonar);
}

central_data_t* get_central_data(void)
{
	return &centralData;
}

byte_stream_t* get_telemetry_upstream() {
	return centralData.telemetry_up_stream;
}
byte_stream_t* get_telemetry_downstream() {
	return centralData.telemetry_down_stream;
}

Imu_Data_t* get_imu_data() {
	return &centralData.imu1;
}
Control_Command_t* get_control_inputs_data() {
	return &centralData.controls;
}