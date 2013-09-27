/*
 * central_data.c
 *
 * Created: 16/09/2013 12:18:14
 *  Author: sfx
 */ 


#include "central_data.h"
#include "remote_controller.h"

static volatile central_data_t centralData;



central_data_t* initialise_central_data(void)
{
	enum GPS_Engine_Setting engine_nav_settings = GPS_ENGINE_AIRBORNE_4G;
	
	init_imu(&centralData.imu1);
	init_bmp085();

	rc_init();

	init_simulation(&centralData.sim_model);
	
	centralData.controls.rpy[ROLL]=0;
	centralData.controls.rpy[PITCH]=0;
	centralData.controls.rpy[YAW]=0;
	centralData.controls.thrust=-1.0;
	
	centralData.number_of_waypoints = 0;

	centralData.simulation_mode=0;
	
	centralData.waypoint_set = false;
	centralData.mission_started = false;
	centralData.waypoint_sending = false;
	centralData.waypoint_receiving = false;
	centralData.waypoint_hold_init = false;
	
	// default GPS home position
	centralData.imu1.attitude.localPosition.origin.longitude=   HOME_LONGITUDE;
	centralData.imu1.attitude.localPosition.origin.latitude =   HOME_LATITUDE;
	centralData.imu1.attitude.localPosition.origin.altitude =   HOME_ALTITUDE;
	centralData.imu1.attitude.localPosition.pos[0]=0;
	centralData.imu1.attitude.localPosition.pos[1]=0;
	centralData.imu1.attitude.localPosition.pos[2]=0;
	
	init_waypoint_list(centralData.waypoint_list,&centralData.number_of_waypoints);
	
	return &centralData;
}

central_data_t* get_central_data(void)
{
	return &centralData;
}

Imu_Data_t* get_imu_data() {
	return &centralData.imu1;
}
Control_Command_t* get_control_inputs_data() {
	return &centralData.controls;
}