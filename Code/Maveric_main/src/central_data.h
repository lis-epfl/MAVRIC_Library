/*
 * central_data.h
 *
 * Created: 16/09/2013 12:18:00
 *  Author: sfx
 */ 


#ifndef CENTRAL_DATA_H_
#define CENTRAL_DATA_H_

#include <stdbool.h>
#include <stdint.h>

#include "gps_ublox.h"
#include "stabilisation.h"
#include "simulation.h"
#include "conf_sim_model.h"
#include "estimator.h"
#include "waypoint_navigation.h"
#include "bmp085.h"
#include "imu.h"

typedef struct  {
	Imu_Data_t imu1;
	Control_Command_t controls;
	Control_Command_t controls_nav;
	simulation_model_t uav_model;

	gps_Data_type GPS_data;
	
	Estimator_Data_t estimation;
	simulation_model_t sim_model;
	
	//local_coordinates_t local_position;
	bool init_gps_position;
	bool init_barometer;
	
	waypoint_struct waypoint_list[MAX_WAYPOINTS];
	uint16_t number_of_waypoints;
	int8_t current_wp;
	
	bool waypoint_set;
	bool mission_started;
	bool waypoint_sending;
	bool waypoint_receiving;
	bool waypoint_hold_init;
	
	uint8_t mav_mode;
	uint8_t mav_state;
	uint32_t simulation_mode;
	
	pressure_data pressure;
	//float pressure_filtered;
	//float altitude_filtered;
	
} central_data_t;


central_data_t* initialise_central_data(void);

central_data_t* get_central_data(void);

Imu_Data_t* get_imu_data();
Control_Command_t* get_control_inputs_data();

#endif /* CENTRAL_DATA_H_ */